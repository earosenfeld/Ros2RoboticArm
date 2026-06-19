#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import asyncio
import websockets
import threading
import time
from datetime import datetime

import numpy as np

# Pure-Python kinematics (no ROS deps) so the 3D visualizer reflects the true
# forward kinematics of the arm rather than a hardcoded marker.
from robot_arm import RobotKinematics, load_default_chain

class MockRobotNode(Node):
    """
    Mock robot node that simulates robot movements and updates the 3D visualizer.
    This node listens to command topics and sends status updates via WebSocket.
    """
    
    def __init__(self):
        super().__init__('mock_robot_node')
        
        # Publishers
        self.robot_status_pub = self.create_publisher(
            String,
            '/robot_status',
            10
        )
        
        self.inspection_status_pub = self.create_publisher(
            String,
            '/inspection_status',
            10
        )
        
        # Subscribers
        self.robot_command_sub = self.create_subscription(
            String,
            '/robot_command',
            self.robot_command_callback,
            10
        )
        
        self.capture_image_sub = self.create_subscription(
            String,
            '/capture_image',
            self.capture_image_callback,
            10
        )
        
        self.run_inspection_sub = self.create_subscription(
            String,
            '/run_inspection',
            self.run_inspection_callback,
            10
        )
        
        # Forward kinematics over the 6-DOF arm chain (gripper locked at zero).
        self.kinematics = RobotKinematics(load_default_chain())

        # Named joint configurations (rad), used to drive FK for the 3D viz.
        self.named_configs = {
            'home': np.zeros(self.kinematics.n),
            'ready': np.array([0.0, -0.5, 0.8, 0.0, 0.6, 0.0]),
            'inspect': np.array([0.5, -0.3, 0.6, 0.0, 0.9, 0.0]),
            'pick': np.array([0.0, 0.7, -1.0, 0.0, 0.8, 0.0]),
        }

        # Robot state
        self.current_pose = 'home'
        self.current_joint_positions = self.named_configs['home'].copy()
        self.gripper_position = 0.0
        self.is_moving = False
        self.robot_status = 'Ready'
        
        # WebSocket server for 3D visualizer
        self.websocket_server = None
        self.websocket_clients = []
        
        # Start WebSocket server in a separate thread
        self.websocket_thread = threading.Thread(target=self._start_websocket_server)
        self.websocket_thread.daemon = True
        self.websocket_thread.start()
        
        self.get_logger().info('Mock Robot Node initialized')
    
    def _start_websocket_server(self):
        """Start WebSocket server for 3D visualizer communication."""
        async def websocket_handler(websocket, path):
            self.websocket_clients.append(websocket)
            self.get_logger().info(f'3D Visualizer connected. Total clients: {len(self.websocket_clients)}')
            
            try:
                # Send initial state
                await self._send_robot_status(websocket)
                
                # Keep connection alive
                async for message in websocket:
                    # Handle any messages from visualizer if needed
                    pass
            except websockets.exceptions.ConnectionClosed:
                pass
            finally:
                if websocket in self.websocket_clients:
                    self.websocket_clients.remove(websocket)
                self.get_logger().info(f'3D Visualizer disconnected. Total clients: {len(self.websocket_clients)}')
        
        async def start_server():
            self.websocket_server = await websockets.serve(
                websocket_handler, 
                "localhost", 
                8001  # Different port for robot status
            )
            await self.websocket_server.wait_closed()
        
        asyncio.run(start_server())
    
    def _end_effector_pose(self):
        """Compute the current end-effector pose via forward kinematics.

        Returns a dict with the base-frame position (x, y, z) and the full 3x3
        orientation, so the 3D visualizer can place the tool frame using the
        arm's true kinematics rather than a hardcoded marker.
        """
        T = self.kinematics.forward_kinematics(self.current_joint_positions)
        return {
            'position': {
                'x': float(T[0, 3]),
                'y': float(T[1, 3]),
                'z': float(T[2, 3]),
            },
            'orientation_matrix': T[:3, :3].tolist(),
            'joint_positions': self.current_joint_positions.tolist(),
        }

    async def _send_robot_status(self, websocket):
        """Send robot status to 3D visualizer."""
        status_data = {
            'type': 'robot_status',
            'status': self.robot_status,
            'current_pose': self.current_pose,
            'gripper_position': self.gripper_position,
            'is_moving': self.is_moving,
            'end_effector': self._end_effector_pose(),
            'timestamp': datetime.now().isoformat()
        }
        
        try:
            await websocket.send(json.dumps(status_data))
        except Exception as e:
            self.get_logger().error(f'Error sending status to visualizer: {e}')
    
    def _broadcast_status(self):
        """Broadcast robot status to all connected visualizers."""
        if not self.websocket_clients:
            return
        
        status_data = {
            'type': 'robot_status',
            'status': self.robot_status,
            'current_pose': self.current_pose,
            'gripper_position': self.gripper_position,
            'is_moving': self.is_moving,
            'end_effector': self._end_effector_pose(),
            'timestamp': datetime.now().isoformat()
        }

        # Send to all connected clients
        for client in self.websocket_clients[:]:  # Copy list to avoid modification during iteration
            try:
                asyncio.run_coroutine_threadsafe(
                    client.send(json.dumps(status_data)), 
                    asyncio.get_event_loop()
                )
            except Exception as e:
                self.get_logger().error(f'Error broadcasting status: {e}')
                if client in self.websocket_clients:
                    self.websocket_clients.remove(client)
    
    def robot_command_callback(self, msg):
        """Callback for robot commands."""
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get('type')
            
            self.get_logger().info(f'Received robot command: {command_type}')
            
            if command_type == 'MoveToPose':
                self._execute_move_to_pose(command_data)
            elif command_type == 'SetGripper':
                self._execute_set_gripper(command_data)
            else:
                self.get_logger().warn(f'Unknown robot command type: {command_type}')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in robot command: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing robot command: {e}')
    
    def capture_image_callback(self, msg):
        """Callback for capture image commands."""
        command = msg.data
        self.get_logger().info(f'Capturing image: {command}')
        
        # Simulate image capture
        self.robot_status = 'Capturing image'
        self._broadcast_status()
        
        # Simulate processing time
        time.sleep(1.0)
        
        self.robot_status = 'Ready'
        self._broadcast_status()
        
        # Publish status update
        status_data = {
            'type': 'image_captured',
            'timestamp': datetime.now().isoformat(),
            'saved': command == 'save'
        }
        self.inspection_status_pub.publish(String(data=json.dumps(status_data)))
    
    def run_inspection_callback(self, msg):
        """Callback for run inspection commands."""
        try:
            command_data = json.loads(msg.data)
            inspection_type = command_data.get('inspection_type', 'standard')
            
            self.get_logger().info(f'Running inspection: {inspection_type}')
            
            # Simulate inspection process
            self.robot_status = f'Running {inspection_type} inspection'
            self._broadcast_status()
            
            # Simulate inspection movements
            time.sleep(2.0)
            
            # Simulate inspection result
            result = {
                'type': 'inspection_result',
                'inspection_type': inspection_type,
                'overall_result': 'PASS',
                'defects_found': 0,
                'confidence': 0.95,
                'timestamp': datetime.now().isoformat()
            }
            
            self.robot_status = 'Ready'
            self._broadcast_status()
            
            # Publish result
            self.inspection_status_pub.publish(String(data=json.dumps(result)))
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in inspection command: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing inspection command: {e}')
    
    def _execute_move_to_pose(self, command_data):
        """Execute MoveToPose command."""
        pose_name = command_data.get('pose', 'home')
        
        self.get_logger().info(f'Moving to pose: {pose_name}')
        
        # Update robot state
        self.is_moving = True
        self.robot_status = f'Moving to {pose_name}'
        self._broadcast_status()
        
        # Simulate movement time
        time.sleep(2.0)

        # Update pose and the joint configuration that drives forward kinematics.
        self.current_pose = pose_name
        if pose_name in self.named_configs:
            self.current_joint_positions = self.named_configs[pose_name].copy()
        self.is_moving = False
        self.robot_status = 'Ready'
        self._broadcast_status()

        # Publish status update including the FK end-effector pose.
        status_data = {
            'type': 'pose_reached',
            'pose': pose_name,
            'end_effector': self._end_effector_pose(),
            'timestamp': datetime.now().isoformat()
        }
        self.robot_status_pub.publish(String(data=json.dumps(status_data)))
    
    def _execute_set_gripper(self, command_data):
        """Execute SetGripper command."""
        position = command_data.get('position', 0.0)
        
        self.get_logger().info(f'Setting gripper to position: {position}')
        
        # Update gripper position
        self.gripper_position = position
        self._broadcast_status()
        
        # Publish status update
        status_data = {
            'type': 'gripper_set',
            'position': position,
            'timestamp': datetime.now().isoformat()
        }
        self.robot_status_pub.publish(String(data=json.dumps(status_data)))

def main(args=None):
    rclpy.init(args=args)
    
    node = MockRobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 