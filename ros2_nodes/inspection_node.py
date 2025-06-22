#!/usr/bin/env python3

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor

    from std_msgs.msg import String, Bool
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import Image
    
    ROS2_AVAILABLE = True
except ImportError:
    # ROS 2 not available - create mock classes for simulation
    class Node:
        def __init__(self, name):
            self.name = name
            self.logger = MockLogger()
        
        def get_logger(self):
            return self.logger
    
    class ReentrantCallbackGroup:
        pass
    
    class MultiThreadedExecutor:
        def __init__(self):
            pass
        def add_node(self, node):
            pass
        def spin(self):
            pass
    
    class MockLogger:
        def info(self, msg): print(f"INFO: {msg}")
        def error(self, msg): print(f"ERROR: {msg}")
        def warn(self, msg): print(f"WARN: {msg}")
    
    class String:
        def __init__(self, data=""):
            self.data = data
    
    class Bool:
        def __init__(self, data=False):
            self.data = data
    
    class Pose:
        def __init__(self):
            self.position = MockPosition()
            self.orientation = MockOrientation()
    
    class MockPosition:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
    
    class MockOrientation:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0
    
    class Image:
        pass
    
    ROS2_AVAILABLE = False

import json
import time
from datetime import datetime
from typing import List, Dict, Any


class InspectionNode(Node):
    """
    ROS 2 node for coordinating inspection routines.
    Receives commands from FastAPI backend and orchestrates robot and camera operations.
    """
    
    def __init__(self):
        super().__init__('inspection_node')
        
        # Initialize callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Publishers
        self.robot_command_pub = self.create_publisher(
            String,
            '/robot_command',
            10
        )
        
        self.camera_command_pub = self.create_publisher(
            String,
            '/capture_image',
            10
        )
        
        self.inspection_command_pub = self.create_publisher(
            String,
            '/run_inspection',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/inspection_status',
            10
        )
        
        # Subscribers
        self.robot_status_sub = self.create_subscription(
            String,
            '/robot_status',
            self.robot_status_callback,
            10
        )
        
        self.inspection_result_sub = self.create_subscription(
            String,
            '/inspection_result',
            self.inspection_result_callback,
            10
        )
        
        self.drawflow_command_sub = self.create_subscription(
            String,
            '/drawflow_command',
            self.drawflow_command_callback,
            10
        )
        
        # State variables
        self.current_routine = None
        self.routine_steps = []
        self.current_step = 0
        self.is_executing = False
        self.robot_ready = True
        self.last_inspection_result = None
        
        # Predefined inspection poses
        self.inspection_poses = {
            'home': self._create_pose(0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0),
            'inspection_1': self._create_pose(0.6, 0.2, 0.4, 0.0, 0.0, 0.0, 1.0),
            'inspection_2': self._create_pose(0.6, -0.2, 0.4, 0.0, 0.0, 0.0, 1.0),
            'inspection_3': self._create_pose(0.7, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0),
            'gripper_open': self._create_pose(0.5, 0.0, 0.6, 0.0, 0.0, 0.0, 1.0),
            'gripper_close': self._create_pose(0.5, 0.0, 0.6, 0.0, 0.0, 0.0, 1.0)
        }
        
        # Timer for routine execution
        self.execution_timer = None
        
        self.get_logger().info('Inspection Node initialized')
        
    def _create_pose(self, x, y, z, qx, qy, qz, qw):
        """Create a Pose message."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose
    
    def drawflow_command_callback(self, msg):
        """Callback for Drawflow-generated commands from FastAPI backend."""
        try:
            command_data = json.loads(msg.data)
            self.get_logger().info(f'Received Drawflow command: {command_data}')
            
            # Parse the command and execute
            self._execute_drawflow_command(command_data)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in Drawflow command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error processing Drawflow command: {str(e)}')
    
    def _execute_drawflow_command(self, command_data):
        """Execute a Drawflow-generated command."""
        command_type = command_data.get('type')
        
        if command_type == 'routine':
            # Execute a complete inspection routine
            self._execute_inspection_routine(command_data.get('steps', []))
        elif command_type == 'MoveToPose':
            self._execute_move_to_pose(command_data)
        elif command_type == 'SetGripper':
            self._execute_set_gripper(command_data)
        elif command_type == 'CaptureImage':
            self._execute_capture_image(command_data)
        elif command_type == 'RunInspection':
            self._execute_run_inspection(command_data)
        else:
            self.get_logger().warn(f'Unknown command type: {command_type}')
    
    def _execute_inspection_routine(self, steps):
        """Execute a complete inspection routine."""
        if self.is_executing:
            self.get_logger().warn('Routine already executing')
            return
        
        self.current_routine = {
            'id': f"routine_{int(time.time())}",
            'steps': steps,
            'start_time': datetime.now().isoformat()
        }
        self.routine_steps = steps
        self.current_step = 0
        self.is_executing = True
        
        self.get_logger().info(f'Starting inspection routine with {len(steps)} steps')
        self.status_pub.publish(String(data=json.dumps({
            'status': 'started',
            'routine_id': self.current_routine['id'],
            'total_steps': len(steps)
        })))
        
        # Start execution
        self._execute_next_step()
    
    def _execute_next_step(self):
        """Execute the next step in the routine."""
        if not self.is_executing or self.current_step >= len(self.routine_steps):
            self._finish_routine()
            return
        
        if not self.robot_ready:
            # Wait for robot to be ready
            self.create_timer(0.5, self._check_robot_ready)
            return
        
        step = self.routine_steps[self.current_step]
        self.get_logger().info(f'Executing step {self.current_step + 1}: {step}')
        
        # Execute the step
        self._execute_drawflow_command(step)
        
        # Move to next step after a delay
        self.execution_timer = self.create_timer(2.0, self._step_complete)
    
    def _step_complete(self):
        """Called when a step is complete."""
        if self.execution_timer:
            self.destroy_timer(self.execution_timer)
            self.execution_timer = None
        
        self.current_step += 1
        self._execute_next_step()
    
    def _check_robot_ready(self):
        """Check if robot is ready for next command."""
        if self.robot_ready:
            self.destroy_timer(self._check_robot_ready)
            self._execute_next_step()
    
    def _finish_routine(self):
        """Finish the current routine."""
        self.is_executing = False
        self.current_routine = None
        self.routine_steps = []
        self.current_step = 0
        
        self.get_logger().info('Inspection routine completed')
        self.status_pub.publish(String(data=json.dumps({
            'status': 'completed',
            'result': self.last_inspection_result
        })))
    
    def _execute_move_to_pose(self, command_data):
        """Execute MoveToPose command."""
        pose_name = command_data.get('pose', 'home')
        pose = self.inspection_poses.get(pose_name)
        
        if pose:
            command = {
                'type': 'MoveToPose',
                'pose': pose_name,
                'pose_data': {
                    'position': {
                        'x': pose.position.x,
                        'y': pose.position.y,
                        'z': pose.position.z
                    },
                    'orientation': {
                        'x': pose.orientation.x,
                        'y': pose.orientation.y,
                        'z': pose.orientation.z,
                        'w': pose.orientation.w
                    }
                }
            }
            
            self.robot_command_pub.publish(String(data=json.dumps(command)))
            self.get_logger().info(f'Moving to pose: {pose_name}')
        else:
            self.get_logger().error(f'Unknown pose: {pose_name}')
    
    def _execute_set_gripper(self, command_data):
        """Execute SetGripper command."""
        position = command_data.get('position', 0.0)  # 0.0 = closed, 1.0 = open
        
        command = {
            'type': 'SetGripper',
            'position': position
        }
        
        self.robot_command_pub.publish(String(data=json.dumps(command)))
        self.get_logger().info(f'Setting gripper to position: {position}')
    
    def _execute_capture_image(self, command_data):
        """Execute CaptureImage command."""
        save_image = command_data.get('save', True)
        
        command = "save" if save_image else "capture"
        self.camera_command_pub.publish(String(data=command))
        self.get_logger().info('Capturing image')
    
    def _execute_run_inspection(self, command_data):
        """Execute RunInspection command."""
        inspection_type = command_data.get('type', 'standard')
        
        command = {
            'type': 'RunInspection',
            'inspection_type': inspection_type
        }
        
        self.inspection_command_pub.publish(String(data=json.dumps(command)))
        self.get_logger().info(f'Running inspection: {inspection_type}')
    
    def robot_status_callback(self, msg):
        """Callback for robot status updates."""
        try:
            status_data = json.loads(msg.data)
            self.robot_ready = status_data.get('ready', True)
            
            if self.robot_ready and self.is_executing:
                self.get_logger().info('Robot ready for next command')
                
        except json.JSONDecodeError:
            # Simple string status
            self.robot_ready = msg.data in ['Ready', 'Movement completed']
    
    def inspection_result_callback(self, msg):
        """Callback for inspection results."""
        try:
            result = json.loads(msg.data)
            self.last_inspection_result = result
            
            self.get_logger().info(f'Inspection result: {result.get("overall_result", "UNKNOWN")}')
            
            # Publish result to backend
            self.status_pub.publish(String(data=json.dumps({
                'type': 'inspection_result',
                'data': result
            })))
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in inspection result: {str(e)}')
    
    def get_example_routine(self):
        """Get an example inspection routine."""
        return {
            'name': 'Example Inspection Routine',
            'steps': [
                {
                    'type': 'MoveToPose',
                    'pose': 'home',
                    'description': 'Move to home position'
                },
                {
                    'type': 'SetGripper',
                    'position': 1.0,
                    'description': 'Open gripper'
                },
                {
                    'type': 'MoveToPose',
                    'pose': 'inspection_1',
                    'description': 'Move to inspection position 1'
                },
                {
                    'type': 'CaptureImage',
                    'save': True,
                    'description': 'Capture image from position 1'
                },
                {
                    'type': 'RunInspection',
                    'type': 'standard',
                    'description': 'Run standard inspection'
                },
                {
                    'type': 'MoveToPose',
                    'pose': 'inspection_2',
                    'description': 'Move to inspection position 2'
                },
                {
                    'type': 'CaptureImage',
                    'save': True,
                    'description': 'Capture image from position 2'
                },
                {
                    'type': 'MoveToPose',
                    'pose': 'home',
                    'description': 'Return to home position'
                },
                {
                    'type': 'SetGripper',
                    'position': 0.0,
                    'description': 'Close gripper'
                }
            ]
        }


def main(args=None):
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        
        inspection_node = InspectionNode()
        
        # Use multi-threaded executor
        executor = MultiThreadedExecutor()
        executor.add_node(inspection_node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            inspection_node.destroy_node()
            rclpy.shutdown()
    else:
        # Simulation mode
        print("🤖 Running Inspection Node in Simulation Mode")
        print("📝 Note: ROS 2 not available - using mock implementation")
        
        inspection_node = InspectionNode()
        
        try:
            # Keep the node running for simulation
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("🛑 Simulation stopped")


if __name__ == '__main__':
    main() 