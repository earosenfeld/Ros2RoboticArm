#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.action import MoveGroup
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPlanningScene

import numpy as np
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_pose


class RobotController(Node):
    """
    ROS 2 node for controlling a 6-DOF robotic arm using MoveIt 2.
    Handles motion planning, trajectory execution, and gripper control.
    """
    
    def __init__(self):
        super().__init__('robot_controller')
        
        # Initialize callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Action clients
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            'move_group',
            callback_group=self.callback_group
        )
        
        # Publishers
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/robot_status',
            10
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Service clients
        self.planning_scene_client = self.create_client(
            GetPlanningScene,
            'move_group/get_planning_scene'
        )
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Robot state
        self.current_joint_state = None
        self.is_moving = False
        
        # Joint limits (example for a 6-DOF arm)
        self.joint_limits = {
            'joint1': (-3.14, 3.14),
            'joint2': (-3.14, 3.14),
            'joint3': (-3.14, 3.14),
            'joint4': (-3.14, 3.14),
            'joint5': (-3.14, 3.14),
            'joint6': (-3.14, 3.14)
        }
        
        self.get_logger().info('Robot Controller initialized')
        
    def joint_state_callback(self, msg):
        """Callback for joint state updates."""
        self.current_joint_state = msg
        
    def move_to_pose(self, pose: Pose, planning_time: float = 5.0) -> bool:
        """
        Move the robot to a specific pose using MoveIt 2.
        
        Args:
            pose: Target pose in base frame
            planning_time: Maximum planning time in seconds
            
        Returns:
            bool: True if movement was successful
        """
        try:
            self.get_logger().info(f'Moving to pose: {pose}')
            self.is_moving = True
            self.status_pub.publish(String(data='Moving to target pose'))
            
            # Create MoveGroup goal
            goal = MoveGroup.Goal()
            goal.request.group_name = "arm"
            goal.request.num_planning_attempts = 10
            goal.request.allowed_planning_time = planning_time
            goal.request.max_velocity_scaling_factor = 0.5
            goal.request.max_acceleration_scaling_factor = 0.5
            
            # Set target pose
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = pose
            goal.request.target_poses = [pose_stamped]
            
            # Send goal
            self.move_group_client.wait_for_server()
            future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.get_logger().error('Goal rejected')
                    self.is_moving = False
                    return False
                
                # Wait for result
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                
                if result_future.result() is not None:
                    result = result_future.result().result
                    if result.error_code.val == 1:  # SUCCESS
                        self.get_logger().info('Movement completed successfully')
                        self.status_pub.publish(String(data='Movement completed'))
                        self.is_moving = False
                        return True
                    else:
                        self.get_logger().error(f'Movement failed: {result.error_code}')
                        self.status_pub.publish(String(data='Movement failed'))
                        self.is_moving = False
                        return False
            
            self.is_moving = False
            return False
            
        except Exception as e:
            self.get_logger().error(f'Error in move_to_pose: {str(e)}')
            self.is_moving = False
            return False
    
    def move_joints(self, joint_positions: list, joint_names: list = None) -> bool:
        """
        Move robot joints to specific positions.
        
        Args:
            joint_positions: List of joint positions
            joint_names: List of joint names (optional)
            
        Returns:
            bool: True if movement was successful
        """
        try:
            if joint_names is None:
                joint_names = [f'joint{i+1}' for i in range(len(joint_positions))]
            
            self.get_logger().info(f'Moving joints: {joint_names} to {joint_positions}')
            self.status_pub.publish(String(data='Moving joints'))
            
            # Create trajectory message
            trajectory = JointTrajectory()
            trajectory.joint_names = joint_names
            
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start.sec = 2
            trajectory.points = [point]
            
            # Publish trajectory
            self.joint_trajectory_pub.publish(trajectory)
            
            # Wait for execution (simplified)
            self.create_timer(2.0, self._check_movement_complete)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error in move_joints: {str(e)}')
            return False
    
    def set_gripper(self, position: float) -> bool:
        """
        Control the gripper position.
        
        Args:
            position: Gripper position (0.0 = closed, 1.0 = open)
            
        Returns:
            bool: True if command was sent successfully
        """
        try:
            self.get_logger().info(f'Setting gripper to position: {position}')
            
            # Create gripper trajectory
            trajectory = JointTrajectory()
            trajectory.joint_names = ['gripper_joint']
            
            point = JointTrajectoryPoint()
            point.positions = [position]
            point.time_from_start.sec = 1
            trajectory.points = [point]
            
            # Publish trajectory
            self.gripper_pub.publish(trajectory)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error in set_gripper: {str(e)}')
            return False
    
    def get_current_pose(self) -> Pose:
        """Get the current end-effector pose."""
        try:
            if self.current_joint_state is None:
                return None
            
            # This would typically involve forward kinematics
            # For now, return a default pose
            pose = Pose()
            pose.position.x = 0.5
            pose.position.y = 0.0
            pose.position.z = 0.5
            pose.orientation.w = 1.0
            
            return pose
            
        except Exception as e:
            self.get_logger().error(f'Error getting current pose: {str(e)}')
            return None
    
    def _check_movement_complete(self):
        """Check if movement is complete and update status."""
        self.is_moving = False
        self.status_pub.publish(String(data='Ready'))
        self.destroy_timer(self._check_movement_complete)
    
    def is_ready(self) -> bool:
        """Check if the robot is ready for new commands."""
        return not self.is_moving


def main(args=None):
    rclpy.init(args=args)
    
    robot_controller = RobotController()
    
    # Use multi-threaded executor for concurrent operations
    executor = MultiThreadedExecutor()
    executor.add_node(robot_controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 