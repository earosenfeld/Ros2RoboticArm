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

# Pure-Python kinematics (no ROS deps) — safe to import at module top so the FK
# math is available even when this node is imported outside a ROS runtime.
from robot_arm import RobotKinematics, load_default_chain


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
        
        # Forward/inverse kinematics over the 6-DOF arm chain (gripper locked).
        self.kinematics = RobotKinematics(load_default_chain())
        self.arm_joint_names = self.kinematics.chain.actuated_joint_names

        # Joint limits sourced from the URDF via the kinematic chain.
        lower, upper = self.kinematics.chain.joint_limits()
        self.joint_limits = {
            name: (float(lo), float(hi))
            for name, lo, hi in zip(self.arm_joint_names, lower, upper)
        }

        self.get_logger().info('Robot Controller initialized')

    # -- kinematics helpers ------------------------------------------------

    def _joint_vector_from_state(self, joint_state) -> np.ndarray:
        """Extract the actuated-joint vector (chain order) from a JointState msg.

        Missing joints default to 0.0 so partial states still yield a pose.
        """
        name_to_pos = dict(zip(joint_state.name, joint_state.position))
        return np.array(
            [name_to_pos.get(n, 0.0) for n in self.arm_joint_names], dtype=float
        )

    @staticmethod
    def _matrix_to_pose(T: np.ndarray) -> Pose:
        """Convert a 4x4 homogeneous transform to a geometry_msgs/Pose."""
        pose = Pose()
        pose.position.x = float(T[0, 3])
        pose.position.y = float(T[1, 3])
        pose.position.z = float(T[2, 3])
        # Rotation matrix -> quaternion (w, x, y, z).
        R = T[:3, :3]
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            i = int(np.argmax(np.diag(R)))
            if i == 0:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif i == 1:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        pose.orientation.w = float(w)
        pose.orientation.x = float(x)
        pose.orientation.y = float(y)
        pose.orientation.z = float(z)
        return pose

    @staticmethod
    def _pose_to_matrix(pose: Pose) -> np.ndarray:
        """Convert a geometry_msgs/Pose to a 4x4 homogeneous transform."""
        x = pose.orientation.x
        y = pose.orientation.y
        z = pose.orientation.z
        w = pose.orientation.w
        n = np.sqrt(x * x + y * y + z * z + w * w)
        if n > 0:
            x, y, z, w = x / n, y / n, z / n, w / n
        R = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=float,
        )
        T = np.eye(4)
        T[:3, :3] = R
        T[0, 3] = pose.position.x
        T[1, 3] = pose.position.y
        T[2, 3] = pose.position.z
        return T
        
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
        """Get the current end-effector pose via forward kinematics.

        Reads the latest joint state, maps it to the actuated-joint vector, and
        runs ``forward_kinematics`` on the 6-DOF arm chain (the gripper prismatic
        is locked at zero). Returns a base-frame ``geometry_msgs/Pose``.
        """
        try:
            if self.current_joint_state is None:
                return None

            q = self._joint_vector_from_state(self.current_joint_state)
            T = self.kinematics.forward_kinematics(q)
            return self._matrix_to_pose(T)

        except Exception as e:
            self.get_logger().error(f'Error getting current pose: {str(e)}')
            return None

    def move_to_cartesian(self, target_pose: Pose, planning_time: float = 5.0) -> bool:
        """Solve IK for ``target_pose`` and move the joints to the solution.

        Runs damped-least-squares inverse kinematics (warm-started from the
        current joint state) to find a joint configuration realizing the target
        end-effector pose, then commands the arm via :meth:`move_joints`.
        Returns ``False`` if IK fails to converge.
        """
        try:
            if self.current_joint_state is not None:
                q_init = self._joint_vector_from_state(self.current_joint_state)
            else:
                q_init = None

            T_target = self._pose_to_matrix(target_pose)
            q_sol, success, iters = self.kinematics.inverse_kinematics(
                T_target, q_init=q_init
            )

            if not success:
                self.get_logger().error(
                    f'IK did not converge for target pose (after {iters} iters)'
                )
                self.status_pub.publish(String(data='IK failed'))
                return False

            self.get_logger().info(
                f'IK solved in {iters} iters; commanding joints {q_sol.tolist()}'
            )
            return self.move_joints(q_sol.tolist(), self.arm_joint_names)

        except Exception as e:
            self.get_logger().error(f'Error in move_to_cartesian: {str(e)}')
            return False
    
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