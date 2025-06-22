#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('ros2_robotic_arm')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_arm.urdf')
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_file, 'r').read()
        }]
    )
    
    # Joint state publisher (for simulation)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_default_positions': True,
            'publish_default_velocities': True,
            'publish_default_efforts': True
        }]
    )
    
    # Robot controller node
    robot_controller = Node(
        package='ros2_robotic_arm',
        executable='robot_controller.py',
        name='robot_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Camera node
    camera_node = Node(
        package='ros2_robotic_arm',
        executable='camera_node.py',
        name='camera_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Inspection node
    inspection_node = Node(
        package='ros2_robotic_arm',
        executable='inspection_node.py',
        name='inspection_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # RViz node
    rviz_config = os.path.join(pkg_share, 'config', 'inspection.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=use_rviz
    )
    
    # Gazebo launch (if using simulation)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        condition=use_gazebo
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'inspection_robot',
            '-file', urdf_file
        ],
        output='screen',
        condition=use_gazebo
    )
    
    # MoveIt 2 launch (if not using simulation)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moveit_ros_move_group'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_file, 'r').read()
        }],
        condition=use_sim_time
    )
    
    # TF2 static transform publisher for camera
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0.075', '0', '0', '0', '0', '0', 'link6', 'camera_color_optical_frame'],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='true',
            description='Use Gazebo simulation'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz'
        ),
        
        # Core nodes
        robot_state_publisher,
        joint_state_publisher,
        robot_controller,
        camera_node,
        inspection_node,
        camera_tf,
        
        # Optional nodes
        gazebo_launch,
        spawn_robot,
        moveit_launch,
        rviz_node,
    ]) 