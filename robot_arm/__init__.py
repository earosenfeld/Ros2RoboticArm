"""robot_arm — pure-Python kinematics for the 6-DOF inspection arm.

This package is deliberately free of any ROS / rclpy dependency so the
kinematics, Jacobian, IK and trajectory generation can be imported and unit
tested with numpy/scipy/pytest alone. The ROS2 nodes in ``ros2_nodes/`` import
from here for their forward/inverse-kinematics math.
"""

from __future__ import annotations

from .urdf_chain import (
    DEFAULT_URDF_PATH,
    Joint,
    KinematicChain,
    Link,
    load_default_chain,
    load_full_chain,
    parse_urdf,
)
from .kinematics import (
    RobotKinematics,
    axis_angle_to_matrix,
    default_kinematics,
    forward_kinematics,
    geometric_jacobian,
    homogeneous,
    inverse_kinematics,
    joint_transform,
    link_frames,
    manipulability,
    rotation_matrix_to_axis_angle,
    rotation_x,
    rotation_y,
    rotation_z,
    rpy_to_matrix,
    transform_inverse,
    translation,
)
from .trajectory import (
    cartesian_line,
    quintic_polynomial,
    trapezoidal_profile,
)
from .planning import (
    Box,
    Sphere,
    in_collision,
    link_sample_points,
    path_length,
    plan_to_config,
    plan_to_pose,
    rrt_connect,
    shortcut_path,
)

__all__ = [
    # urdf_chain
    "DEFAULT_URDF_PATH",
    "Joint",
    "Link",
    "KinematicChain",
    "parse_urdf",
    "load_default_chain",
    "load_full_chain",
    # kinematics
    "RobotKinematics",
    "default_kinematics",
    "forward_kinematics",
    "link_frames",
    "geometric_jacobian",
    "inverse_kinematics",
    "manipulability",
    "joint_transform",
    "rpy_to_matrix",
    "axis_angle_to_matrix",
    "rotation_x",
    "rotation_y",
    "rotation_z",
    "rotation_matrix_to_axis_angle",
    "homogeneous",
    "translation",
    "transform_inverse",
    # trajectory
    "quintic_polynomial",
    "trapezoidal_profile",
    "cartesian_line",
    # planning
    "Sphere",
    "Box",
    "in_collision",
    "link_sample_points",
    "rrt_connect",
    "shortcut_path",
    "path_length",
    "plan_to_config",
    "plan_to_pose",
]
