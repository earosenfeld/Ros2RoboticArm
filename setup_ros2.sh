#!/bin/bash

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source the workspace
source install/setup.bash

# Add local bin to PATH for pip and virtualenv
export PATH="$HOME/.local/bin:$PATH"

echo "ROS 2 Humble workspace activated!"
echo "ROS 2 version: $(ros2 --version)"
echo "Available nodes:"
ros2 pkg executables ros2_robotic_arm
echo ""
echo "To run a node, use: ros2 run ros2_robotic_arm <node_name>"
echo "Example: ros2 run ros2_robotic_arm robot_controller.py" 