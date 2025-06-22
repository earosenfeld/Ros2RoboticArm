# ROS 2 Robotic Arm Inspection System - Setup Guide

This guide will help you set up and run the complete ROS 2 Robotic Arm Inspection System with Drawflow integration.

## Prerequisites

### System Requirements
- Ubuntu 20.04 or later (recommended for ROS 2)
- Python 3.8+
- ROS 2 Humble or later
- Intel RealSense camera (optional - system works in simulation mode)

### Required Software
1. **ROS 2 Humble** - [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
2. **Python Dependencies** - Listed in `requirements.txt`
3. **Gazebo** (for simulation) - Usually comes with ROS 2
4. **RViz** (for visualization) - Usually comes with ROS 2

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd Ros2RoboticArm
```

### 2. Install Python Dependencies
```bash
pip install -r requirements.txt
```

### 3. Build the ROS 2 Workspace
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the package
colcon build

# Source the workspace
source install/setup.bash
```

### 4. Make Nodes Executable
```bash
chmod +x ros2_nodes/*.py
```

## Running the System

### Option 1: Complete System (Recommended)

#### Terminal 1: Start ROS 2 Nodes
```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch the inspection system
ros2 launch ros2_robotic_arm inspection_system.launch.py
```

#### Terminal 2: Start FastAPI Backend
```bash
# Navigate to project directory
cd Ros2RoboticArm

# Start the backend
python backend/main.py
```

#### Terminal 3: Serve Drawflow UI
```bash
# Navigate to project directory
cd Ros2RoboticArm

# Serve the UI (Python 3)
python -m http.server 8080 --directory drawflow_ui

# Or use any other web server
# nginx, apache, etc.
```

#### Terminal 4: Run Demo (Optional)
```bash
# Navigate to project directory
cd Ros2RoboticArm

# Run the demo script
python demo.py
```

### Option 2: Individual Components

#### Start Only ROS 2 Nodes
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start individual nodes
ros2 run ros2_robotic_arm robot_controller.py &
ros2 run ros2_robotic_arm camera_node.py &
ros2 run ros2_robotic_arm inspection_node.py &
```

#### Start Only Backend
```bash
python backend/main.py
```

## Using the System

### 1. Web Interface
1. Open your browser and go to `http://localhost:8080`
2. You'll see the Drawflow interface with robot nodes in the sidebar
3. Drag nodes from the sidebar to the canvas
4. Connect nodes to create inspection workflows
5. Click "Execute" to run the workflow

### 2. API Interface
The system provides a RESTful API at `http://localhost:8000`:

#### Health Check
```bash
curl http://localhost:8000/health
```

#### Get Example Workflow
```bash
curl http://localhost:8000/api/example-workflow
```

#### Execute Workflow
```bash
curl -X POST http://localhost:8000/api/execute-workflow \
  -H "Content-Type: application/json" \
  -d @workflow.json
```

### 3. ROS 2 Topics
Monitor the system using ROS 2 topics:

```bash
# Robot status
ros2 topic echo /robot_status

# Inspection results
ros2 topic echo /inspection_result

# Camera images
ros2 topic echo /camera/color/image_raw

# Drawflow commands
ros2 topic echo /drawflow_command
```

## Available Nodes

### Drawflow Nodes
1. **MoveToPose** - Move robot to specified position
   - Options: home, inspection_1, inspection_2, inspection_3
2. **SetGripper** - Control gripper position
   - Options: 0.0 (Closed), 1.0 (Open)
3. **CaptureImage** - Capture image from camera
   - Options: Save image (true/false)
4. **RunInspection** - Execute inspection routine
   - Options: standard, detailed, quick

### ROS 2 Nodes
1. **robot_controller** - Handles robot arm control and MoveIt 2 integration
2. **camera_node** - Manages RealSense camera and image processing
3. **inspection_node** - Coordinates inspection routines and workflow execution

## Configuration

### Robot Poses
Edit predefined poses in `ros2_nodes/inspection_node.py`:
```python
self.inspection_poses = {
    'home': self._create_pose(0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0),
    'inspection_1': self._create_pose(0.6, 0.2, 0.4, 0.0, 0.0, 0.0, 1.0),
    # Add more poses as needed
}
```

### Camera Settings
Modify camera parameters in `ros2_nodes/camera_node.py`:
```python
self.color_width = 640
self.color_height = 480
self.fps = 30
```

### Inspection Areas
Configure inspection areas in `ros2_nodes/camera_node.py`:
```python
self.inspection_areas = [
    {'name': 'area_1', 'x': 100, 'y': 100, 'w': 200, 'h': 200},
    # Add more areas as needed
]
```

## Troubleshooting

### Common Issues

#### 1. ROS 2 Nodes Not Starting
```bash
# Check if ROS 2 is properly sourced
echo $ROS_DISTRO

# Check if package is built
ros2 pkg list | grep ros2_robotic_arm
```

#### 2. FastAPI Backend Not Starting
```bash
# Check if port 8000 is available
netstat -tuln | grep 8000

# Check Python dependencies
pip list | grep fastapi
```

#### 3. Drawflow UI Not Loading
```bash
# Check if files exist
ls -la drawflow_ui/

# Check web server
curl http://localhost:8080
```

#### 4. Camera Not Working
- The system automatically falls back to simulation mode if RealSense camera is not available
- Check camera permissions: `ls -la /dev/video*`
- Install RealSense SDK if needed

### Debug Mode
Enable debug logging by setting environment variables:
```bash
export ROS_LOG_LEVEL=DEBUG
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
```

## Development

### Adding New Nodes
1. Create new node in `ros2_nodes/`
2. Register node in `drawflow_ui/script.js`
3. Add node configuration in `backend/main.py`
4. Update URDF if needed

### Extending Functionality
- Add new inspection algorithms in `camera_node.py`
- Create new robot poses in `inspection_node.py`
- Extend API endpoints in `backend/main.py`

## API Documentation

Once the backend is running, visit `http://localhost:8000/docs` for interactive API documentation.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review ROS 2 and FastAPI documentation
3. Check the logs in each terminal for error messages 