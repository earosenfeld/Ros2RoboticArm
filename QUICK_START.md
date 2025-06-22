# 🚀 Quick Start Guide

## ✅ **System Status: READY TO SETUP!**

Your ROS 2 Robotic Arm Inspection System is ready to be set up with a virtual environment!

## 🛠️ **Setup Instructions**

### **Option 1: Automated Setup (Recommended)**

Run the automated setup script:

```bash
# Make the script executable (if needed)
chmod +x setup.sh

# Run the setup script
./setup.sh
```

This will:
- ✅ Create a Python virtual environment
- ✅ Install all required dependencies
- ✅ Create convenient start/test scripts
- ✅ Set up the complete system

### **Option 2: Manual Setup**

If you prefer manual setup:

```bash
# 1. Create virtual environment
python3 -m venv venv

# 2. Activate virtual environment
source venv/bin/activate

# 3. Upgrade pip
pip install --upgrade pip

# 4. Install dependencies
pip install -r requirements_simple.txt
```

## 🚀 **Starting the System**

### **Option 1: Automated Start**

After setup, use the automated start script:

```bash
./start_system.sh
```

This will start both backend and frontend servers automatically.

### **Option 2: Manual Start**

Start the servers manually:

```bash
# Activate virtual environment
source venv/bin/activate

# Terminal 1: Start backend
cd backend
python -m uvicorn main_simple:app --host 0.0.0.0 --port 8000 --reload

# Terminal 2: Start frontend
cd drawflow_ui
python -m http.server 8080
```

## 🧪 **Testing the System**

### **Automated Test**

```bash
./test_system.sh
```

### **Manual Test**

```bash
# Activate virtual environment
source venv/bin/activate

# Run the demo
python demo_simple.py
```

## 🌐 **Access Points**

Once the system is running:

- **🎨 Frontend (Drawflow UI)**: http://localhost:8080
- **🤖 3D Robot Visualizer**: http://localhost:8080/robot_visualizer.html
- **🔧 Backend API**: http://localhost:8000
- **📚 API Documentation**: http://localhost:8000/docs
- **💚 Health Check**: http://localhost:8000/health

## 🎮 **How to Use**

#### **1. Open the Web Interface**
Open your browser and go to: **http://localhost:8080**

You'll see the Drawflow visual programming interface with:
- **MoveToPose** nodes (robot movement)
- **SetGripper** nodes (gripper control)
- **CaptureImage** nodes (camera capture)
- **RunInspection** nodes (defect detection)

#### **2. Create a Workflow**
1. **Drag nodes** from the left panel to the canvas
2. **Connect nodes** by dragging from output to input
3. **Configure nodes** by double-clicking them
4. **Save your workflow** using the save button

#### **3. Execute Your Workflow**
1. Click the **"Execute Workflow"** button
2. Watch the **real-time status updates** in the console
3. See the **simulated robot** perform your commands

#### **4. Monitor Results**
- Check the **console output** for detailed logs
- View **inspection results** in the status panel
- Monitor **robot status** via WebSocket updates

### 🤖 **3D Robot Visualizer**

#### **Open the 3D Visualizer**
Click the **"3D Robot View"** button in the main interface, or go directly to:
**http://localhost:8080/robot_visualizer.html**

#### **Features of the 3D Visualizer**
- **🎯 Real-time 3D Robot Model**: See a detailed 6-DOF robotic arm
- **🔄 Live Movement**: Watch the robot move in real-time as commands execute
- **🎮 Interactive Controls**: Manually control the robot with buttons and sliders
- **📡 WebSocket Integration**: Syncs with the backend for real-time updates
- **📊 Status Monitoring**: Real-time status display with connection indicators
- **🎨 Visual Effects**: Camera flash effects and inspection animations

#### **Using the 3D Visualizer**
1. **Manual Control**: Use the controls panel to move the robot manually
2. **Pose Selection**: Choose from predefined poses (home, inspection, pick, place, camera)
3. **Gripper Control**: Adjust gripper position with the slider
4. **Real-time Sync**: Watch the robot respond to workflow execution from the main interface
5. **Connection Status**: Monitor backend connection with the status indicator

#### **3D Robot Features**
- **6-DOF Arm**: Base, shoulder, elbow, wrist1, wrist2, wrist3 joints
- **Gripper**: Animated gripper with realistic finger movement
- **Shadows & Lighting**: Professional 3D rendering with shadows
- **Smooth Animation**: Interpolated movement between poses
- **Multiple Poses**: 5 predefined poses for different tasks

## 🔧 **API Testing**

Run the demo script to test all API endpoints:

```bash
# Activate virtual environment first
source venv/bin/activate

# Run the demo
python demo_simple.py
```

This will test:
- ✅ Health and status endpoints
- ✅ Single command execution
- ✅ Complete routine execution
- ✅ Workflow execution
- ✅ WebSocket real-time updates

## 📋 **Available Commands**

#### **MoveToPose**
```json
{
  "type": "MoveToPose",
  "data": {"pose": "home"},
  "routine_id": "demo_001"
}
```

#### **SetGripper**
```json
{
  "type": "SetGripper",
  "data": {"position": 1.0},
  "routine_id": "demo_001"
}
```

#### **CaptureImage**
```json
{
  "type": "CaptureImage",
  "data": {"save": true},
  "routine_id": "demo_001"
}
```

#### **RunInspection**
```json
{
  "type": "RunInspection",
  "data": {"type": "standard"},
  "routine_id": "demo_001"
}
```

## 🎯 **Example Workflow**

The system includes a pre-built example workflow that:
1. Moves to home position
2. Opens the gripper
3. Captures an image
4. Runs standard inspection

## 🔍 **What's Simulated**

- **🤖 Robot Movement**: 2-second delays simulate real movement
- **📸 Camera Capture**: 1-second delays simulate image capture
- **🔍 Inspection**: 3-second delays with random defect detection
- **📊 Results**: Simulated inspection results with random defects
- **🎨 3D Visualization**: Real-time 3D robot model with smooth animations

## 🛑 **Stopping the System**

### **Automated Start**
If you used `./start_system.sh`, press **Ctrl+C** to stop both servers.

### **Manual Start**
To stop the servers:
1. Press **Ctrl+C** in the terminal running the backend
2. Press **Ctrl+C** in the terminal running the frontend

## 🔄 **Restarting**

### **Automated Restart**
```bash
./start_system.sh
```

### **Manual Restart**
```bash
# Activate virtual environment
source venv/bin/activate

# Terminal 1: Start backend
cd backend
python -m uvicorn main_simple:app --host 0.0.0.0 --port 8000 --reload

# Terminal 2: Start frontend
cd drawflow_ui
python -m http.server 8080
```

## 🚀 **Next Steps**

1. **Explore the UI**: Try creating different workflows
2. **Test the 3D Visualizer**: Open the robot visualizer and experiment with controls
3. **Test the API**: Use the demo script or curl commands
4. **Check Documentation**: Visit http://localhost:8000/docs
5. **Install ROS 2**: For full robot control (see SETUP_GUIDE.md)

## 🆘 **Troubleshooting**

### **Virtual Environment Issues**
- **venv not found**: Run `python3 -m venv venv` to create it
- **Activation fails**: Use `source venv/bin/activate` (not `venv/bin/activate`)
- **Dependencies missing**: Run `pip install -r requirements_simple.txt`

### **Server Issues**
- **Frontend not loading**: Check if port 8080 is available
- **Backend not responding**: Check if port 8000 is available
- **WebSocket errors**: Ensure both servers are running
- **Node connection issues**: Try refreshing the page
- **3D Visualizer not loading**: Check browser console for Three.js errors
- **WebSocket disconnection**: The visualizer will auto-reconnect every 5 seconds

### **Permission Issues**
- **Script not executable**: Run `chmod +x setup.sh start_system.sh test_system.sh`
- **Port access denied**: Check if ports are already in use

## 📞 **Support**

- Check the logs in the terminal for detailed error messages
- Review the full documentation in `SETUP_GUIDE.md`
- API documentation available at http://localhost:8000/docs
- 3D Visualizer shows connection status in real-time

## 📁 **Project Structure**

```
Ros2RoboticArm/
├── venv/                    # Virtual environment (created by setup)
├── backend/                 # FastAPI backend
│   └── main_simple.py      # Main backend application
├── drawflow_ui/            # Frontend interface
│   ├── index.html          # Main Drawflow interface
│   ├── robot_visualizer.html # 3D robot visualizer
│   ├── script.js           # Frontend JavaScript
│   └── styles.css          # Frontend styles
├── setup.sh                # Automated setup script
├── start_system.sh         # Automated start script
├── test_system.sh          # Automated test script
├── demo_simple.py          # Demo script
├── requirements_simple.txt # Python dependencies
├── QUICK_START.md          # This file
└── SYSTEM_SUMMARY.md       # Complete system overview
```

---

**🎉 Enjoy your ROS 2 Robotic Arm Inspection System with 3D Visualization!** 