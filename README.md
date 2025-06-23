# 🤖 ROS 2 Robotic Arm Inspection System

A comprehensive Python-based robotic inspection system featuring visual programming, 3D robot visualization, and real-time simulation.

## 🎯 **Features**

- **🎨 Visual Programming Interface** - Drag-and-drop workflow creation with Drawflow
- **🤖 3D Robot Visualization** - Real-time 3D robotic arm with Three.js
- **🔧 FastAPI Backend** - REST API and WebSocket communication
- **📡 Real-time Updates** - Live status monitoring and synchronization
- **🎮 Interactive Controls** - Manual robot operation and pose selection
- **📊 Status Monitoring** - Comprehensive system status display

## 🚀 **Quick Start**

### **Prerequisites**
- Python 3.8 or higher
- Git
- Web browser (Chrome, Firefox, Safari, Edge)

### **1. Automated Setup (Recommended)**

```bash
# Clone the repository
git clone <repository-url>
cd Ros2RoboticArm

# Run automated setup
chmod +x setup.sh
./setup.sh
```

### **2. Start the System**

**Option A: Automated Start (Recommended)**
```bash
# This starts both backend and frontend servers
./start_system.sh
```

**Option B: Manual Start (If you encounter port conflicts)**
```bash
# First, check if ports are in use
lsof -i :8000  # Check backend port
lsof -i :8080  # Check frontend port

# Kill any processes using these ports if needed
sudo kill -9 $(lsof -t -i:8000)  # Kill backend port
sudo kill -9 $(lsof -t -i:8080)  # Kill frontend port

# Then start the system
./start_system.sh
```

### **3. Access the System**

Once started successfully, you'll see:
```
✅ System started successfully!
🌐 Access points:
   • Main Interface: http://localhost:8080
   • 3D Robot Visualizer: http://localhost:8080/robot_visualizer.html
   • Backend API: http://localhost:8000
   • API Documentation: http://localhost:8000/docs
```

Open your browser and navigate to:
- **Main Interface**: http://localhost:8080
- **3D Robot Visualizer**: http://localhost:8080/robot_visualizer.html

### **4. Test the System**

```bash
# Run comprehensive tests
./test_system.sh

# Run live demonstration
./demo_showcase.sh
```

### **5. Test Documentation**

For comprehensive testing and demonstration guides:
- **Test Demonstration Guide**: `TEST_DEMONSTRATION.md` - Complete testing checklist and procedures
- **Live Demo Script**: `demo_showcase.sh` - Interactive demonstration of all features
- **Automated Tests**: `test_system.sh` - Automated system validation

## 🧪 **Testing & Demonstration**

### **Quick Test Commands**
```bash
# Clear ports and start system
./kill_ports.sh
./start_system.sh

# Run automated tests
./test_system.sh

# Run live demonstration
./demo_showcase.sh
```

### **Manual Testing Checklist**
1. **Backend API**: `curl http://localhost:8000/health`
2. **Frontend Interface**: Open http://localhost:8080
3. **3D Visualizer**: Open http://localhost:8080/robot_visualizer.html
4. **API Documentation**: Open http://localhost:8000/docs

### **Demonstration for Presentations**
```bash
# Run the showcase demo
./demo_showcase.sh

# Then show in browser:
# 1. http://localhost:8080 - Visual programming interface
# 2. http://localhost:8080/robot_visualizer.html - 3D robot movement
```

For detailed testing procedures, see `TEST_DEMONSTRATION.md`.

## 🛠️ **Manual Setup (Alternative)**

If you prefer manual setup or encounter issues with automated scripts:

### **Step 1: Environment Setup**
```bash
# 1. Create virtual environment
python3 -m venv venv

# 2. Activate virtual environment
source venv/bin/activate

# 3. Install dependencies
pip install -r requirements_simple.txt
```

### **Step 2: Start Backend Server**
```bash
# Terminal 1 - Start backend (FastAPI)
cd backend
python -m uvicorn main_simple:app --host 0.0.0.0 --port 8000 --reload
```

You should see:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
INFO:     Application startup complete.
```

### **Step 3: Start Frontend Server**
```bash
# Terminal 2 - Start frontend (HTTP server)
cd drawflow_ui
python -m http.server 8080
```

You should see:
```
Serving HTTP on 0.0.0.0 port 8080 ...
```

### **Step 4: Access the System**
- Open http://localhost:8080 in your browser
- The 3D visualizer is available at http://localhost:8080/robot_visualizer.html

## 🎮 **How to Use**

### **Visual Programming Interface**
1. **Open the main interface**: http://localhost:8080
2. **Create nodes**: Drag robot nodes from the left panel to the canvas
3. **Connect nodes**: Click and drag from output points to input points to create connections
4. **Configure nodes**: Double-click nodes to open configuration modal
5. **Execute workflows**: Click "Execute Workflow" to run your robot program
6. **Watch the 3D robot**: Open the 3D visualizer to see real-time robot movements

### **3D Robot Visualizer**
1. **Access visualizer**: Click "3D Robot View" button or go to http://localhost:8080/robot_visualizer.html
2. **Manual control**: Use the control panel to manually operate the robot
3. **Real-time sync**: Watch the robot move in real-time as workflows execute
4. **Status monitoring**: Monitor connection status and robot state

### **Available Robot Commands**
- **MoveToPose** - Move robot to predefined positions (home, inspection, grasp)
- **SetGripper** - Control gripper opening/closing (0.0 = closed, 1.0 = open)
- **CaptureImage** - Simulate camera capture with optional saving
- **RunInspection** - Execute inspection routines

### **Debugging Tools**
The interface includes several debugging tools:
- **Test Connections** - Test node connection functionality
- **Inspect Nodes** - View detailed node information
- **Basic Test** - Create and connect simple test nodes
- **Test Workflow** - Create a sample workflow for testing

## 🏗️ **System Architecture**

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Drawflow UI   │    │  3D Robot Viz   │    │  FastAPI Backend│
│   (Frontend)    │◄──►│   (Frontend)    │◄──►│   (Backend)     │
│   Port: 8080    │    │   Port: 8080    │    │   Port: 8000    │
│                 │    │                 │    │                 │
│ • Visual Nodes  │    │ • 3D Robot Model│    │ • REST API      │
│ • Workflow Edit │    │ • Real-time Anim│    │ • WebSocket     │
│ • Node Config   │    │ • Manual Control│    │ • Simulation    │
│ • Save/Load     │    │ • Status Display│    │ • Command Exec  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## 📁 **Project Structure**

```
Ros2RoboticArm/
├── venv/                    # Virtual environment
├── backend/                 # FastAPI backend
│   └── main_simple.py      # Main backend application
├── drawflow_ui/            # Frontend interface
│   ├── index.html          # Main Drawflow interface
│   ├── robot_visualizer.html # 3D robot visualizer
│   ├── script.js           # Frontend JavaScript
│   └── styles.css          # Frontend styles
├── ros2_nodes/             # ROS 2 nodes (optional)
│   ├── mock_robot_node.py  # Mock robot node
│   └── inspection_node.py  # Inspection node
├── setup.sh                # Automated setup script
├── start_system.sh         # Automated start script
├── test_system.sh          # Automated test script
├── demo_simple.py          # Demo script
├── requirements_simple.txt # Python dependencies
├── QUICK_START.md          # Quick start guide
└── SYSTEM_SUMMARY.md       # Complete system overview
```

## 🔧 **Technical Stack**

### **Backend**
- **Python 3.8+** - Modern Python features
- **FastAPI** - High-performance web framework
- **Uvicorn** - ASGI server
- **WebSockets** - Real-time communication
- **Pydantic** - Data validation

### **Frontend**
- **HTML5/CSS3** - Modern responsive design
- **JavaScript (ES6+)** - Async/await, WebSocket
- **Three.js** - 3D graphics and animation
- **Drawflow** - Visual programming framework
- **Font Awesome** - Icon library

## 🎯 **Key Features**

### **✅ Visual Programming**
- Drag-and-drop interface with 4 specialized robot nodes
- Workflow save/load functionality
- Real-time execution with visual feedback
- Node configuration modals

### **✅ 3D Visualization**
- Realistic 6-DOF robot model
- Smooth animations and transitions
- Interactive manual controls
- Real-time status synchronization

### **✅ Backend Integration**
- REST API endpoints for all robot operations
- WebSocket real-time updates
- Simulated robot responses
- Comprehensive error handling

### **✅ User Experience**
- Professional UI design with modern styling
- Responsive layout for different screen sizes
- Real-time status monitoring
- Toast notifications for user feedback

## 🧪 **Testing**

### **Automated Testing**
```bash
./test_system.sh
```

### **Manual Testing**
```bash
# Activate virtual environment
source venv/bin/activate

# Run demo
python demo_simple.py
```

### **API Testing**
```bash
# Test health endpoint
curl http://localhost:8000/health

# Test status endpoint
curl http://localhost:8000/api/status

# Test command execution
curl -X POST http://localhost:8000/api/execute-command \
  -H "Content-Type: application/json" \
  -d '{"type": "MoveToPose", "data": {"pose": "home"}}'
```

## 🆘 **Troubleshooting**

### **Common Issues & Solutions**

#### **Port Already in Use Error**
```bash
# Option 1: Use the provided script (recommended)
./kill_ports.sh

# Option 2: Manual port clearing
# Check what's using the ports
lsof -i :8000  # Backend port
lsof -i :8080  # Frontend port

# Kill processes using these ports
sudo kill -9 $(lsof -t -i:8000)
sudo kill -9 $(lsof -t -i:8080)

# Then restart the system
./start_system.sh
```

#### **Virtual Environment Issues**
```bash
# Ensure virtual environment is activated
source venv/bin/activate

# Reinstall dependencies if needed
pip install -r requirements_simple.txt
```

#### **Permission Issues**
```bash
# Make scripts executable
chmod +x *.sh
```

#### **WebSocket Connection Issues**
- Ensure both backend (port 8000) and frontend (port 8080) are running
- Check browser console for WebSocket connection errors
- Verify firewall settings aren't blocking local connections

#### **Node Connection Issues**
- Use the "Test Connections" button to verify node functionality
- Check that nodes have proper input/output points visible
- Ensure you're clicking on the connection points (small circles) not the node body

### **Getting Help**
- Check terminal logs for detailed error messages
- Review `QUICK_START.md` for detailed instructions
- Check `SYSTEM_SUMMARY.md` for complete system overview
- API documentation available at http://localhost:8000/docs

## 🔮 **Future Enhancements**

- **Real ROS 2 Integration** - Connect to actual robot hardware
- **Camera Feed** - Real camera integration with OpenCV
- **Advanced Inspection** - Machine learning defect detection
- **Multi-robot Support** - Control multiple robots
- **Mobile Interface** - Responsive mobile design
- **Database Integration** - Store workflows and results

## 📄 **License**

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🤝 **Contributing**

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## 📞 **Support**

For support and questions:
- Check the documentation in `QUICK_START.md` and `SYSTEM_SUMMARY.md`
- Review the API documentation at http://localhost:8000/docs
- Check the terminal logs for detailed error messages

---

**🎉 Enjoy your ROS 2 Robotic Arm Inspection System with 3D Visualization!**