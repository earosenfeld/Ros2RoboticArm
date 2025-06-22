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

```bash
# Automated start (starts both servers)
./start_system.sh
```

### **3. Test the System**

```bash
# Run comprehensive tests
./test_system.sh
```

### **4. Access the System**

- **Main Interface**: http://localhost:8080
- **3D Robot Visualizer**: http://localhost:8080/robot_visualizer.html
- **API Documentation**: http://localhost:8000/docs

## 🛠️ **Manual Setup**

If you prefer manual setup:

```bash
# 1. Create virtual environment
python3 -m venv venv

# 2. Activate virtual environment
source venv/bin/activate

# 3. Install dependencies
pip install -r requirements_simple.txt

# 4. Start backend (Terminal 1)
cd backend
python -m uvicorn main_simple:app --host 0.0.0.0 --port 8000 --reload

# 5. Start frontend (Terminal 2)
cd drawflow_ui
python -m http.server 8080
```

## 🎮 **How to Use**

### **Visual Programming**
1. Open http://localhost:8080
2. Drag robot nodes from the left panel
3. Connect nodes to create workflows
4. Configure nodes by double-clicking
5. Execute workflows and watch the 3D robot

### **3D Robot Visualizer**
1. Click "3D Robot View" button or go to http://localhost:8080/robot_visualizer.html
2. Use controls to manually operate the robot
3. Watch real-time synchronization with workflow execution
4. Monitor connection status and robot state

### **Available Robot Commands**
- **MoveToPose** - Move robot to predefined positions
- **SetGripper** - Control gripper opening/closing
- **CaptureImage** - Simulate camera capture
- **RunInspection** - Execute inspection routines

## 🏗️ **System Architecture**

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Drawflow UI   │    │  3D Robot Viz   │    │  FastAPI Backend│
│   (Frontend)    │◄──►│   (Frontend)    │◄──►│   (Backend)     │
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
- **Python 3.11+** - Modern Python features
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
- Drag-and-drop interface
- 4 specialized robot nodes
- Workflow save/load
- Real-time execution

### **✅ 3D Visualization**
- Realistic 6-DOF robot model
- Smooth animations
- Interactive controls
- Real-time status sync

### **✅ Backend Integration**
- REST API endpoints
- WebSocket real-time updates
- Simulated robot responses
- Comprehensive error handling

### **✅ User Experience**
- Professional UI design
- Responsive layout
- Status monitoring
- Toast notifications

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

### **Common Issues**
- **Virtual environment not activated**: Run `source venv/bin/activate`
- **Ports already in use**: Check if servers are already running
- **Dependencies missing**: Run `pip install -r requirements_simple.txt`
- **Scripts not executable**: Run `chmod +x *.sh`

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