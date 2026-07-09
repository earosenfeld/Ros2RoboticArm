# 🧪 Test Demonstration Guide

This guide shows how to test and demonstrate that the ROS 2 Robotic Arm Inspection System is working correctly.

## 🚀 Quick Test Sequence

### **Step 1: Clear Ports and Start System**
```bash
# Clear any existing processes
./kill_ports.sh

# Start the system
./start_system.sh
```

### **Step 2: Run Automated Tests**
```bash
# Run comprehensive system tests
./test_system.sh
```

### **Step 3: Manual Testing**
Open your browser and test the following:

1. **Main Interface**: http://localhost:8080
2. **3D Robot Visualizer**: http://localhost:8080/robot_visualizer.html
3. **API Documentation**: http://localhost:8000/docs

## 📋 Test Checklist

### **✅ Backend API Tests**
- [ ] Health endpoint: `curl http://localhost:8000/health`
- [ ] Status endpoint: `curl http://localhost:8000/api/status`
- [ ] API documentation: http://localhost:8000/docs

### **✅ Frontend Interface Tests**
- [ ] Main interface loads: http://localhost:8080
- [ ] 3D visualizer loads: http://localhost:8080/robot_visualizer.html
- [ ] Node creation works (drag from left panel)
- [ ] Node connections work (click and drag between nodes)
- [ ] Node configuration works (double-click nodes)

### **✅ Robot Command Tests**
- [ ] MoveToPose command
- [ ] SetGripper command
- [ ] CaptureImage command
- [ ] RunInspection command

### **✅ Workflow Tests**
- [ ] Create simple workflow
- [ ] Execute workflow
- [ ] Watch 3D robot movement
- [ ] Real-time status updates

## 🎯 Demonstration Script

### **For Presentations/Demos**

1. **Start the System**
   ```bash
   ./kill_ports.sh
   ./start_system.sh
   ```

2. **Show Backend is Running**
   ```bash
   curl http://localhost:8000/health
   ```

3. **Run Demo Script**
   ```bash
   python demo_simple.py
   ```

4. **Show Web Interface**
   - Open http://localhost:8080
   - Demonstrate node creation
   - Show node connections
   - Execute a workflow

5. **Show 3D Visualization**
   - Open http://localhost:8080/robot_visualizer.html
   - Show real-time robot movement
   - Demonstrate manual controls

## 🔧 Individual Test Commands

### **Backend Health Check**
```bash
curl http://localhost:8000/health
```

### **System Status**
```bash
curl http://localhost:8000/api/status
```

### **Execute Single Command**
```bash
curl -X POST http://localhost:8000/api/execute-command \
  -H "Content-Type: application/json" \
  -d '{"type": "MoveToPose", "data": {"pose": "home"}}'
```

### **Execute Routine**
```bash
curl -X POST http://localhost:8000/api/execute-routine \
  -H "Content-Type: application/json" \
  -d '{
    "name": "Test Routine",
    "steps": [
      {"type": "MoveToPose", "pose": "home"},
      {"type": "SetGripper", "position": 1.0}
    ]
  }'
```

### **Get Example Workflow**
```bash
curl http://localhost:8000/api/example-workflow
```

## 🎮 Interactive Testing

### **Test Node Creation**
1. Open http://localhost:8080
2. Drag "MoveToPose" node from left panel
3. Drag "SetGripper" node from left panel
4. Verify nodes appear on canvas

### **Test Node Connections**
1. Click on output point (right side) of first node
2. Drag to input point (left side) of second node
3. Verify connection line appears
4. Use "Test Connections" button to verify

### **Test Node Configuration**
1. Double-click a node
2. Verify configuration modal opens
3. Change settings and click "Save"
4. Verify changes are applied

### **Test Workflow Execution**
1. Create a simple workflow with 2-3 nodes
2. Click "Execute Workflow"
3. Watch status updates
4. Open 3D visualizer to see robot movement

## 🐛 Troubleshooting Tests

### **If Ports Are Busy**
```bash
./kill_ports.sh
```

### **If Backend Won't Start**
```bash
# Check if virtual environment is activated
source venv/bin/activate

# Check if dependencies are installed
pip install -r requirements_simple.txt

# Try manual start
cd backend
python -m uvicorn main_simple:app --host 0.0.0.0 --port 8000 --reload
```

### **If Frontend Won't Start**
```bash
cd drawflow_ui
python -m http.server 8080
```

### **If Tests Fail**
```bash
# Check if both servers are running
curl http://localhost:8000/health
curl http://localhost:8080/

# Check logs for errors
# Look at terminal output for detailed error messages
```

## 📊 Expected Test Results

### **Backend Health Response**
```json
{
  "status": "healthy",
  "mode": "simulation",
  "robot_status": "idle",
  "timestamp": "2024-01-01T12:00:00Z"
}
```

### **System Status Response**
```json
{
  "status": "operational",
  "current_pose": "home",
  "gripper_position": 0.0,
  "is_moving": false,
  "last_command": null
}
```

### **Command Execution Response**
```json
{
  "status": "success",
  "message": "Command executed successfully",
  "command_type": "MoveToPose",
  "timestamp": "2024-01-01T12:00:00Z"
}
```

## 🎉 Success Indicators

### **✅ System is Working When:**
- Backend responds to health checks
- Frontend loads without errors
- Nodes can be created and connected
- Workflows execute successfully
- 3D robot moves in real-time
- WebSocket connections work
- No error messages in console

### **❌ System Needs Attention When:**
- Port conflicts occur
- Backend won't start
- Frontend won't load
- Nodes can't be connected
- Workflows don't execute
- 3D robot doesn't move
- WebSocket connection fails

## 📞 Getting Help

If tests fail:
1. Check the README.md troubleshooting section
2. Review terminal logs for error messages
3. Ensure both servers are running
4. Verify virtual environment is activated
5. Check that all dependencies are installed

---

**🎯 Use this guide to demonstrate that your ROS 2 Robotic Arm Inspection System is fully functional!** 