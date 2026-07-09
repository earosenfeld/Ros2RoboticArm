#!/bin/bash

echo "🤖 ROS 2 Robotic Arm Inspection System - Live Demonstration"
echo "=========================================================="
echo ""

# Check if system is running
echo "🔍 Checking system status..."
if curl -s http://localhost:8000/health > /dev/null; then
    echo "✅ Backend server is running"
else
    echo "❌ Backend server is not running. Please start it first:"
    echo "   ./kill_ports.sh && ./start_system.sh"
    exit 1
fi

if curl -s http://localhost:8080/ > /dev/null; then
    echo "✅ Frontend server is running"
else
    echo "❌ Frontend server is not running. Please start it first:"
    echo "   ./kill_ports.sh && ./start_system.sh"
    exit 1
fi

echo ""
echo "🌐 System Access Points:"
echo "   • Main Interface: http://localhost:8080"
echo "   • 3D Robot Visualizer: http://localhost:8080/robot_visualizer.html"
echo "   • API Documentation: http://localhost:8000/docs"
echo ""

# Show current system status
echo "📊 Current System Status:"
curl -s http://localhost:8000/api/status | python3 -c "
import json, sys
data = json.load(sys.stdin)
print(f'   • Robot Status: {data[\"robot_status\"]}')
print(f'   • Current Pose: {data[\"current_pose\"]}')
print(f'   • Gripper Position: {data[\"gripper_position\"]}')
print(f'   • Is Moving: {data[\"is_moving\"]}')
"

echo ""
echo "🎮 Available Demo Commands:"
echo ""

# Demo 1: Move to home
echo "1️⃣ Moving robot to home position..."
curl -s -X POST http://localhost:8000/api/execute-command \
  -H "Content-Type: application/json" \
  -d '{"type": "MoveToPose", "data": {"pose": "home"}}' | python3 -c "
import json, sys
data = json.load(sys.stdin)
print(f'   ✅ {data[\"message\"]}')
"

sleep 2

# Demo 2: Open gripper
echo ""
echo "2️⃣ Opening gripper..."
curl -s -X POST http://localhost:8000/api/execute-command \
  -H "Content-Type: application/json" \
  -d '{"type": "SetGripper", "data": {"position": 1.0}}' | python3 -c "
import json, sys
data = json.load(sys.stdin)
print(f'   ✅ {data[\"message\"]}')
"

sleep 2

# Demo 3: Move to inspection
echo ""
echo "3️⃣ Moving to inspection position..."
curl -s -X POST http://localhost:8000/api/execute-command \
  -H "Content-Type: application/json" \
  -d '{"type": "MoveToPose", "data": {"pose": "inspection_position"}}' | python3 -c "
import json, sys
data = json.load(sys.stdin)
print(f'   ✅ {data[\"message\"]}')
"

sleep 2

# Demo 4: Capture image
echo ""
echo "4️⃣ Capturing inspection image..."
curl -s -X POST http://localhost:8000/api/execute-command \
  -H "Content-Type: application/json" \
  -d '{"type": "CaptureImage", "data": {"save": true}}' | python3 -c "
import json, sys
data = json.load(sys.stdin)
print(f'   ✅ {data[\"message\"]}')
"

sleep 2

# Demo 5: Run inspection
echo ""
echo "5️⃣ Running inspection routine..."
curl -s -X POST http://localhost:8000/api/execute-command \
  -H "Content-Type: application/json" \
  -d '{"type": "RunInspection", "data": {"type": "standard"}}' | python3 -c "
import json, sys
data = json.load(sys.stdin)
print(f'   ✅ {data[\"message\"]}')
"

echo ""
echo "🎯 Demo Complete!"
echo ""
echo "📋 What to Show Next:"
echo "   1. Open http://localhost:8080 to show the visual programming interface"
echo "   2. Drag nodes from the left panel to create a workflow"
echo "   3. Connect nodes by clicking and dragging between connection points"
echo "   4. Double-click nodes to configure them"
echo "   5. Click 'Execute Workflow' to run the robot program"
echo "   6. Open http://localhost:8080/robot_visualizer.html to show 3D robot movement"
echo ""
echo "🔧 System is ready for demonstration!" 