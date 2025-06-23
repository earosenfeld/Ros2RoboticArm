#!/bin/bash
# Activation script for ROS 2 Robotic Arm Inspection System
echo "🤖 Activating ROS 2 Robotic Arm Inspection System Environment"
echo "=============================================================="
source venv/bin/activate
echo "✅ Virtual environment activated!"
echo "🌐 To start the system:"
echo "   1. Terminal 1: cd backend && python -m uvicorn main_simple:app --host 0.0.0.0 --port 8000 --reload"
echo "   2. Terminal 2: cd drawflow_ui && python -m http.server 8080"
echo "   3. Open browser: http://localhost:8080"
echo ""
