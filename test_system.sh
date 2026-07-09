#!/bin/bash
# Test script for ROS 2 Robotic Arm Inspection System

# Activate virtual environment
source venv/bin/activate

echo "🧪 Testing ROS 2 Robotic Arm Inspection System"
echo "=============================================="

# Check if backend is running
if curl -s http://localhost:8000/health > /dev/null; then
    echo "✅ Backend server is running"
else
    echo "❌ Backend server is not running. Please start it first."
    echo "   Run: ./start_system.sh"
    exit 1
fi

# Check if frontend is running
if curl -s http://localhost:8080/ > /dev/null; then
    echo "✅ Frontend server is running"
else
    echo "❌ Frontend server is not running. Please start it first."
    echo "   Run: ./start_system.sh"
    exit 1
fi

# Run the demo
echo "🚀 Running system demo..."
python demo_simple.py

echo "✅ System test completed successfully!"
