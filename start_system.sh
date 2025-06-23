#!/bin/bash
# Start script for ROS 2 Robotic Arm Inspection System

# Activate virtual environment
source venv/bin/activate

echo "🤖 Starting ROS 2 Robotic Arm Inspection System"
echo "=============================================="

# Function to cleanup background processes
cleanup() {
    echo "🛑 Stopping servers..."
    kill $BACKEND_PID $FRONTEND_PID 2>/dev/null || true
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Start backend server
echo "🔧 Starting backend server..."
cd backend
python -m uvicorn main_simple:app --host 0.0.0.0 --port 8000 --reload &
BACKEND_PID=$!

# Wait a moment for backend to start
sleep 3

# Start frontend server
echo "🎨 Starting frontend server..."
cd ../drawflow_ui
python -m http.server 8080 &
FRONTEND_PID=$!

echo "✅ System started successfully!"
echo "🌐 Access points:"
echo "   • Main Interface: http://localhost:8080"
echo "   • 3D Robot Visualizer: http://localhost:8080/robot_visualizer.html"
echo "   • Backend API: http://localhost:8000"
echo "   • API Documentation: http://localhost:8000/docs"
echo ""
echo "🛑 Press Ctrl+C to stop the system"

# Wait for background processes
wait
