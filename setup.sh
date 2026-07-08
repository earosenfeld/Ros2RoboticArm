#!/bin/bash

# ROS 2 Robotic Arm Inspection System Setup Script
# This script sets up a virtual environment and installs all required dependencies

set -e  # Exit on any error

echo "🤖 ROS 2 Robotic Arm Inspection System Setup"
echo "=============================================="

# Check if Python 3 is available
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed. Please install Python 3.8+ first."
    exit 1
fi

# Check Python version
PYTHON_VERSION=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
echo "✅ Python version: $PYTHON_VERSION"

# Create virtual environment
echo "📦 Creating virtual environment..."
if [ -d "venv" ]; then
    echo "⚠️  Virtual environment already exists. Removing old one..."
    rm -rf venv
fi

python3 -m venv venv
echo "✅ Virtual environment created successfully!"

# Activate virtual environment
echo "🔧 Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo "⬆️  Upgrading pip..."
pip install --upgrade pip

# Install dependencies
echo "📚 Installing Python dependencies..."
if [ -f "requirements.txt" ]; then
    pip install -r requirements.txt
    echo "✅ Dependencies installed successfully!"
else
    echo "❌ requirements.txt not found!"
    exit 1
fi

# Create activation script
echo "📝 Creating activation script..."
cat > activate_env.sh << 'EOF'
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
EOF

chmod +x activate_env.sh

# Create start script
echo "🚀 Creating start script..."
cat > start_system.sh << 'EOF'
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
EOF

chmod +x start_system.sh

# Create test script
echo "🧪 Creating test script..."
cat > test_system.sh << 'EOF'
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
EOF

chmod +x test_system.sh

echo ""
echo "🎉 Setup completed successfully!"
echo "=============================================="
echo ""
echo "📋 Next steps:"
echo "   1. Activate the environment:"
echo "      source venv/bin/activate"
echo "      # or use: ./activate_env.sh"
echo ""
echo "   2. Start the system:"
echo "      ./start_system.sh"
echo ""
echo "   3. Test the system:"
echo "      ./test_system.sh"
echo ""
echo "   4. Access the system:"
echo "      • Main Interface: http://localhost:8080"
echo "      • 3D Robot Visualizer: http://localhost:8080/robot_visualizer.html"
echo "      • API Documentation: http://localhost:8000/docs"
echo ""
echo "📚 For more information, see:"
echo "   • QUICK_START.md - Quick start guide"
echo "   • SETUP_GUIDE.md - Detailed setup instructions"
echo "   • SYSTEM_SUMMARY.md - Complete system overview"
echo ""
echo "🤖 Enjoy your ROS 2 Robotic Arm Inspection System!"
EOF 