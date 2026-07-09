#!/bin/bash

echo "🔧 Killing processes using ports 8000 and 8080..."

# Check if ports are in use
echo "📋 Checking port usage..."

if lsof -i :8000 > /dev/null 2>&1; then
    echo "⚠️  Port 8000 (backend) is in use. Killing processes..."
    sudo kill -9 $(lsof -t -i:8000) 2>/dev/null
    echo "✅ Killed processes on port 8000"
else
    echo "✅ Port 8000 (backend) is free"
fi

if lsof -i :8080 > /dev/null 2>&1; then
    echo "⚠️  Port 8080 (frontend) is in use. Killing processes..."
    sudo kill -9 $(lsof -t -i:8080) 2>/dev/null
    echo "✅ Killed processes on port 8080"
else
    echo "✅ Port 8080 (frontend) is free"
fi

echo "🎉 Ports cleared! You can now run ./start_system.sh" 