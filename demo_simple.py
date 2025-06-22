#!/usr/bin/env python3
"""
Simple demo script for the ROS 2 Robotic Arm Inspection System (Simulation Mode)
"""

import asyncio
import aiohttp
import json
from datetime import datetime

# API base URL
BASE_URL = "http://localhost:8000"

async def test_api_endpoints():
    """Test various API endpoints."""
    print("🚀 Testing ROS 2 Robotic Arm Inspection API (Simulation Mode)")
    print("=" * 60)
    
    async with aiohttp.ClientSession() as session:
        
        # Test health endpoint
        print("\n1. Testing health endpoint...")
        async with session.get(f"{BASE_URL}/health") as response:
            health_data = await response.json()
            print(f"✅ Health: {health_data['status']}")
            print(f"   Mode: {health_data['mode']}")
            print(f"   Robot Status: {health_data['robot_status']}")
        
        # Test status endpoint
        print("\n2. Testing status endpoint...")
        async with session.get(f"{BASE_URL}/api/status") as response:
            status_data = await response.json()
            print(f"✅ System Status: {status_data['status']}")
            print(f"   Current Pose: {status_data['current_pose']}")
            print(f"   Gripper Position: {status_data['gripper_position']}")
            print(f"   Is Moving: {status_data['is_moving']}")
        
        # Test single command execution
        print("\n3. Testing single command execution...")
        move_command = {
            "type": "MoveToPose",
            "data": {"pose": "inspection_position"},
            "routine_id": "demo_001"
        }
        
        async with session.post(f"{BASE_URL}/api/execute-command", 
                               json=move_command) as response:
            result = await response.json()
            print(f"✅ Move Command: {result['status']}")
            print(f"   Message: {result['message']}")
        
        # Test gripper command
        print("\n4. Testing gripper command...")
        gripper_command = {
            "type": "SetGripper",
            "data": {"position": 1.0},
            "routine_id": "demo_001"
        }
        
        async with session.post(f"{BASE_URL}/api/execute-command", 
                               json=gripper_command) as response:
            result = await response.json()
            print(f"✅ Gripper Command: {result['status']}")
            print(f"   Message: {result['message']}")
        
        # Test capture image command
        print("\n5. Testing capture image command...")
        capture_command = {
            "type": "CaptureImage",
            "data": {"save": True},
            "routine_id": "demo_001"
        }
        
        async with session.post(f"{BASE_URL}/api/execute-command", 
                               json=capture_command) as response:
            result = await response.json()
            print(f"✅ Capture Command: {result['status']}")
            print(f"   Message: {result['message']}")
        
        # Test inspection command
        print("\n6. Testing inspection command...")
        inspection_command = {
            "type": "RunInspection",
            "data": {"type": "standard"},
            "routine_id": "demo_001"
        }
        
        async with session.post(f"{BASE_URL}/api/execute-command", 
                               json=inspection_command) as response:
            result = await response.json()
            print(f"✅ Inspection Command: {result['status']}")
            print(f"   Message: {result['message']}")
        
        # Test complete routine execution
        print("\n7. Testing complete routine execution...")
        routine = {
            "name": "Demo Inspection Routine",
            "description": "A simple demonstration routine",
            "steps": [
                {
                    "type": "MoveToPose",
                    "pose": "home",
                    "description": "Move to home position"
                },
                {
                    "type": "SetGripper",
                    "position": 0.0,
                    "description": "Close gripper"
                },
                {
                    "type": "MoveToPose",
                    "pose": "inspection_position",
                    "description": "Move to inspection position"
                },
                {
                    "type": "CaptureImage",
                    "save": True,
                    "description": "Capture inspection image"
                },
                {
                    "type": "RunInspection",
                    "type": "standard",
                    "description": "Run standard inspection"
                }
            ]
        }
        
        async with session.post(f"{BASE_URL}/api/execute-routine", 
                               json=routine) as response:
            result = await response.json()
            print(f"✅ Routine Execution: {result['status']}")
            print(f"   Message: {result['message']}")
            print(f"   Total Steps: {result['total_steps']}")
        
        # Test example workflow
        print("\n8. Testing example workflow...")
        async with session.get(f"{BASE_URL}/api/example-workflow") as response:
            workflow_data = await response.json()
            print(f"✅ Example Workflow Retrieved")
            print(f"   Nodes: {len(workflow_data['drawflow']['Home']['data'])}")
        
        # Test workflow execution
        print("\n9. Testing workflow execution...")
        async with session.post(f"{BASE_URL}/api/execute-workflow", 
                               json=workflow_data) as response:
            result = await response.json()
            print(f"✅ Workflow Execution: {result['status']}")
            print(f"   Message: {result['message']}")
            print(f"   Total Steps: {result['total_steps']}")

async def test_websocket():
    """Test WebSocket connection for real-time updates."""
    print("\n10. Testing WebSocket connection...")
    
    try:
        async with aiohttp.ClientSession() as session:
            async with session.ws_connect(f"{BASE_URL.replace('http', 'ws')}/ws/status") as ws:
                print("✅ WebSocket connected")
                
                # Wait for a few status updates
                for i in range(3):
                    try:
                        msg = await asyncio.wait_for(ws.receive_json(), timeout=6.0)
                        print(f"   Status Update {i+1}: {msg['type']} - {msg['status']}")
                    except asyncio.TimeoutError:
                        print(f"   Status Update {i+1}: Timeout")
                        break
                
                print("✅ WebSocket test completed")
                
    except Exception as e:
        print(f"❌ WebSocket test failed: {e}")

def main():
    """Main function to run the demo."""
    print("🎯 ROS 2 Robotic Arm Inspection System Demo")
    print("📝 Running in Simulation Mode")
    print("🌐 Backend: http://localhost:8000")
    print("🎨 Frontend: http://localhost:8080")
    print("📚 API Docs: http://localhost:8000/docs")
    print("=" * 60)
    
    # Run the async demo
    asyncio.run(test_api_endpoints())
    asyncio.run(test_websocket())
    
    print("\n" + "=" * 60)
    print("✅ Demo completed successfully!")
    print("\n🎮 Next Steps:")
    print("   1. Open http://localhost:8080 in your browser")
    print("   2. Create a visual workflow using the Drawflow interface")
    print("   3. Execute the workflow and watch the robot simulation")
    print("   4. Check http://localhost:8000/docs for API documentation")
    print("\n🔧 To stop the servers:")
    print("   - Press Ctrl+C in the terminal windows running the servers")

if __name__ == "__main__":
    main() 