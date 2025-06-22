#!/usr/bin/env python3
"""
Demo script for ROS 2 Robotic Arm Inspection System with Drawflow integration.

This script demonstrates how to:
1. Start the FastAPI backend
2. Create and execute a simple inspection routine
3. Monitor the execution status
"""

import asyncio
import aiohttp
import json
import time
from datetime import datetime


class InspectionSystemDemo:
    """Demo class for the inspection system."""
    
    def __init__(self, api_url="http://localhost:8000"):
        self.api_url = api_url
        self.session = None
    
    async def start_session(self):
        """Start HTTP session."""
        self.session = aiohttp.ClientSession()
    
    async def close_session(self):
        """Close HTTP session."""
        if self.session:
            await self.session.close()
    
    async def check_health(self):
        """Check if the API is running."""
        try:
            async with self.session.get(f"{self.api_url}/health") as response:
                if response.status == 200:
                    data = await response.json()
                    print(f"✅ API is healthy: {data}")
                    return True
                else:
                    print(f"❌ API health check failed: {response.status}")
                    return False
        except Exception as e:
            print(f"❌ Failed to connect to API: {e}")
            return False
    
    async def get_example_workflow(self):
        """Get an example workflow from the API."""
        try:
            async with self.session.get(f"{self.api_url}/api/example-workflow") as response:
                if response.status == 200:
                    workflow = await response.json()
                    print("📋 Retrieved example workflow")
                    return workflow
                else:
                    print(f"❌ Failed to get example workflow: {response.status}")
                    return None
        except Exception as e:
            print(f"❌ Error getting example workflow: {e}")
            return None
    
    async def execute_workflow(self, workflow):
        """Execute a workflow."""
        try:
            async with self.session.post(
                f"{self.api_url}/api/execute-workflow",
                json=workflow
            ) as response:
                if response.status == 200:
                    result = await response.json()
                    print(f"🚀 Workflow execution started: {result}")
                    return result
                else:
                    error_text = await response.text()
                    print(f"❌ Failed to execute workflow: {response.status} - {error_text}")
                    return None
        except Exception as e:
            print(f"❌ Error executing workflow: {e}")
            return None
    
    async def get_status(self):
        """Get current system status."""
        try:
            async with self.session.get(f"{self.api_url}/api/status") as response:
                if response.status == 200:
                    status = await response.json()
                    return status
                else:
                    print(f"❌ Failed to get status: {response.status}")
                    return None
        except Exception as e:
            print(f"❌ Error getting status: {e}")
            return None
    
    async def create_simple_routine(self):
        """Create a simple inspection routine."""
        routine = {
            "name": "Simple Inspection Demo",
            "steps": [
                {
                    "type": "MoveToPose",
                    "pose": "home",
                    "description": "Move to home position"
                },
                {
                    "type": "SetGripper",
                    "position": 1.0,
                    "description": "Open gripper"
                },
                {
                    "type": "MoveToPose",
                    "pose": "inspection_1",
                    "description": "Move to inspection position 1"
                },
                {
                    "type": "CaptureImage",
                    "save": True,
                    "description": "Capture image from position 1"
                },
                {
                    "type": "RunInspection",
                    "type": "standard",
                    "description": "Run standard inspection"
                },
                {
                    "type": "MoveToPose",
                    "pose": "home",
                    "description": "Return to home position"
                }
            ]
        }
        return routine
    
    async def monitor_execution(self, duration=30):
        """Monitor execution for a specified duration."""
        print(f"📊 Monitoring execution for {duration} seconds...")
        start_time = time.time()
        
        while time.time() - start_time < duration:
            status = await self.get_status()
            if status:
                print(f"⏱️  Status at {datetime.now().strftime('%H:%M:%S')}: {status.get('status', 'unknown')}")
                
                # Check for inspection results
                latest_status = status.get('latest_status', {})
                if 'inspection' in latest_status:
                    inspection_status = latest_status['inspection']
                    if inspection_status.get('status') == 'completed':
                        print("✅ Inspection routine completed!")
                        return True
            
            await asyncio.sleep(2)
        
        print("⏰ Monitoring timeout reached")
        return False


async def main():
    """Main demo function."""
    print("🤖 ROS 2 Robotic Arm Inspection System Demo")
    print("=" * 50)
    
    demo = InspectionSystemDemo()
    
    try:
        await demo.start_session()
        
        # Check if API is running
        print("\n1. Checking API health...")
        if not await demo.check_health():
            print("❌ API is not running. Please start the backend first:")
            print("   python backend/main.py")
            return
        
        # Get example workflow
        print("\n2. Getting example workflow...")
        workflow = await demo.get_example_workflow()
        if not workflow:
            print("❌ Failed to get example workflow")
            return
        
        # Execute workflow
        print("\n3. Executing workflow...")
        result = await demo.execute_workflow(workflow)
        if not result:
            print("❌ Failed to execute workflow")
            return
        
        # Monitor execution
        print("\n4. Monitoring execution...")
        await demo.monitor_execution(duration=30)
        
        print("\n🎉 Demo completed!")
        
    except Exception as e:
        print(f"❌ Demo failed: {e}")
    
    finally:
        await demo.close_session()


if __name__ == "__main__":
    print("Starting demo...")
    print("Make sure you have:")
    print("1. ROS 2 environment sourced")
    print("2. FastAPI backend running: python backend/main.py")
    print("3. ROS 2 nodes running: ros2 launch ros2_robotic_arm inspection_system.launch.py")
    print()
    
    asyncio.run(main()) 