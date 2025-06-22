#!/usr/bin/env python3

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
import json
import asyncio
import uvicorn
from datetime import datetime
import logging
import time
import random

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DrawflowCommand(BaseModel):
    """Model for Drawflow-generated commands."""
    type: str
    data: Dict[str, Any]
    routine_id: Optional[str] = None

class InspectionRoutine(BaseModel):
    """Model for complete inspection routines."""
    name: str
    steps: List[Dict[str, Any]]
    description: Optional[str] = None

class DrawflowWorkflow(BaseModel):
    """Model for Drawflow workflow data."""
    drawflow: Dict[str, Any]

class SimulatedRobot:
    """Simulated robot for demonstration purposes."""
    
    def __init__(self):
        self.current_pose = "home"
        self.gripper_position = 0.0
        self.is_moving = False
        self.status = "Ready"
        
    async def move_to_pose(self, pose: str) -> bool:
        """Simulate moving to a pose."""
        self.is_moving = True
        self.status = f"Moving to {pose}"
        logger.info(f"🤖 Robot moving to pose: {pose}")
        
        # Simulate movement time
        await asyncio.sleep(2)
        
        self.current_pose = pose
        self.is_moving = False
        self.status = "Ready"
        logger.info(f"✅ Robot reached pose: {pose}")
        return True
    
    async def set_gripper(self, position: float) -> bool:
        """Simulate setting gripper position."""
        self.status = f"Setting gripper to {position}"
        logger.info(f"🤖 Setting gripper to position: {position}")
        
        # Simulate gripper movement
        await asyncio.sleep(1)
        
        self.gripper_position = position
        self.status = "Ready"
        logger.info(f"✅ Gripper set to position: {position}")
        return True
    
    async def capture_image(self, save: bool = True) -> bool:
        """Simulate capturing an image."""
        self.status = "Capturing image"
        logger.info(f"📸 Capturing image (save={save})")
        
        # Simulate camera capture time
        await asyncio.sleep(1)
        
        self.status = "Ready"
        logger.info(f"✅ Image captured successfully")
        return True
    
    async def run_inspection(self, inspection_type: str = "standard") -> Dict[str, Any]:
        """Simulate running inspection."""
        self.status = f"Running {inspection_type} inspection"
        logger.info(f"🔍 Running {inspection_type} inspection")
        
        # Simulate inspection time
        await asyncio.sleep(3)
        
        # Simulate inspection results
        defects_found = random.randint(0, 3)
        result = {
            "timestamp": datetime.now().isoformat(),
            "defects_found": defects_found,
            "defects": [
                {
                    "area": f"area_{i+1}",
                    "type": "simulated_defect",
                    "severity": random.choice(["low", "medium", "high"]),
                    "confidence": round(random.uniform(0.7, 0.95), 2),
                    "location": {"x": random.randint(100, 500), "y": random.randint(100, 400)}
                }
                for i in range(defects_found)
            ],
            "overall_result": "PASS" if defects_found == 0 else "FAIL",
            "confidence": round(random.uniform(0.8, 0.98), 2)
        }
        
        self.status = "Ready"
        logger.info(f"✅ Inspection completed: {result['overall_result']}")
        return result

# Create simulated robot instance
simulated_robot = SimulatedRobot()

# Create FastAPI app
app = FastAPI(
    title="ROS 2 Robotic Arm Inspection API (Simulation Mode)",
    description="FastAPI backend for Drawflow-based robot inspection system - Running in simulation mode",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
    
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(f"WebSocket connected. Total connections: {len(self.active_connections)}")
    
    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        logger.info(f"WebSocket disconnected. Total connections: {len(self.active_connections)}")
    
    async def send_personal_message(self, message: str, websocket: WebSocket):
        try:
            await websocket.send_text(message)
        except Exception as e:
            logger.error(f"Error sending message: {e}")
            self.disconnect(websocket)
    
    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception as e:
                logger.error(f"Error broadcasting message: {e}")
                self.disconnect(connection)

manager = ConnectionManager()

@app.on_event("startup")
async def startup_event():
    """Initialize the application."""
    logger.info("🚀 Starting ROS 2 Robotic Arm Inspection API (Simulation Mode)")
    logger.info("📝 Note: Running in simulation mode - no real robot control")

@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "ROS 2 Robotic Arm Inspection API (Simulation Mode)",
        "version": "1.0.0",
        "status": "running",
        "mode": "simulation"
    }

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "mode": "simulation",
        "robot_status": simulated_robot.status
    }

@app.post("/api/execute-command")
async def execute_command(command: DrawflowCommand):
    """Execute a single Drawflow command."""
    try:
        logger.info(f"📥 Received command: {command.type}")
        
        if command.type == "MoveToPose":
            pose = command.data.get("pose", "home")
            success = await simulated_robot.move_to_pose(pose)
        elif command.type == "SetGripper":
            position = command.data.get("position", 0.0)
            success = await simulated_robot.set_gripper(position)
        elif command.type == "CaptureImage":
            save = command.data.get("save", True)
            success = await simulated_robot.capture_image(save)
        elif command.type == "RunInspection":
            inspection_type = command.data.get("type", "standard")
            result = await simulated_robot.run_inspection(inspection_type)
            # Broadcast result to WebSocket clients
            await manager.broadcast(json.dumps({
                "type": "inspection_result",
                "data": result
            }))
            success = True
        else:
            raise HTTPException(status_code=400, detail=f"Unknown command type: {command.type}")
        
        if success:
            return {
                "status": "success",
                "message": f"Command {command.type} executed successfully",
                "command_id": f"cmd_{datetime.now().timestamp()}"
            }
        else:
            raise HTTPException(status_code=500, detail="Failed to execute command")
            
    except Exception as e:
        logger.error(f"Error executing command: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/execute-routine")
async def execute_routine(routine: InspectionRoutine):
    """Execute a complete inspection routine."""
    try:
        logger.info(f"📋 Starting routine: {routine.name} with {len(routine.steps)} steps")
        
        # Execute each step
        for i, step in enumerate(routine.steps):
            logger.info(f"🔄 Executing step {i+1}/{len(routine.steps)}: {step.get('type', 'unknown')}")
            
            if step["type"] == "MoveToPose":
                await simulated_robot.move_to_pose(step.get("pose", "home"))
            elif step["type"] == "SetGripper":
                await simulated_robot.set_gripper(step.get("position", 0.0))
            elif step["type"] == "CaptureImage":
                await simulated_robot.capture_image(step.get("save", True))
            elif step["type"] == "RunInspection":
                result = await simulated_robot.run_inspection(step.get("type", "standard"))
                # Broadcast result to WebSocket clients
                await manager.broadcast(json.dumps({
                    "type": "inspection_result",
                    "data": result
                }))
            
            # Small delay between steps
            await asyncio.sleep(0.5)
        
        logger.info(f"✅ Routine '{routine.name}' completed successfully")
        return {
            "status": "success",
            "message": f"Routine '{routine.name}' completed successfully",
            "routine_id": f"routine_{datetime.now().timestamp()}",
            "total_steps": len(routine.steps)
        }
            
    except Exception as e:
        logger.error(f"Error executing routine: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/execute-workflow")
async def execute_workflow(workflow: DrawflowWorkflow):
    """Execute a Drawflow workflow."""
    try:
        # Parse Drawflow workflow and convert to commands
        commands = parse_drawflow_workflow(workflow.drawflow)
        
        if not commands:
            raise HTTPException(status_code=400, detail="No valid commands found in workflow")
        
        logger.info(f"🔄 Executing workflow with {len(commands)} commands")
        
        # Execute each command
        for i, command in enumerate(commands):
            logger.info(f"🔄 Executing command {i+1}/{len(commands)}: {command.get('type', 'unknown')}")
            
            if command["type"] == "MoveToPose":
                await simulated_robot.move_to_pose(command.get("pose", "home"))
            elif command["type"] == "SetGripper":
                await simulated_robot.set_gripper(command.get("position", 0.0))
            elif command["type"] == "CaptureImage":
                await simulated_robot.capture_image(command.get("save", True))
            elif command["type"] == "RunInspection":
                result = await simulated_robot.run_inspection(command.get("type", "standard"))
                # Broadcast result to WebSocket clients
                await manager.broadcast(json.dumps({
                    "type": "inspection_result",
                    "data": result
                }))
            
            # Small delay between commands
            await asyncio.sleep(0.5)
        
        logger.info(f"✅ Workflow executed successfully")
        return {
            "status": "success",
            "message": f"Workflow executed successfully with {len(commands)} commands",
            "routine_id": f"workflow_{datetime.now().timestamp()}",
            "total_steps": len(commands)
        }
            
    except Exception as e:
        logger.error(f"Error executing workflow: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/status")
async def get_status():
    """Get current system status."""
    return {
        "status": "active",
        "timestamp": datetime.now().isoformat(),
        "mode": "simulation",
        "robot_status": simulated_robot.status,
        "current_pose": simulated_robot.current_pose,
        "gripper_position": simulated_robot.gripper_position,
        "is_moving": simulated_robot.is_moving
    }

@app.get("/api/example-workflow")
async def get_example_workflow():
    """Get an example Drawflow workflow."""
    return {
        "drawflow": {
            "Home": {
                "data": {
                    "1": {
                        "id": 1,
                        "name": "MoveToPose",
                        "data": {"pose": "home", "description": "Move to home position"},
                        "class": "MoveToPose",
                        "html": "",
                        "typenode": False,
                        "inputs": {},
                        "outputs": {"output": {"connections": [{"node": "2", "input": "input"}]}},
                        "pos_x": 100,
                        "pos_y": 100
                    },
                    "2": {
                        "id": 2,
                        "name": "SetGripper",
                        "data": {"position": "1.0 (Open)", "description": "Open gripper"},
                        "class": "SetGripper",
                        "html": "",
                        "typenode": False,
                        "inputs": {"input": {"connections": [{"node": "1", "output": "output"}]}},
                        "outputs": {"output": {"connections": [{"node": "3", "input": "input"}]}},
                        "pos_x": 300,
                        "pos_y": 100
                    },
                    "3": {
                        "id": 3,
                        "name": "CaptureImage",
                        "data": {"save": True, "description": "Capture image"},
                        "class": "CaptureImage",
                        "html": "",
                        "typenode": False,
                        "inputs": {"input": {"connections": [{"node": "2", "output": "output"}]}},
                        "outputs": {"output": {"connections": [{"node": "4", "input": "input"}]}},
                        "pos_x": 500,
                        "pos_y": 100
                    },
                    "4": {
                        "id": 4,
                        "name": "RunInspection",
                        "data": {"type": "standard", "description": "Run inspection"},
                        "class": "RunInspection",
                        "html": "",
                        "typenode": False,
                        "inputs": {"input": {"connections": [{"node": "3", "output": "output"}]}},
                        "outputs": {},
                        "pos_x": 700,
                        "pos_y": 100
                    }
                }
            }
        }
    }

@app.websocket("/ws/status")
async def websocket_status(websocket: WebSocket):
    """WebSocket endpoint for real-time status updates."""
    await manager.connect(websocket)
    
    try:
        while True:
            # Send periodic status updates
            status_data = {
                "type": "robot_status",
                "status": simulated_robot.status,
                "current_pose": simulated_robot.current_pose,
                "gripper_position": simulated_robot.gripper_position,
                "is_moving": simulated_robot.is_moving
            }
            await manager.send_personal_message(json.dumps(status_data), websocket)
            
            # Wait for any message from client (keep connection alive)
            try:
                await asyncio.wait_for(websocket.receive_text(), timeout=5.0)
            except asyncio.TimeoutError:
                # Send status update every 5 seconds
                continue
                
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        manager.disconnect(websocket)

def parse_drawflow_workflow(workflow_data: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Parse Drawflow workflow and convert to commands."""
    commands = []
    
    try:
        # Extract nodes from workflow
        if 'Home' in workflow_data and 'data' in workflow_data['Home']:
            nodes = workflow_data['Home']['data']
            
            # Convert nodes to commands
            for node_id, node in nodes.items():
                command = convert_drawflow_node_to_command(node)
                if command:
                    commands.append(command)
        
        # Sort commands by execution order (simplified)
        commands.sort(key=lambda x: int(x.get('node_id', 0)))
        
        return commands
        
    except Exception as e:
        logger.error(f"Error parsing Drawflow workflow: {e}")
        return []

def convert_drawflow_node_to_command(node: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Convert a single Drawflow node to a command."""
    try:
        node_name = node.get('name')
        node_data = node.get('data', {})
        
        base_command = {
            "type": node_name,
            "description": node_data.get('description', ''),
            "node_id": node.get('id')
        }
        
        if node_name == 'MoveToPose':
            return {
                **base_command,
                "pose": node_data.get('pose', 'home')
            }
        elif node_name == 'SetGripper':
            position_str = node_data.get('position', '0.0 (Closed)')
            position = 1.0 if 'Open' in position_str else 0.0
            return {
                **base_command,
                "position": position
            }
        elif node_name == 'CaptureImage':
            return {
                **base_command,
                "save": node_data.get('save', True)
            }
        elif node_name == 'RunInspection':
            return {
                **base_command,
                "type": node_data.get('type', 'standard')
            }
        else:
            logger.warn(f"Unknown node type: {node_name}")
            return None
            
    except Exception as e:
        logger.error(f"Error converting node to command: {e}")
        return None

if __name__ == "__main__":
    uvicorn.run(
        "main_simple:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    ) 