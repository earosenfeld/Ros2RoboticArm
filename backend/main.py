#!/usr/bin/env python3

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
import json
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import String
import uvicorn
from datetime import datetime
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize ROS 2
rclpy.init()

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

class ROS2Bridge(Node):
    """ROS 2 bridge for communicating with robot nodes."""
    
    def __init__(self):
        super().__init__('fastapi_bridge')
        
        # Publishers
        self.drawflow_command_pub = self.create_publisher(
            String,
            '/drawflow_command',
            10
        )
        
        # Subscribers
        self.inspection_status_sub = self.create_subscription(
            String,
            '/inspection_status',
            self.inspection_status_callback,
            10
        )
        
        self.robot_status_sub = self.create_subscription(
            String,
            '/robot_status',
            self.robot_status_callback,
            10
        )
        
        # Status storage
        self.latest_status = {}
        self.status_callbacks = []
        
        self.get_logger().info('ROS 2 Bridge initialized')
    
    def inspection_status_callback(self, msg):
        """Callback for inspection status updates."""
        try:
            status_data = json.loads(msg.data)
            self.latest_status['inspection'] = status_data
            
            # Notify WebSocket clients
            for callback in self.status_callbacks:
                try:
                    callback(status_data)
                except Exception as e:
                    logger.error(f"Error in status callback: {e}")
                    
        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON in inspection status: {e}")
    
    def robot_status_callback(self, msg):
        """Callback for robot status updates."""
        try:
            self.latest_status['robot'] = msg.data
            
            # Notify WebSocket clients
            for callback in self.status_callbacks:
                try:
                    callback({'type': 'robot_status', 'status': msg.data})
                except Exception as e:
                    logger.error(f"Error in status callback: {e}")
                    
        except Exception as e:
            logger.error(f"Error in robot status callback: {e}")
    
    def publish_drawflow_command(self, command: Dict[str, Any]):
        """Publish a Drawflow command to ROS 2."""
        try:
            command_msg = String()
            command_msg.data = json.dumps(command)
            self.drawflow_command_pub.publish(command_msg)
            logger.info(f"Published Drawflow command: {command}")
            return True
        except Exception as e:
            logger.error(f"Error publishing Drawflow command: {e}")
            return False
    
    def add_status_callback(self, callback):
        """Add a callback for status updates."""
        self.status_callbacks.append(callback)
    
    def remove_status_callback(self, callback):
        """Remove a status callback."""
        if callback in self.status_callbacks:
            self.status_callbacks.remove(callback)

# Create ROS 2 bridge instance
ros2_bridge = ROS2Bridge()

# Create FastAPI app
app = FastAPI(
    title="ROS 2 Robotic Arm Inspection API",
    description="FastAPI backend for Drawflow-based robot inspection system",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify actual origins
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
    logger.info("Starting ROS 2 Robotic Arm Inspection API")

@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown."""
    logger.info("Shutting down API")
    ros2_bridge.destroy_node()
    rclpy.shutdown()

@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "ROS 2 Robotic Arm Inspection API",
        "version": "1.0.0",
        "status": "running"
    }

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "ros2_bridge": "active"
    }

@app.post("/api/execute-command")
async def execute_command(command: DrawflowCommand):
    """Execute a single Drawflow command."""
    try:
        # Convert Drawflow command to ROS 2 format
        ros2_command = {
            "type": command.type,
            **command.data
        }
        
        # Publish to ROS 2
        success = ros2_bridge.publish_drawflow_command(ros2_command)
        
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
        # Convert routine to ROS 2 format
        ros2_routine = {
            "type": "routine",
            "name": routine.name,
            "steps": routine.steps,
            "description": routine.description,
            "routine_id": f"routine_{datetime.now().timestamp()}"
        }
        
        # Publish to ROS 2
        success = ros2_bridge.publish_drawflow_command(ros2_routine)
        
        if success:
            return {
                "status": "success",
                "message": f"Routine '{routine.name}' started successfully",
                "routine_id": ros2_routine["routine_id"],
                "total_steps": len(routine.steps)
            }
        else:
            raise HTTPException(status_code=500, detail="Failed to start routine")
            
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
        
        # Create routine from commands
        routine = {
            "type": "routine",
            "name": "Drawflow Workflow",
            "steps": commands,
            "routine_id": f"workflow_{datetime.now().timestamp()}"
        }
        
        # Publish to ROS 2
        success = ros2_bridge.publish_drawflow_command(routine)
        
        if success:
            return {
                "status": "success",
                "message": f"Workflow executed successfully with {len(commands)} commands",
                "routine_id": routine["routine_id"],
                "total_steps": len(commands)
            }
        else:
            raise HTTPException(status_code=500, detail="Failed to execute workflow")
            
    except Exception as e:
        logger.error(f"Error executing workflow: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/status")
async def get_status():
    """Get current system status."""
    return {
        "status": "active",
        "timestamp": datetime.now().isoformat(),
        "latest_status": ros2_bridge.latest_status
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
    
    # Add status callback
    async def status_callback(status_data):
        await manager.send_personal_message(
            json.dumps(status_data), 
            websocket
        )
    
    ros2_bridge.add_status_callback(status_callback)
    
    try:
        while True:
            # Keep connection alive
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)
        ros2_bridge.remove_status_callback(status_callback)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        manager.disconnect(websocket)
        ros2_bridge.remove_status_callback(status_callback)

def parse_drawflow_workflow(workflow_data: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Parse Drawflow workflow and convert to ROS 2 commands."""
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
        # In a real implementation, you'd need proper topological sorting
        commands.sort(key=lambda x: int(x.get('node_id', 0)))
        
        return commands
        
    except Exception as e:
        logger.error(f"Error parsing Drawflow workflow: {e}")
        return []

def convert_drawflow_node_to_command(node: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Convert a single Drawflow node to a ROS 2 command."""
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
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    ) 