# Setup Guide

Install and run the Robotic Arm Inspection Workcell Simulation — the web demo
(Drawflow UI + 3D visualizer + FastAPI backend), the pure-Python kinematics
library and its tests, and (optionally) the ROS 2 nodes.

See [README.md](README.md) for what the project is and what's real vs simulated.

---

## Prerequisites

- **Python 3.10+** (3.10 / 3.11 / 3.12 are CI-tested)
- **Git**
- A modern web browser (Chrome, Firefox, Safari, Edge) for the demo
- **Optional, ROS 2 nodes only:** Ubuntu 22.04, ROS 2 Humble, and MoveIt 2.
  The simulation and kinematics library do **not** require ROS 2.

---

## 1. Clone and create a virtual environment

```bash
git clone <repository-url>
cd Ros2RoboticArm

python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
```

> An `activate_env.sh` helper is provided that activates `.venv` and prints the
> start commands.

---

## 2. Install dependencies

For the **web demo + backend**:

```bash
pip install -r requirements.txt
```

For the **kinematics library + tests** (lightweight; no FastAPI/OpenCV needed):

```bash
pip install numpy pytest matplotlib
```

---

## 3. Run the kinematics tests

The kinematics library (`robot_arm/`) is validated by a pytest suite that
parses `urdf/robot_arm.urdf` and checks forward kinematics, the Jacobian,
inverse kinematics convergence, and trajectory continuity:

```bash
python -m pytest tests/ -q
```

This is the same command run in CI across Python 3.10/3.11/3.12.

To regenerate the figures embedded in the README (workspace cloud, IK Cartesian
path, trajectory profile):

```bash
python scripts/make_figures.py
```

---

## 4. Run the web demo

The demo is two local servers: a FastAPI backend (port 8000) and a static file
server for the Drawflow UI (port 8080).

### Automated (recommended)

```bash
./start_system.sh
```

This activates the virtual environment, starts both servers, and prints the
access points. Press **Ctrl+C** to stop both.

If the ports are already in use:

```bash
./kill_ports.sh      # frees 8000 and 8080
./start_system.sh
```

### Manual

```bash
# Terminal 1 — backend (FastAPI)
source .venv/bin/activate
cd backend
python -m uvicorn main_simple:app --host 0.0.0.0 --port 8000 --reload

# Terminal 2 — frontend (static server)
cd drawflow_ui
python -m http.server 8080
```

### Access points

| What | URL |
|------|-----|
| Drawflow visual-programming UI | http://localhost:8080 |
| 3D robot visualizer | http://localhost:8080/robot_visualizer.html |
| Backend REST API | http://localhost:8000 |
| Interactive API docs (Swagger) | http://localhost:8000/docs |
| Health check | http://localhost:8000/health |

---

## 5. Using the demo

1. Open http://localhost:8080. Drag robot nodes from the left panel onto the
   canvas: **MoveToPose**, **SetGripper**, **CaptureImage**, **RunInspection**.
2. Connect node outputs to inputs to build an inspection workflow.
3. Double-click a node to configure it; click **Execute Workflow** to run it.
4. Open the 3D visualizer to watch the arm move in real time, or to drive it
   manually with the pose buttons and gripper slider.

### API examples

```bash
# Health
curl http://localhost:8000/health

# Single command
curl -X POST http://localhost:8000/api/execute-command \
  -H "Content-Type: application/json" \
  -d '{"type": "MoveToPose", "data": {"pose": "home"}}'

# Example workflow payload
curl http://localhost:8000/api/example-workflow
```

A standalone `python demo_simple.py` script exercises the API endpoints and
WebSocket stream end-to-end (start the backend first).

---

## 6. (Optional) ROS 2 nodes

The nodes in `ros2_nodes/` are scaffolding for a real ROS 2 + MoveIt stack and
are **not** required for the simulation above. They need a ROS 2 workspace:

```bash
# Source ROS 2 (Humble)
source /opt/ros/humble/setup.bash

# Build the package and source the workspace
colcon build
source install/setup.bash

# Make the node scripts executable
chmod +x ros2_nodes/*.py
```

The `package.xml` declares the ROS 2 dependencies (rclpy, MoveIt, RealSense,
Gazebo, tf2, etc.). Without a ROS 2 environment these nodes will not run — see
the "What's real vs simulated" note in the README.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| Port 8000/8080 already in use | `./kill_ports.sh`, then restart |
| `venv` activation fails | use `source .venv/bin/activate` (not bare path) |
| Backend won't start | confirm deps: `pip install -r requirements.txt` |
| Frontend blank / 404 | run the static server from inside `drawflow_ui/` |
| WebSocket disconnects | ensure both servers are running; the visualizer auto-reconnects |
| Tests can't import `robot_arm` | run pytest from the repo root |

For deeper API exploration, the live Swagger docs are at
http://localhost:8000/docs while the backend is running.
