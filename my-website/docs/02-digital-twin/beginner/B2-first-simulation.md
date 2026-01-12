# B2: Running Your First Simulation

> Launch Gazebo and interact with a pre-built humanoid world.

## Learning Objectives

By the end of this lesson, you will be able to:
- Install and configure Gazebo Classic with ROS 2
- Launch a simulation world from the command line
- Navigate the Gazebo interface
- Monitor simulation performance with RTF
- Interact with the simulated environment

---

## Prerequisites

Before starting, ensure you have:
- Ubuntu 22.04 (native or WSL2)
- ROS 2 Humble installed
- Chapter 1 completed (URDF fundamentals)

---

## Installing Gazebo Classic

Gazebo Classic (version 11) is the stable simulation platform for ROS 2 Humble.

### Step 1: Install Gazebo and ROS 2 Packages

```bash
# Update package lists
sudo apt update

# Install Gazebo Classic
sudo apt install gazebo

# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Verify installation
gazebo --version
```

Expected output:
```
Gazebo multi-robot simulator, version 11.x.x
```

### Step 2: Verify ROS 2 Integration

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Check Gazebo ROS packages
ros2 pkg list | grep gazebo
```

Expected output includes:
```
gazebo_msgs
gazebo_plugins
gazebo_ros
gazebo_ros2_control
```

---

## Your First Launch

Let's launch the pre-built humanoid lab world.

### Step 1: Set Up Your Workspace

```bash
# Create workspace if it doesn't exist
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy the humanoid_lab.world from chapter assets
# (Assuming you've cloned the book repository)
cp /path/to/robot-book/chapters/02-digital-twin/beginner/assets/humanoid_lab.world .
```

### Step 2: Launch Gazebo with the World

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Launch Gazebo with the humanoid lab world
gazebo --verbose humanoid_lab.world
```

You should see:
1. Gazebo window opens
2. A laboratory environment loads
3. A simple humanoid model appears
4. Physics simulation starts automatically

### Step 3: Explore the Interface

The Gazebo interface has several key areas:

```
┌─────────────────────────────────────────────────────────────┐
│  File  Edit  Camera  View  Window  Help                     │
├───────────────────────────────────────────────┬─────────────┤
│                                               │  World      │
│                                               │  ├─ Models  │
│           3D VIEWPORT                         │  │  └─ ...  │
│                                               │  ├─ Lights  │
│     [Your humanoid robot here]                │  └─ Physics │
│                                               │             │
│                                               │  Insert     │
│                                               │  [Models]   │
├───────────────────────────────────────────────┴─────────────┤
│  Time: 00:00:15.432  RTF: 0.95  Iterations: 15432           │
└─────────────────────────────────────────────────────────────┘
```

**Key Interface Elements**:
| Element | Location | Purpose |
|---------|----------|---------|
| 3D Viewport | Center | View and interact with simulation |
| World Panel | Right | Hierarchy of models and settings |
| Insert Panel | Right | Add new models from library |
| Time Display | Bottom | Current simulation time |
| RTF Display | Bottom | Real-Time Factor performance |

---

## Navigating the 3D Viewport

### Mouse Controls

| Action | Control |
|--------|---------|
| Rotate view | Left-click + drag |
| Pan view | Middle-click + drag |
| Zoom | Scroll wheel |
| Select model | Left-click on model |

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `R` | Reset view to default |
| `T` | Toggle translation mode |
| `E` | Toggle rotation mode |
| `Esc` | Deselect / Cancel |
| `Space` | Pause/Resume simulation |

### Practice Exercise

1. Zoom in on the humanoid model
2. Rotate the view to see it from different angles
3. Pan to explore the laboratory environment
4. Press `Space` to pause, then resume

---

## Monitoring Performance

### Real-Time Factor (RTF)

The RTF indicator at the bottom shows simulation health:

| RTF | Status | Action |
|-----|--------|--------|
| >= 0.95 | Excellent | No action needed |
| 0.8 - 0.95 | Good | Acceptable for digital twin |
| 0.5 - 0.8 | Warning | Consider simplifying world |
| &lt; 0.5 | Critical | Reduce complexity or use headless |

### Command-Line Monitoring

```bash
# In a new terminal, check Gazebo stats
gz stats
```

Output shows real-time performance:
```
Factor[1.00] SimTime[10.52] RealTime[10.52] Paused[F]
```

### Common Performance Issues

| Issue | Symptom | Solution |
|-------|---------|----------|
| Complex meshes | Low RTF, GPU usage high | Simplify collision geometries |
| Many models | Gradual slowdown | Reduce model count |
| Physics instability | RTF spikes, model jitter | Check URDF inertias |

---

## ROS 2 Integration

While Gazebo is running, ROS 2 topics are automatically available.

### List Available Topics

```bash
# In a new terminal
source /opt/ros/humble/setup.bash

# List all topics
ros2 topic list
```

Expected topics include:
```
/clock
/gazebo/link_states
/gazebo/model_states
/joint_states
/parameter_events
/rosout
```

### Echo a Topic

```bash
# Watch joint states
ros2 topic echo /joint_states
```

Output (simplified):
```yaml
header:
  stamp:
    sec: 10
    nanosec: 520000000
  frame_id: ''
name:
- left_hip_yaw
- left_hip_roll
- ...
position:
- 0.0
- 0.0
- ...
velocity:
- 0.0
- 0.0
- ...
```

### Publish a Command

```bash
# Send a simple velocity command (if robot supports it)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## Launching with ROS 2 Launch Files

For production use, launch Gazebo through ROS 2 launch files:

### Basic Launch Command

```bash
# Launch Gazebo as a ROS 2 node
ros2 launch gazebo_ros gazebo.launch.py world:=humanoid_lab.world
```

### Benefits of ROS 2 Launch

- Automatic parameter handling
- Respawn on crash
- Integrated with other ROS 2 nodes
- Unified logging

---

## Troubleshooting

### Gazebo Won't Start

```bash
# Check if another instance is running
killall gzserver gzclient

# Try with verbose logging
gazebo --verbose
```

### Black Screen / No Graphics

```bash
# For WSL2 or headless systems
export LIBGL_ALWAYS_SOFTWARE=1
gazebo --verbose
```

### World File Not Found

```bash
# Check Gazebo model path
echo $GAZEBO_MODEL_PATH

# Add custom path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src
```

### Low RTF Warning

```bash
# Try headless mode for better performance
gzserver humanoid_lab.world &

# Visualize with lighter client
gzclient
```

---

## What's Next?

You've successfully launched your first simulation! In the Intermediate tier, you'll learn to:
- Create custom world files from scratch
- Spawn your own URDF models
- Control joints programmatically

**Next**: [I1: Building Simulation Worlds](../intermediate/I1-building-worlds.md)

---

## AI Agent Assisted Prompts

Use these prompts with your AI coding assistant:

### Installation Help
```
I'm getting error "gazebo: command not found" on Ubuntu 22.04 with ROS 2
Humble. Walk me through the complete installation process and verify each step.
```

### Performance Optimization
```
My Gazebo simulation is running at RTF 0.4 with a 20-DOF humanoid model.
I have an Intel i7 with integrated graphics. Suggest optimizations to reach
RTF >= 0.8 without removing model complexity.
```

### ROS 2 Integration
```
I want to create a Python node that subscribes to /joint_states from Gazebo
and republishes processed data to /joint_states_filtered. Show me the
complete node implementation using rclpy.
```

---

## Summary

- Gazebo Classic (v11) is the primary simulation platform for ROS 2 Humble
- Launch worlds with `gazebo --verbose world_name.world`
- Navigate using mouse (rotate, pan, zoom) and keyboard shortcuts
- Monitor RTF to ensure performance >= 0.8
- ROS 2 topics automatically bridge simulation to your nodes

---

| Previous | Up | Next |
|----------|-----|------|
| [B1: Digital Twin Concepts](B1-digital-twin-concepts.md) | [Beginner Tier](../README.md#beginner-tier) | [I1: Building Worlds](../intermediate/I1-building-worlds.md) |
