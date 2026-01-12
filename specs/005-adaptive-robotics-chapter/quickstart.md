# Quickstart: Chapter 5 - Adaptive Robotics

**Feature**: 005-adaptive-robotics-chapter
**Date**: 2025-12-30

---

## Prerequisites

### Required Software

| Software | Version | Purpose |
|----------|---------|---------|
| Ubuntu | 22.04 LTS | Operating system |
| ROS 2 | Humble or Iron | Robot middleware |
| Python | 3.10+ | Programming language |
| Gazebo | Fortress (Humble) or Harmonic (Iron) | Simulation |
| TurtleBot3 | Latest | Robot platform |

### Installation Check

```bash
# Verify ROS 2 installation
ros2 --version

# Verify Python version
python3 --version

# Verify TurtleBot3 packages
ros2 pkg list | grep turtlebot3
```

### Required Knowledge

- Completed Chapters 1-4 (or equivalent)
- Basic Python programming (variables, functions, conditionals)
- ROS 2 fundamentals (nodes, topics, messages)

---

## Quick Setup

### 1. Set Environment Variables

```bash
# Add to ~/.bashrc
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
```

### 2. Create Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone chapter code (when available)
# git clone https://github.com/your-org/adaptive_robotics.git

# Build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch Simulation

```bash
# Terminal 1: Start Gazebo with TurtleBot3
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Start behavior switcher (after building)
ros2 run adaptive_robotics behavior_switcher
```

---

## Learning Path

### Beginner Tier (No ROS 2 Required)

| Lesson | Duration | Prerequisites |
|--------|----------|---------------|
| B1: Feedback Loops | 30 min | None |
| B2: Reactive vs Adaptive | 30 min | B1 |
| B3: Environment Triggers | 30 min | B2 |

**Start Here**: Open `chapters/05-adaptive-robotics/beginner/B1-feedback-loops.md`

### Intermediate Tier (ROS 2 + Gazebo)

| Lesson | Duration | Prerequisites |
|--------|----------|---------------|
| I1: Behavior Switching | 45 min | B1-B3, ROS 2 basics |
| I2: Thresholds & Triggers | 45 min | I1 |
| I3: Logging & Replay | 45 min | I2 |

**Start Here**: Ensure Gazebo simulation is running, then open `chapters/05-adaptive-robotics/intermediate/I1-behavior-switching.md`

### Advanced Tier (Heuristics + Adaptation)

| Lesson | Duration | Prerequisites |
|--------|----------|---------------|
| A1: Weighted Scoring | 60 min | I1-I3 |
| A2: Memory Adjustment | 60 min | A1 |
| A3: Meta-Control | 60 min | A2 |

**Start Here**: Complete intermediate tier, then open `chapters/05-adaptive-robotics/advanced/A1-weighted-scoring.md`

---

## First Exercise: See Behavior Switching

### Step 1: Launch Simulation

```bash
# Terminal 1
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Step 2: Monitor Sensor Data

```bash
# Terminal 2
ros2 topic echo /scan --field ranges[0]
```

### Step 3: Manually Trigger Behavior

```bash
# Terminal 3: Drive robot toward obstacle
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10
```

### Step 4: Observe Threshold Crossing

Watch Terminal 2. When `ranges[0]` drops below 0.5m, the robot should switch to avoid behavior.

---

## Configuration

### Threshold Parameters

```yaml
# config/adaptive_params.yaml
behavior_switcher:
  ros__parameters:
    default_behavior: "explore"
    activate_threshold: 0.5      # meters
    deactivate_threshold: 0.7    # meters (hysteresis)
    decision_rate: 10.0          # Hz
```

### Load Custom Config

```bash
ros2 run adaptive_robotics behavior_switcher \
  --ros-args --params-file config/adaptive_params.yaml
```

---

## Troubleshooting

### Gazebo Not Starting

```bash
# Check graphics driver
glxinfo | grep "OpenGL version"

# Try software rendering
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### No Sensor Data

```bash
# List active topics
ros2 topic list

# Check if /scan exists
ros2 topic info /scan

# If missing, restart Gazebo
```

### Behavior Not Switching

1. Check threshold values in config
2. Verify sensor readings with `ros2 topic echo /scan`
3. Check decision logs in `logs/` directory
4. Increase `decision_rate` if sluggish

---

## Next Steps

After completing quickstart:

1. **Beginner**: Work through B1-B3 lessons
2. **Intermediate**: Implement behavior switcher from scratch
3. **Advanced**: Add heuristic scoring and memory adaptation

---

## Resources

| Resource | Location |
|----------|----------|
| Spec | `specs/005-adaptive-robotics-chapter/spec.md` |
| Data Model | `specs/005-adaptive-robotics-chapter/data-model.md` |
| Research | `specs/005-adaptive-robotics-chapter/research.md` |
| ROS 2 Docs | https://docs.ros.org/en/humble/ |
| TurtleBot3 | https://emanual.robotis.com/docs/en/platform/turtlebot3/ |

---

*End of Quickstart*
