# Quickstart Guide: Chapter 1 - The Robotic Nervous System (ROS 2)

**Branch**: `001-ros2-chapter` | **Date**: 2025-12-20
**Purpose**: Reader onboarding and environment setup guide

---

## Welcome

This chapter introduces ROS 2 (Robot Operating System 2) as the "nervous system" of robots. Just as your nervous system coordinates sensory input and motor output, ROS 2 coordinates the flow of data between sensors, processors, and actuators in robotic systems.

**What You'll Learn**:
- What ROS 2 is and why it matters for robotics
- Core concepts: nodes, topics, services, and actions
- How to write Python code that controls robots
- How to describe robot structure with URDF
- Advanced patterns for AI integration

**Time Investment**: 6-12 hours total (2-4 hours per tier)

---

## Prerequisites

Before starting this chapter, ensure you have:

| Requirement | Details |
|-------------|---------|
| **Operating System** | Ubuntu 22.04 LTS (recommended) |
| **Alternative OS** | Windows 10/11 with WSL2, macOS with Docker |
| **RAM** | 8 GB minimum, 16 GB recommended |
| **Disk Space** | 10 GB free for ROS 2 installation |
| **Internet** | Required for package installation |
| **Python** | Basic familiarity (variables, functions, classes) |
| **Command Line** | Basic terminal usage |

---

## Environment Setup

### Option A: Ubuntu 22.04 (Recommended)

```bash
# 1. Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Enable Universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# 3. Add ROS 2 apt repository
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 5. Source the environment
source /opt/ros/humble/setup.bash

# 6. Add to bashrc for persistence
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Option B: Windows with WSL2

```powershell
# 1. Enable WSL2 (PowerShell as Administrator)
wsl --install -d Ubuntu-22.04

# 2. Open Ubuntu terminal and follow Option A instructions
```

### Option C: macOS with Docker

```bash
# 1. Install Docker Desktop for Mac

# 2. Pull ROS 2 Humble image
docker pull osrf/ros:humble-desktop

# 3. Run container with display forwarding
docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  osrf/ros:humble-desktop \
  bash
```

---

## Verify Installation

Run these commands to verify your setup:

```bash
# Check ROS 2 version
$ ros2 --version
ros2 0.9.0

# List available packages
$ ros2 pkg list | head -5
action_msgs
action_tutorials_cpp
action_tutorials_interfaces
action_tutorials_py
ament_cmake

# Run the talker demo
$ ros2 run demo_nodes_cpp talker
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'

# In another terminal, run the listener
$ ros2 run demo_nodes_cpp listener
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
```

If all commands work, your environment is ready!

---

## Chapter Structure

```
Chapter 1: The Robotic Nervous System (ROS 2)
│
├── BEGINNER TIER (2-4 hours)
│   ├── Lesson B1: Introduction to ROS 2
│   │   └── Concepts, architecture, first demo
│   └── Lesson B2: Basic Sensors Overview
│       └── IMU, LIDAR, cameras, force sensors
│
├── INTERMEDIATE TIER (2-4 hours)
│   ├── Lesson I1: Nodes, Topics, Services, Actions
│   │   └── Python node creation, pub/sub, services
│   └── Lesson I2: Python ROS Bridge (rclpy)
│       └── Parameters, launch files, executors
│
└── ADVANCED TIER (2-4 hours)
    ├── Lesson A1: URDF & Humanoid Robot Description
    │   └── Robot modeling, visualization in RViz
    └── Lesson A2: Advanced ROS 2 Patterns & AI Integration
        └── Action servers, feedback, AI patterns
```

---

## Learning Paths

### Path 1: Complete Beginner (No Prior Experience)

1. Start with **Beginner Tier** (required)
2. Complete all exercises before moving on
3. Take breaks between lessons
4. Use AI prompts for reinforcement
5. Proceed to Intermediate Tier

**Estimated Time**: 10-12 hours total

### Path 2: Python Developer (New to ROS)

1. Skim **Beginner Tier** (focus on concepts)
2. Complete installation and verification
3. Deep dive into **Intermediate Tier**
4. Build on **Advanced Tier**

**Estimated Time**: 6-8 hours total

### Path 3: ROS 1 Veteran

1. Review ROS 2 architecture changes in B1
2. Skip B2 unless humanoid sensors are new
3. Focus on rclpy differences in Intermediate
4. Explore Advanced for action patterns

**Estimated Time**: 4-6 hours total

---

## Code Repository

All code examples are available in the repository:

```
chapters/01-ros2-nervous-system/
├── code/
│   ├── beginner/
│   │   └── demo_commands.sh
│   ├── intermediate/
│   │   ├── minimal_publisher.py
│   │   ├── minimal_subscriber.py
│   │   └── launch/
│   │       └── talker_listener_launch.py
│   └── advanced/
│       ├── fibonacci_action_server.py
│       ├── fibonacci_action_client.py
│       └── urdf/
│           └── humanoid_basic.urdf
```

### Running Examples

```bash
# Navigate to chapter code
cd chapters/01-ros2-nervous-system/code/intermediate

# Source ROS 2 (if not in bashrc)
source /opt/ros/humble/setup.bash

# Run a Python example
python3 minimal_publisher.py

# In another terminal
python3 minimal_subscriber.py
```

---

## Troubleshooting

### "ros2: command not found"

```bash
# Solution: Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Or add to bashrc permanently
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### "Package not found" errors

```bash
# Solution: Install missing packages
sudo apt update
sudo apt install ros-humble-<package-name>
```

### Python import errors

```bash
# Solution: Ensure ROS 2 Python packages are in path
source /opt/ros/humble/setup.bash
python3 -c "import rclpy; print('Success!')"
```

### RViz not displaying

```bash
# For WSL2: Install VcXsrv or WSLg
# For Docker: Ensure X11 forwarding is configured
# For VM: Enable 3D acceleration
```

---

## Getting Help

### AI Assistant

Each lesson includes AI-assisted prompts. Use them to:
- Clarify concepts
- Debug issues
- Explore extensions

### Community Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS Discourse](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/)

---

## Success Checklist

Before moving to the next tier, verify:

### Beginner Tier
- [ ] ROS 2 installed and verified
- [ ] Can run talker/listener demo
- [ ] Understand nodes, topics, services, actions conceptually
- [ ] Know the purpose of humanoid robot sensors

### Intermediate Tier
- [ ] Can create and run Python publisher/subscriber
- [ ] Understand service request/response pattern
- [ ] Can use parameters in nodes
- [ ] Can write and run launch files

### Advanced Tier
- [ ] Can read and write basic URDF files
- [ ] Can visualize robot in RViz2
- [ ] Understand action server/client pattern
- [ ] Know how AI agents interface with ROS 2

---

## Next Steps

Once you complete this chapter, you'll be ready for:

- **Chapter 2**: The Digital Twin (Gazebo & Unity)
- **Chapter 4**: AI-Robot Brain (NVIDIA Isaac)

Good luck with your robotics journey!
