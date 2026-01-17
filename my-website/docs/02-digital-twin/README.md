---
id: chapter_2_overview
title: "Chapter 2: Digital Twin & Simulation"
sidebar_position: 11
chapter: chapter_2_digital_twin
---

# Chapter 2: Digital Twin & Simulation

> Create a virtual robot replica that mirrors and controls your physical humanoid.

## Overview

This chapter teaches you to build a **Digital Twin**—a synchronized virtual copy of a physical robot. You'll learn to run simulations in Gazebo, create custom worlds, spawn humanoid models, and build a bridge node that enables real-time bidirectional communication between simulation and reality.

**Prerequisites**: Chapter 1 (URDF Fundamentals) completed, ROS 2 Humble installed

**Time Estimate**: 6-12 hours total (2-4 hours per tier)

---

## Learning Path

### Beginner Tier
*Goal: Understand digital twin concepts and run your first simulation*

| Lesson | Title | Duration | Description |
|--------|-------|----------|-------------|
| [B1](beginner/B1-digital-twin-concepts.md) | What is a Digital Twin? | 45-60 min | Core concepts, mental models, physical-virtual sync |
| [B2](beginner/B2-first-simulation.md) | Running Your First Simulation | 60-90 min | Gazebo launch, navigation, ROS 2 integration |

**Assets**:
- [humanoid_lab.world](beginner/assets/humanoid_lab.world) - Pre-built demo world

**Outcome**: Launch Gazebo with a pre-built humanoid world and explain digital twin concepts.

---

### Intermediate Tier
*Goal: Build custom simulation worlds and spawn humanoid models*

| Lesson | Title | Duration | Description |
|--------|-------|----------|-------------|
| [I1](intermediate/I1-building-worlds.md) | Building Simulation Worlds | 60-90 min | World file creation, physics config, lighting |
| [I2](intermediate/I2-spawning-models.md) | Spawning and Controlling Models | 60-90 min | URDF spawning, joint control, ROS 2 launch |

**Assets**:
- [simple_lab.world](intermediate/assets/simple_lab.world) - Template world file
- [spawn_humanoid.launch.py](intermediate/assets/launch/spawn_humanoid.launch.py) - ROS 2 launch file
- [joint_commander.py](intermediate/assets/src/joint_commander.py) - Joint control example

**Outcome**: Create your own `.world` file and spawn a URDF humanoid with joint control.

---

### Advanced Tier
*Goal: Implement bidirectional data synchronization for digital twin loop*

| Lesson | Title | Duration | Description |
|--------|-------|----------|-------------|
| [A1](advanced/A1-data-synchronization.md) | Digital Twin Architecture | 60-90 min | Data sync patterns, topic mapping, latency design |
| [A2](advanced/A2-building-bridge.md) | Building the Bridge Node | 90-120 min | Bridge implementation, latency monitoring, AI training |

**Assets**:
- [bridge_node.py](advanced/src/bridge_node.py) - Bidirectional sync node
- [latency_monitor.py](advanced/src/latency_monitor.py) - Latency tracking node
- [sensor_streamer.py](advanced/src/sensor_streamer.py) - AI training data streamer
- [AI Training Architecture](advanced/assets/diagrams/ai-training-architecture.md) - RL integration diagram

**Outcome**: Build a bridge node with &lt;50ms latency for real-time robot-simulation sync.

---

## Exercises

| Exercise | Tier | Focus | Time |
|----------|------|-------|------|
| [Exercise 01: Launch World](exercises/exercise-01-launch-world.md) | Beginner | Launch and explore Gazebo world | 30-45 min |
| [Exercise 02: Create World](exercises/exercise-02-create-world.md) | Intermediate | Build a custom simulation world | 45-60 min |
| [Exercise 03: Build Bridge](exercises/exercise-03-build-bridge.md) | Advanced | Implement digital twin bridge | 60-90 min |

---

## Key Concepts

| Concept | Definition | Target |
|---------|------------|--------|
| **Digital Twin** | Virtual replica synchronized with physical robot | Real-time sync |
| **Gazebo Classic** | Physics simulation platform for robotics | Version 11.x |
| **Real-Time Factor (RTF)** | Simulation speed vs real time | >= 0.8 |
| **Bridge Node** | ROS 2 node connecting sim to physical robot | Bidirectional |
| **Latency Threshold** | Maximum acceptable delay | &lt;= 50ms |

---

## Technical Requirements

| Component | Version | Notes |
|-----------|---------|-------|
| Ubuntu | 22.04 LTS | Native or WSL2 |
| ROS 2 | Humble | With rclpy |
| Gazebo | Classic (11.x) | gazebo_ros_pkgs |
| Python | 3.10+ | Standard library |

---

## Troubleshooting

### Gazebo Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| **Gazebo won't start** | Missing dependencies | `sudo apt install gazebo11 ros-humble-gazebo-ros-pkgs` |
| **Black screen** | GPU driver issue | Try `export LIBGL_ALWAYS_SOFTWARE=1` |
| **Slow startup** | First-time model download | Wait for models to download; check `~/.gazebo/models/` |
| **World file not found** | Incorrect path | Use absolute path or set `GAZEBO_RESOURCE_PATH` |

### RTF Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| **RTF &lt; 0.5** | Complex world | Simplify collision geometry, reduce polygon count |
| **RTF drops over time** | Memory leak | Restart Gazebo; check for orphaned processes |
| **RTF spikes** | Physics solver issues | Increase solver iterations in world file |
| **RTF shows 0** | Simulation paused | Press Space in Gazebo to resume |

### URDF Spawning Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| **Model explodes** | Invalid inertia | Check all links have positive definite inertia |
| **Model falls through floor** | Missing collision | Add `<collision>` tags to all links |
| **Joints don't move** | Missing transmission | Add `<transmission>` and controller |
| **Spawn service fails** | Topic not available | Ensure gazebo_ros is launched first |

### Latency Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| **Latency > 100ms** | Network congestion | Use wired connection, reduce message size |
| **Latency spikes** | Python GC pauses | Use C++ for production, tune GC |
| **Latency grows over time** | Queue buildup | Reduce publish rate, use BEST_EFFORT QoS |
| **Clock skew** | Unsync'd clocks | Use `/use_sim_time` parameter |

### Bridge Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| **No messages received** | QoS mismatch | Align publisher/subscriber QoS profiles |
| **Mode not changing** | Parameter not set | Use `ros2 param set /bridge_node mode "live"` |
| **Connection lost alerts** | Timeout too short | Increase watchdog timeout |
| **Safety blocks all commands** | Limits too strict | Adjust max_joint_position/velocity params |

---

## Quick Reference

### Common Commands

```bash
# Launch Gazebo with world
gazebo --verbose path/to/world.world

# Check Gazebo stats
gz stats

# Spawn URDF model
ros2 run gazebo_ros spawn_entity.py -entity robot -file robot.urdf -x 0 -y 0 -z 1

# List topics
ros2 topic list | grep -E "(joint|sim|hw)"

# Echo joint states
ros2 topic echo /joint_states --once

# Reset simulation
ros2 service call /reset_simulation std_srvs/srv/Empty
```

### Environment Setup

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Set Gazebo paths (if needed)
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/your_models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/ros2_ws/src/your_worlds
```

---

## Chapter Contents

```
02-digital-twin/
├── README.md                          # This file
├── beginner/
│   ├── B1-digital-twin-concepts.md    # What is a Digital Twin?
│   ├── B2-first-simulation.md         # Running Your First Simulation
│   └── assets/
│       └── humanoid_lab.world         # Demo world file
├── intermediate/
│   ├── I1-building-worlds.md          # Building Simulation Worlds
│   ├── I2-spawning-models.md          # Spawning and Controlling Models
│   └── assets/
│       ├── simple_lab.world           # Template world
│       ├── launch/
│       │   └── spawn_humanoid.launch.py
│       └── src/
│           └── joint_commander.py
├── advanced/
│   ├── A1-data-synchronization.md     # Digital Twin Architecture
│   ├── A2-building-bridge.md          # Building the Bridge Node
│   ├── src/
│   │   ├── bridge_node.py             # Bridge node implementation
│   │   ├── latency_monitor.py         # Latency tracking
│   │   └── sensor_streamer.py         # AI training streamer
│   └── assets/
│       └── diagrams/
│           └── ai-training-architecture.md
└── exercises/
    ├── exercise-01-launch-world.md    # Beginner exercise
    ├── exercise-02-create-world.md    # Intermediate exercise
    └── exercise-03-build-bridge.md    # Advanced exercise
```

---

## Navigation

| Previous | Home | Next |
|----------|------|------|
| [Chapter 1: ROS 2 & Nervous System](../01-ros2-nervous-system/README.md) | [Book Home](../../README.md) | Chapter 3: Coming Soon |
