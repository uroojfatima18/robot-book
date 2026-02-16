---
id: chapter_1_beginner_tier
title: "Beginner Tier: ROS 2 Fundamentals"
sidebar_position: 4
tier: beginner
chapter: chapter_1_ros2
estimated_time: "2-4 hours"
---

# Beginner Tier: ROS 2 Fundamentals

## Welcome to the Beginner Tier

This tier introduces the foundational concepts of ROS 2 with zero assumptions about prior robotics knowledge. You'll learn *what* ROS 2 is, *why* it's important, and *how* to install and run it.

---

## Tier Overview

```
🟢 BEGINNER TIER - Foundation & Fundamentals
═══════════════════════════════════════════════════

What You'll Learn:
• ROS 2 architecture and core concepts (nodes, topics, services, actions)
• Installation and setup on Ubuntu 22.04 (or WSL2/Docker)
• Running your first ROS 2 demonstration
• Sensor systems overview for humanoid robots
• Key terminology and mental models

What You'll Build:
• Working ROS 2 installation
• Understanding of pub/sub communication
• Foundation for intermediate hands-on coding
```

---

## Learning Objectives

By the end of the Beginner tier, you will be able to:

1. **Define** what ROS 2 is and how it fits into robot architecture
2. **Explain** the nervous system metaphor for understanding ROS 2
3. **Differentiate** between ROS 1 and ROS 2
4. **Describe** the four core ROS 2 communication patterns
5. **Install** ROS 2 Humble on your system
6. **Run** and observe the talker/listener demo
7. **Identify** sensor types used in humanoid robots
8. **Understand** the role of simulation in robot development

---

## Prerequisites

Before starting this tier, you should have:

- **Operating System**: Ubuntu 22.04 LTS (native, WSL2, or Docker)
  - WSL2 Installation Guide: https://docs.microsoft.com/windows/wsl/
  - Docker Setup: Docker Hub Ubuntu 22.04 image
- **Basic Command-Line Skills**: Comfortable navigating directories and running shell commands
  - `cd`, `ls`, `mkdir`, `apt-get install`
- **Text Editor or IDE**: Any text editor will do (VS Code recommended)
- **Internet Connection**: For downloading ROS 2 and dependencies

**Knowledge Assumptions**: None. This tier starts from zero.

---

## Lessons in This Tier

### Lesson B1: Introduction to ROS 2
**Duration**: 1-2 hours

What is ROS 2? Why is it not a real operating system? Understand the middleware concept, the nervous system metaphor, and the key differences between ROS 1 and ROS 2.

**Key Topics**:
- What is ROS 2 (and what it's not)
- The nervous system metaphor
- Core building blocks: nodes, topics, services, actions
- Why ROS 2 over ROS 1
- Installation guide
- Running the talker/listener demo

**Outcomes**:
- ✅ ROS 2 installed and verified
- ✅ First demo running successfully
- ✅ Understanding of core concepts

**File**: [B1: Introduction to ROS 2](./01-intro-to-ros2.md)

---

### Lesson B2: Basic Sensors Overview
**Duration**: 1-2 hours

What sensors do humanoid robots use? How do they work? What does the data look like? This lesson surveys the main sensor types without implementation details—purely conceptual understanding.

**Key Topics**:
- IMU (Inertial Measurement Unit): What it measures, how it's used
- LIDAR (Light Detection and Ranging): 2D/3D scanning, point clouds
- Depth Cameras: RGB + Depth data
- Force/Torque Sensors: Grip control and balance
- Encoders: Motor feedback
- Simulation vs. Real Sensors
- Sensor data visualization in RViz2

**Outcomes**:
- ✅ Know the major sensor types
- ✅ Understand sensor data shapes and units
- ✅ Grasp how sensors connect to the ROS 2 ecosystem

**File**: [B2: Basic Sensors Overview](./02-sensors-overview.md)

---

## Progression & Scaffolding

The Beginner tier is scaffolded to build understanding progressively:

```
Lesson B1                          Lesson B2
└─ What is ROS 2?                  └─ What sensors feed ROS 2?
   ├─ Theory (nervous system)          ├─ Sensor types (IMU, LIDAR, camera)
   ├─ Installation (hands-on)          ├─ Data formats (point clouds, etc.)
   ├─ First demo (observation)         └─ Visualization (RViz2 intro)
   └─ Understanding (mental model)
                ↓
        Ready for Intermediate Tier
     (where we START writing code)
```

---

## Estimated Timeline

| Lesson | Duration | Cumulative | Notes |
|--------|----------|-----------|-------|
| B1: Intro to ROS 2 | 1-2 hours | 1-2 hours | Includes installation |
| B2: Sensors Overview | 1-2 hours | 2-4 hours | Conceptual, no coding |
| **Beginner Total** | **2-4 hours** | **2-4 hours** | Pure foundation |

---

## What You'll NOT Do (Yet)

This tier is intentionally focused on understanding. You will NOT:

- Write Python code (that's Intermediate)
- Build robot URDF models (that's Advanced)
- Implement action servers (that's Advanced)
- Deploy to real robots (that's beyond this chapter)

This keeps the cognitive load manageable and ensures you have a solid mental model before coding.

---

## Hands-On Exercises

At the end of this tier, you'll complete:

- **Exercise B1**: Install ROS 2 and verify installation
- **Exercise B2**: Run talker/listener demo and explore output
- **Exercise B3**: Identify sensor data in RViz2 (simulator)
- **Checkpoint Quiz**: Conceptual questions on ROS 2 architecture

All exercises are in [Beginner Exercises](./exercises/beginner-exercises.md).

---

## AI-Assisted Learning

Stuck? Use these AI prompts to get help:

- **Clarification**: "Explain nodes and topics as if I'm completely new to robotics"
- **Troubleshooting**: "My ROS 2 installation failed at step X. How do I fix this?"
- **Visualization**: "What does topic pub/sub look like in a humanoid robot?"
- **Analogies**: "Explain services vs. topics using a real-world analogy"

See [Beginner AI Prompts](../ai-prompts/beginner-prompts.md) for a full library.

---

## What's Next?

After completing this tier:

1. **Review** the key takeaways in each lesson
2. **Complete** the exercises in the exercises folder
3. **Ask** clarifying questions in the AI prompts if needed
4. **Move Forward** to the **Intermediate Tier** where you'll write your first Python ROS 2 nodes

The Intermediate tier assumes you've completed this tier and understand the core concepts. There, you'll get hands-on with code.

---

## Resources

- **Official ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **ROS 2 Installation Guide**: https://docs.ros.org/en/humble/Installation.html
- **Understanding URDF**: https://wiki.ros.org/urdf
- **Gazebo Simulation**: https://gazebosim.org/
- **RViz2 Documentation**: https://github.com/ros2/rviz

---

## Ready to Start?

Begin with **[Lesson B1: Introduction to ROS 2](./01-intro-to-ros2.md)**.

---

*"The best way to learn robotics is to build understanding first. Let's begin with the fundamentals."*
