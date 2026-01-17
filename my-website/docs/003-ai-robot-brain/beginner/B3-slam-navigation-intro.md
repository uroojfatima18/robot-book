---
id: b_lesson3_slam_nav
title: "SLAM and Navigation Introduction"
sidebar_position: 29
tier: beginner
chapter: chapter_3_ai_brain
estimated_time: "60-90 minutes"
prerequisites: ["b_lesson2_sensors"]
---

# B3: SLAM and Navigation Concepts

**How Robots Map Their World and Find Their Way**

---

## Learning Objectives

By the end of this lesson, you will be able to:
- Explain what SLAM is and why it's challenging
- Describe the localization-mapping chicken-and-egg problem
- Identify the components of autonomous navigation
- Understand the difference between global and local planning

---

## Prerequisites

- Completed [B1: Introduction to Robotic Perception](B1-introduction-perception.md)
- Completed [B2: Understanding Sensor Types](B2-sensor-types.md)
- Basic understanding of coordinate frames ([Gazebo Refresher](refresher-gazebo.md))

---

## Theory: The SLAM Problem

### What is SLAM?

**SLAM** stands for **Simultaneous Localization and Mapping**. It's the process by which a robot:

1. **Builds a map** of an unknown environment
2. **Localizes itself** within that map
3. Does **both at the same time**

### The Chicken-and-Egg Problem

SLAM is fundamentally challenging because:

- **To build a map**, you need to know where you are (localization)
- **To know where you are**, you need a map (mapping)

Neither can be solved independently - they must be solved together!

### How SLAM Works (Conceptual)

```
┌─────────────────────────────────────────────────────────────┐
│                         SLAM Loop                            │
│                                                              │
│   ┌──────────┐     ┌──────────┐     ┌──────────┐           │
│   │  Sensor  │────>│  Feature │────>│   Map    │           │
│   │   Data   │     │ Matching │     │  Update  │           │
│   └──────────┘     └──────────┘     └──────────┘           │
│        │                                  │                  │
│        │           ┌──────────┐          │                  │
│        └──────────>│   Pose   │<─────────┘                  │
│                    │ Estimate │                              │
│                    └──────────┘                              │
└─────────────────────────────────────────────────────────────┘
```

1. **Sensor data** arrives (LIDAR scan, camera image)
2. **Features** are extracted and matched to existing map
3. **Pose estimate** is updated based on matches
4. **Map** is updated with new observations
5. **Loop closure**: When revisiting a location, correct accumulated drift

### Types of SLAM

| Type | Sensor | Map Output | Use Case |
|------|--------|------------|----------|
| **2D SLAM** | 2D LIDAR | Occupancy grid (2D) | Indoor mobile robots |
| **3D SLAM** | 3D LIDAR | Point cloud / mesh | Autonomous vehicles |
| **Visual SLAM** | Camera | Feature map | Drones, AR/VR |
| **RGB-D SLAM** | RGBD camera | Dense 3D map | Manipulation |

### SLAM Output: The Occupancy Grid

Most 2D SLAM systems produce an **occupancy grid map**:

```
┌───┬───┬───┬───┬───┬───┬───┬───┐
│   │   │ █ │ █ │ █ │   │   │   │  █ = Occupied (wall)
├───┼───┼───┼───┼───┼───┼───┼───┤
│   │   │   │   │ █ │   │   │   │  ░ = Unknown
├───┼───┼───┼───┼───┼───┼───┼───┤
│   │ R │   │   │ █ │   │   │   │  (blank) = Free
├───┼───┼───┼───┼───┼───┼───┼───┤
│ █ │ █ │   │   │ █ │ █ │ █ │ █ │  R = Robot
├───┼───┼───┼───┼───┼───┼───┼───┤
│ ░ │ ░ │   │   │   │   │ ░ │ ░ │
└───┴───┴───┴───┴───┴───┴───┴───┘
```

Each cell stores probability: 0 (free) to 1 (occupied), with 0.5 meaning unknown.

---

## Theory: Autonomous Navigation

Once we have a map, how does the robot navigate? Navigation has three main components:

![Navigation Architecture](diagrams/navigation-architecture.svg)

*Alt-text: Navigation architecture diagram showing three layers. Global Planning (top, blue) computes path from current position to goal. Local Planning (middle, sky blue) avoids obstacles while following global path. Execution Layer (bottom, green) sends velocity commands to motors. Arrows show data flow between layers and to sensors and motors.*

### 1. Global Planning

**Goal**: Find a path from current position to goal on the map.

**Algorithms**:
- **A\***: Classic pathfinding, optimal but can be slow
- **Dijkstra**: Finds shortest path, explores more than A*
- **NavFn**: ROS 2/Nav2's default, based on wavefront propagation

**Output**: A sequence of waypoints (the global path)

### 2. Local Planning

**Goal**: Follow the global path while avoiding dynamic obstacles.

**Algorithms**:
- **DWB (Dynamic Window)**: Samples velocities, simulates trajectories
- **TEB (Timed Elastic Band)**: Optimizes trajectory in real-time
- **MPPI (Model Predictive Path Integral)**: Modern sampling-based approach

**Output**: Velocity commands (linear and angular velocity)

### 3. Execution Layer

**Goal**: Convert velocity commands to motor signals.

**Responsibilities**:
- Motor control
- Odometry publishing
- Safety limits (max speed, acceleration)

### Recovery Behaviors

What happens when the robot gets stuck?

1. **Clear costmaps**: Reset obstacle layer, try again
2. **Spin**: Rotate in place to get new sensor data
3. **Back up**: Move away from obstacle
4. **Wait**: Wait for dynamic obstacle to move

---

## The Navigation Stack (Nav2)

ROS 2's navigation stack is called **Nav2**. It provides:

| Component | Function |
|-----------|----------|
| **Map Server** | Loads and serves the map |
| **AMCL** | Localization (where am I on the map?) |
| **Planner Server** | Global path planning |
| **Controller Server** | Local planning / trajectory following |
| **Behavior Server** | Recovery behaviors |
| **BT Navigator** | Coordinates everything via behavior tree |

### Nav2 Lifecycle

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  Unconfigured  │────>│  Inactive  │────>│   Active   │
└─────────────┘    └─────────────┘    └─────────────┘
                           │                    │
                           │                    ▼
                           │            ┌─────────────┐
                           └───────────>│  Finalized  │
                                        └─────────────┘
```

Nav2 uses **lifecycle nodes** that must be configured and activated before they work.

---

## Demonstration: Viewing Navigation in Action

### Step 1: Launch Navigation Simulation

```bash
# Terminal 1: Launch Gazebo with TurtleBot3
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch Nav2 with pre-made map
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    map:=/opt/ros/${ROS_DISTRO}/share/turtlebot3_navigation2/map/map.yaml
```

### Step 2: Set Initial Pose

In RViz2:
1. Click **2D Pose Estimate** button
2. Click on the map where the robot actually is
3. Drag to indicate orientation

### Step 3: Send Navigation Goal

1. Click **Nav2 Goal** button
2. Click destination on the map
3. Watch the robot navigate!

### Step 4: Observe the Components

```bash
# See the global path
ros2 topic echo /plan

# See local velocity commands
ros2 topic echo /cmd_vel

# See the costmap
ros2 topic echo /local_costmap/costmap
```

---

## Key Concepts Summary

| Concept | Definition |
|---------|------------|
| **SLAM** | Simultaneous Localization and Mapping |
| **Occupancy Grid** | 2D map with obstacle probabilities |
| **Global Planning** | Path from start to goal |
| **Local Planning** | Real-time obstacle avoidance |
| **Loop Closure** | Correcting drift when revisiting locations |
| **Nav2** | ROS 2 navigation stack |

---

## Hands-On Exercise

### Exercise B3.1: SLAM Conceptual Questions

Answer the following without code:

1. **Why is SLAM called "simultaneous"?**
   - Why can't we map first, then localize?

2. **Loop Closure Scenario**:
   - A robot drives in a large loop and returns to its starting point
   - The odometry says it's 2 meters away from start
   - What does this mean, and how does loop closure help?

3. **Planning Trade-offs**:
   - A global path goes through a narrow doorway
   - A dynamic obstacle (person) blocks the doorway
   - What should happen?

<details>
<summary>Click for answers</summary>

1. SLAM is "simultaneous" because mapping requires knowing your pose (to place observations correctly) and localization requires a map (to match observations against). Neither can happen first - they must happen together.

2. The 2m error is odometry drift (wheel slip, measurement errors). Loop closure detects "I've been here before" and corrects the entire trajectory, spreading the error throughout the path.

3. The local planner should detect the obstacle and either: (a) stop and wait, (b) request a replan, or (c) find a local detour. If stuck too long, recovery behaviors activate.

</details>

### Exercise B3.2: Navigation Components Matching

Match each Nav2 component to its responsibility:

| Component | Responsibility |
|-----------|---------------|
| Map Server | ? |
| AMCL | ? |
| Planner Server | ? |
| Controller Server | ? |

Options:
- A. Computes global path to goal
- B. Generates velocity commands
- C. Loads and serves the map
- D. Estimates robot pose on map

<details>
<summary>Click for answers</summary>

- Map Server → C. Loads and serves the map
- AMCL → D. Estimates robot pose on map
- Planner Server → A. Computes global path to goal
- Controller Server → B. Generates velocity commands

</details>

---

## AI Agent Assisted Prompts

### Prompt 1: SLAM Challenges
```
I'm building a robot that needs to map a large warehouse with many similar-
looking aisles. What challenges might SLAM face in this environment? What
sensors or techniques could help?
```

### Prompt 2: Navigation Safety
```
My mobile robot navigates near humans. What safety considerations should I
build into the navigation system? How can I ensure the robot stops in time
when a person walks in front of it?
```

### Prompt 3: Map Representation Trade-offs
```
Compare occupancy grid maps vs feature-based maps for mobile robot navigation.
What are the advantages and disadvantages of each? When would I choose one
over the other?
```

---

## Summary

In this lesson, you learned:

1. **SLAM** solves localization and mapping simultaneously
2. The **chicken-and-egg problem** makes SLAM fundamentally challenging
3. **Occupancy grids** represent maps as probability grids
4. **Navigation** has three layers: global planning, local planning, and execution
5. **Nav2** is ROS 2's navigation stack with lifecycle-managed nodes

---

## Next Steps

**Congratulations!** You've completed the Beginner tier!

- **Test yourself**: Complete [Beginner Exercises](../exercises/beginner-exercises.md)
- **Ready for more?**: Continue to [Intermediate Tier](../intermediate/I1-camera-depth-processing.md)

You now understand the fundamental concepts of robotic perception and navigation. The Intermediate tier will teach you to implement these systems with actual code!
