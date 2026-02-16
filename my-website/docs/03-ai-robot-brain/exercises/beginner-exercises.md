---
id: chapter_3_beginner_exercises
title: "Beginner Tier Exercises"
sidebar_position: 40
chapter: chapter_3_ai_brain
section: exercises
---

# Beginner Tier Exercises

**Test Your Understanding of Perception, Sensors, and Navigation Concepts**

---

## Overview

These exercises test your understanding of the concepts from the Beginner tier:
- B1: Introduction to Robotic Perception
- B2: Understanding Sensor Types
- B3: SLAM and Navigation Concepts

Complete these exercises before moving to the Intermediate tier.

---

## Exercise 1: Perception Pipeline Ordering

**Task**: Put these perception pipeline stages in the correct order (1-4):

| Stage | Order (1-4) |
|-------|-------------|
| Feature Extraction | ___ |
| Interpretation | ___ |
| Sensing | ___ |
| Preprocessing | ___ |

**Bonus**: For each stage, give one example of what happens there.

<details>
<summary>Click for answer</summary>

1. **Sensing** - Camera captures pixels, LIDAR measures distances
2. **Preprocessing** - Noise filtering, calibration, synchronization
3. **Feature Extraction** - Edge detection, object detection, clustering
4. **Interpretation** - "This is a person," "This area is navigable"

</details>

---

## Exercise 2: Sensor Selection Challenge

**Scenario**: You're building a robot for each situation below. Choose the most appropriate sensor(s) and justify your choice.

### Scenario A: Delivery Robot in Office Building
- Environment: Indoor, well-lit hallways
- Tasks: Navigate to rooms, avoid people and furniture
- Budget: Medium

**Your answer**:
- Primary sensor: _______________
- Secondary sensor: _______________
- Justification: _______________

### Scenario B: Agricultural Drone for Crop Monitoring
- Environment: Outdoor, variable lighting
- Tasks: Fly over fields, detect plant health by color
- Budget: Low-Medium

**Your answer**:
- Primary sensor: _______________
- Justification: _______________

### Scenario C: Robot Arm for Bin Picking
- Environment: Indoor factory
- Tasks: Identify and grasp objects of different colors/sizes
- Budget: Medium-High

**Your answer**:
- Primary sensor: _______________
- Secondary sensor: _______________
- Justification: _______________

<details>
<summary>Click for suggested answers</summary>

**A: Delivery Robot**
- Primary: 2D LIDAR - reliable obstacle detection, good for SLAM
- Secondary: RGB camera - for recognizing room signs/numbers
- Why: Indoor environment suits LIDAR well; camera adds recognition capability

**B: Agricultural Drone**
- Primary: RGB camera with multispectral option
- Why: Need color for plant health analysis; must work outdoors where depth cameras struggle; lightweight for flight

**C: Bin Picking Robot**
- Primary: RGBD camera (e.g., RealSense)
- Secondary: Force/torque sensor at gripper
- Why: Need both color (object identification) and depth (grasping location)

</details>

---

## Exercise 3: SLAM Concept Questions

Answer these conceptual questions about SLAM:

### Question 3.1
**Why is SLAM called "the chicken-and-egg problem"?**

Your answer: _______________

### Question 3.2
**A robot drives in a square and returns to start. Its odometry shows it's 0.5m away from where it started. What happened and how does SLAM handle this?**

Your answer: _______________

### Question 3.3
**What does each color mean in a standard ROS 2 occupancy grid map?**
- White: _______________
- Black: _______________
- Gray: _______________

<details>
<summary>Click for answers</summary>

**3.1**: SLAM is chicken-and-egg because:
- To build a map, you need to know where you are (to place observations correctly)
- To know where you are, you need a map (to match observations against)
- Neither can be solved first - they must be solved simultaneously

**3.2**: The 0.5m error is **odometry drift** from accumulated wheel slip and measurement errors. SLAM handles this through **loop closure** - when the robot recognizes it's back at the start, it adjusts the entire trajectory to eliminate the error.

**3.3**:
- White = Free space (robot can pass)
- Black = Occupied (wall/obstacle)
- Gray = Unknown (not yet observed)

</details>

---

## Exercise 4: Navigation Components Identification

**Task**: Match each Nav2 component to its correct description.

| Component | Description |
|-----------|-------------|
| 1. Map Server | A. Generates velocity commands to follow path |
| 2. AMCL | B. Computes global path from start to goal |
| 3. Planner Server | C. Handles stuck situations (spin, backup) |
| 4. Controller Server | D. Loads and publishes the map |
| 5. Behavior Server | E. Estimates robot's pose on the map |

**Answers**: 1=___, 2=___, 3=___, 4=___, 5=___

<details>
<summary>Click for answers</summary>

1=D, 2=E, 3=B, 4=A, 5=C

</details>

---

## Exercise 5: True or False

Mark each statement as True (T) or False (F):

| # | Statement | T/F |
|---|-----------|-----|
| 1 | RGB cameras provide depth information directly | ___ |
| 2 | LIDAR works well in complete darkness | ___ |
| 3 | Time-of-Flight cameras are best for outdoor use | ___ |
| 4 | Global planning happens once, local planning happens continuously | ___ |
| 5 | SLAM can only work with LIDAR sensors | ___ |
| 6 | Occupancy grids store the probability of each cell being occupied | ___ |
| 7 | The perception pipeline always ends with sensor data | ___ |

<details>
<summary>Click for answers</summary>

1. **False** - RGB cameras only capture light intensity (color), not depth
2. **True** - LIDAR uses its own laser light source
3. **False** - ToF cameras struggle outdoors due to infrared interference from sunlight
4. **True** - Global planner finds the path once (or when replanning), local planner runs continuously to follow it
5. **False** - Visual SLAM uses cameras, RGB-D SLAM uses depth cameras
6. **True** - Each cell stores 0 (free) to 1 (occupied), with 0.5 meaning unknown
7. **False** - Pipeline ends with interpretation/understanding, not raw sensor data

</details>

---

## Exercise 6: Diagram Interpretation

Look at the perception pipeline diagram from B1 and answer:

1. What enters the pipeline at the "Sensing" stage?
2. What exits the pipeline at the "Interpretation" stage?
3. If a camera image has sensor noise, which stage removes it?
4. If the robot needs to classify "is this a person?", which stage does that?

<details>
<summary>Click for answers</summary>

1. Raw sensor signals from the physical world (photons, laser pulses, forces)
2. Semantic understanding for decision-making (e.g., "obstacle at 2m", "person detected")
3. **Preprocessing** - noise filtering happens here
4. **Interpretation** - semantic classification happens here

</details>

---

## Exercise 7: Practical Scenario Analysis

**Scenario**: A mobile robot is navigating through a crowded hallway. It has a 2D LIDAR and an RGB camera.

Answer these questions:

### 7.1 Perception
What can each sensor contribute to understanding the scene?
- LIDAR: _______________
- Camera: _______________

### 7.2 Navigation Challenge
A person suddenly walks in front of the robot's planned path. Trace what happens:
1. Which sensor detects the person first?
2. Which navigation component reacts?
3. What does the robot do?

### 7.3 Failure Mode
The hallway has a glass door. What problems might occur and why?

<details>
<summary>Click for answers</summary>

**7.1**:
- LIDAR: Distance to obstacles, wall positions, navigable space
- Camera: Person recognition, door signs, color-based identification

**7.2**:
1. LIDAR detects the person as a new obstacle (faster update rate)
2. Local planner (Controller Server) detects obstacle in trajectory
3. Robot either: stops, slows down, or finds a local detour around the person

**7.3**:
Glass is problematic because:
- LIDAR may pass through or give erratic readings (glass is semi-transparent to lasers)
- Robot might plan a path through the door if it appears as free space
- Solution: Use camera to detect visual features of the door, or add ultrasonic sensors

</details>

---

## Self-Assessment Checklist

Before moving to the Intermediate tier, confirm you can:

- [ ] Explain the four stages of the perception pipeline
- [ ] Compare RGB cameras, depth cameras, and LIDAR
- [ ] Describe what SLAM does and why it's challenging
- [ ] Explain the difference between global and local planning
- [ ] Identify the main components of Nav2
- [ ] Understand occupancy grid maps

---

## Next Steps

**Congratulations on completing the Beginner tier exercises!**

If you scored well on these exercises, you're ready for:
- [I1: Camera and Depth Data Processing](../intermediate/I1-camera-depth-processing.md)

If you struggled with any topics, review the corresponding lesson before continuing.
