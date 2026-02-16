---
id: chapter_3_glossary
title: "AI Robot Brain Glossary"
sidebar_position: 25
chapter: chapter_3_ai_brain
---

# AI Robot Brain Glossary

A consolidated reference of key terms used throughout Chapter 3. Refer back to this glossary whenever you encounter unfamiliar terminology.

---

## Perception & Sensing

### Perception
The process of transforming raw sensor data into meaningful information about the environment. Includes preprocessing, feature extraction, and semantic understanding.

### Perception Pipeline
A series of processing stages that convert raw sensor data into actionable information: Sensing → Preprocessing → Feature Extraction → Interpretation.

### cv_bridge
A ROS 2 package that converts between ROS image messages and OpenCV image formats, enabling image processing in ROS 2 nodes.

### Depth Image
An image where each pixel represents the distance from the camera to that point in the scene. Used for 3D perception and obstacle detection.

### Point Cloud
A collection of 3D points representing surfaces in the environment. Generated from LIDAR or depth cameras.

### RGB-D Camera
A camera that captures both color (RGB) and depth (D) information simultaneously. Examples: Intel RealSense, Microsoft Kinect.

### Feature Extraction
The process of identifying meaningful patterns or characteristics in sensor data, such as edges, corners, or objects.

---

## SLAM (Simultaneous Localization and Mapping)

### SLAM
Simultaneous Localization and Mapping - the process of building a map of an unknown environment while simultaneously determining the robot's position within that map.

### SLAM Toolbox
A ROS 2 package that implements SLAM algorithms for 2D mapping using laser scan data.

### Occupancy Grid
A 2D grid representation of the environment where each cell indicates whether that space is occupied, free, or unknown.

### Localization
The process of determining a robot's position and orientation (pose) within a known map.

### Mapping
The process of building a representation of the environment from sensor data.

### Loop Closure
The process of recognizing when a robot has returned to a previously visited location, enabling correction of accumulated drift in the map.

### Pose
The position (x, y, z) and orientation (roll, pitch, yaw) of a robot in space.

---

## Navigation

### Nav2
The Navigation 2 stack for ROS 2 - a complete autonomous navigation system including path planning, obstacle avoidance, and recovery behaviors.

### Global Planner
A path planning algorithm that computes a complete path from the robot's current position to the goal, considering the known map.

### Local Planner
A path planning algorithm that generates immediate motion commands based on the global plan and local sensor data, enabling dynamic obstacle avoidance.

### Costmap
A 2D grid that assigns a cost value to each cell, representing how difficult or dangerous it is for the robot to traverse that space.

### Static Layer
A costmap layer derived from the pre-built map, representing permanent obstacles.

### Obstacle Layer
A costmap layer that marks obstacles detected by sensors in real-time.

### Inflation Layer
A costmap layer that adds safety margins around obstacles by inflating their cost values.

### Recovery Behavior
Predefined actions the robot takes when navigation fails, such as rotating in place or backing up.

### Behavior Tree
A hierarchical structure for organizing robot behaviors and decision-making logic. Used in Nav2 for navigation control.

### DWA (Dynamic Window Approach)
A local planning algorithm that generates velocity commands by simulating trajectories and selecting the best one based on a cost function.

---

## Coordinate Frames & Transforms

### TF2
The Transform library in ROS 2 that tracks coordinate frame relationships over time.

### base_link
The robot's base coordinate frame, typically at the center of the robot's footprint.

### map
The fixed world coordinate frame representing the global map.

### odom
The odometry frame that tracks the robot's motion from a starting position. Subject to drift over time.

### camera_frame
The coordinate frame attached to a camera sensor.

### Transform
A mathematical representation of the position and orientation relationship between two coordinate frames.

---

## Reinforcement Learning

### Reinforcement Learning (RL)
A machine learning paradigm where an agent learns to make decisions by interacting with an environment and receiving rewards or penalties.

### Agent
The learning entity in RL that observes states, takes actions, and receives rewards.

### Environment
The world or simulation in which the RL agent operates and learns.

### State
A representation of the current situation or configuration of the environment.

### Action
A decision or command that the agent can execute to change the environment state.

### Reward
A scalar feedback signal that indicates how good or bad an action was in a given state.

### Policy
A mapping from states to actions that defines the agent's behavior. Can be deterministic or stochastic.

### MDP (Markov Decision Process)
A mathematical framework for modeling decision-making where outcomes depend only on the current state and action.

### PPO (Proximal Policy Optimization)
A popular RL algorithm that learns policies by optimizing a clipped objective function. Known for stability and sample efficiency.

### SAC (Soft Actor-Critic)
An off-policy RL algorithm that learns both a policy and a value function, optimizing for maximum entropy.

### Episode
A complete sequence of states, actions, and rewards from an initial state to a terminal state.

### Exploration vs. Exploitation
The trade-off between trying new actions to discover better strategies (exploration) and using known good actions (exploitation).

---

## Simulation & Deployment

### Sim-to-Real Transfer
The process of transferring policies or behaviors learned in simulation to real-world robots.

### Domain Randomization
A technique for improving sim-to-real transfer by training on varied simulation parameters (lighting, textures, physics).

### Reality Gap
The difference between simulated and real-world environments that can cause learned behaviors to fail when deployed.

### ONNX (Open Neural Network Exchange)
A format for representing trained neural network models that can be loaded and executed across different frameworks.

### Policy Network
A neural network that implements a policy, mapping observations to actions.

### Inference
The process of using a trained model to make predictions or decisions on new data.

---

## Algorithms & Techniques

### Kalman Filter
An algorithm that estimates the state of a system from noisy measurements, commonly used for sensor fusion and localization.

### Particle Filter
A probabilistic localization algorithm that represents the robot's pose as a set of weighted particles.

### A* (A-star)
A graph search algorithm used for path planning that finds the shortest path by using heuristics.

### Dijkstra's Algorithm
A graph search algorithm that finds the shortest path between nodes, used as a basis for many path planners.

### Gradient Descent
An optimization algorithm used to train neural networks by iteratively adjusting parameters to minimize a loss function.

---

## Common Abbreviations

| Abbreviation | Full Name |
|--------------|-----------|
| SLAM | Simultaneous Localization and Mapping |
| Nav2 | Navigation 2 |
| RL | Reinforcement Learning |
| PPO | Proximal Policy Optimization |
| SAC | Soft Actor-Critic |
| MDP | Markov Decision Process |
| DWA | Dynamic Window Approach |
| RGB-D | Red Green Blue - Depth |
| LIDAR | Light Detection and Ranging |
| TF | Transform (TF2 = Transform 2) |
| ONNX | Open Neural Network Exchange |
| IMU | Inertial Measurement Unit |

---

**Next**: [Introduction](./introduction.md) | [B1: Introduction to Robotic Perception](./beginner/B1-introduction-perception.md)
