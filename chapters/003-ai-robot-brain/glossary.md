# Chapter 3: AI-Robot Brain - Glossary

## A

**Action Space**: The set of all possible actions a robot can take in reinforcement learning. For a mobile robot, this might include forward/backward movement and rotation.

**Actor-Critic**: A reinforcement learning architecture that combines policy-based (actor) and value-based (critic) methods.

**Autonomous Navigation**: The ability of a robot to move from one location to another without human intervention, handling obstacles and dynamic environments.

---

## B

**Behavior Tree**: A hierarchical structure for organizing robot behaviors, commonly used in Nav2 for complex navigation logic.

**BEST_EFFORT**: A ROS 2 QoS policy that prioritizes speed over reliability, suitable for high-frequency sensor data.

---

## C

**Costmap**: A 2D grid representation of the environment where each cell has a cost value indicating traversability. Used by Nav2 for path planning.

**cv_bridge**: A ROS 2 package that converts between ROS image messages and OpenCV image formats.

---

## D

**Depth Camera**: A sensor that measures the distance to objects in the scene, producing a depth image where each pixel represents distance.

**DWA (Dynamic Window Approach)**: A local path planning algorithm that considers the robot's dynamics and velocity constraints.

---

## E

**Episode**: In reinforcement learning, a complete sequence of states, actions, and rewards from start to terminal state.

**Exploration**: In reinforcement learning, the process of trying new actions to discover their outcomes, balancing with exploitation of known good actions.

---

## F

**Feature Extraction**: The process of identifying and extracting meaningful patterns or characteristics from sensor data.

**Footprint**: The 2D shape representing the robot's physical extent, used for collision checking in navigation.

---

## G

**Global Planner**: A path planning algorithm that computes a complete path from start to goal, considering the entire known map.

**Goal Pose**: The target position and orientation that a robot should reach during navigation.

---

## I

**IMU (Inertial Measurement Unit)**: A sensor that measures acceleration and angular velocity, used for estimating robot motion.

**Inflation Layer**: A costmap layer that inflates obstacles to account for the robot's size and provide safety margins.

---

## L

**LIDAR (Light Detection and Ranging)**: A sensor that uses laser pulses to measure distances, creating a 2D or 3D point cloud of the environment.

**Local Planner**: A path planning algorithm that computes short-term trajectories, reacting to dynamic obstacles and following the global plan.

**Localization**: The process of determining a robot's position and orientation within a known map.

---

## M

**Map**: A representation of the environment, typically a 2D occupancy grid where cells are marked as free, occupied, or unknown.

**MDP (Markov Decision Process)**: A mathematical framework for modeling decision-making in situations where outcomes are partly random and partly under the control of a decision maker.

---

## N

**Nav2**: The ROS 2 navigation stack, providing path planning, obstacle avoidance, and goal-reaching capabilities.

---

## O

**Obstacle Layer**: A costmap layer that marks cells occupied by detected obstacles from sensor data.

**Occupancy Grid**: A 2D grid map where each cell represents the probability that the space is occupied.

**ONNX (Open Neural Network Exchange)**: A format for representing machine learning models, enabling interoperability between frameworks.

---

## P

**Particle Filter**: An algorithm used in localization that represents the robot's pose as a set of weighted particles.

**Perception Pipeline**: The sequence of processing steps that transform raw sensor data into meaningful information about the environment.

**Point Cloud**: A set of 3D points representing the surfaces of objects in the environment, typically from LIDAR or depth cameras.

**Policy**: In reinforcement learning, a mapping from states to actions that defines the robot's behavior.

**PPO (Proximal Policy Optimization)**: A popular reinforcement learning algorithm that balances exploration and exploitation while maintaining stable learning.

---

## Q

**QoS (Quality of Service)**: ROS 2 policies that control message delivery reliability, durability, and other communication properties.

---

## R

**Recovery Behavior**: Actions taken by a navigation system when it gets stuck or fails to make progress toward the goal.

**RELIABLE**: A ROS 2 QoS policy that guarantees message delivery, suitable for critical data like commands.

**Reinforcement Learning (RL)**: A machine learning paradigm where an agent learns to make decisions by receiving rewards or penalties for its actions.

**Reward Function**: In reinforcement learning, a function that assigns a numerical reward to state-action pairs, guiding the learning process.

**RGB Camera**: A standard color camera that captures red, green, and blue color information.

---

## S

**SAC (Soft Actor-Critic)**: An off-policy reinforcement learning algorithm that maximizes both expected reward and entropy.

**Sensor Fusion**: The process of combining data from multiple sensors to produce more accurate and reliable information.

**Sim-to-Real Transfer**: The process of transferring policies or models trained in simulation to real-world robots.

**SLAM (Simultaneous Localization and Mapping)**: The problem of building a map of an unknown environment while simultaneously tracking the robot's location within that map.

**SLAM Toolbox**: A ROS 2 package that implements SLAM algorithms for 2D mapping and localization.

**State Space**: The set of all possible states a robot can be in, including position, velocity, sensor readings, etc.

**Static Layer**: A costmap layer that represents the static map of the environment.

---

## T

**TF2 (Transform Library 2)**: A ROS 2 system for managing coordinate frame transformations, essential for multi-sensor robots.

**Transform**: A mathematical representation of the position and orientation of one coordinate frame relative to another.

---

## V

**Value Function**: In reinforcement learning, a function that estimates the expected cumulative reward from a given state or state-action pair.

**Visual Odometry**: The process of estimating a robot's motion by analyzing changes in camera images over time.

---

## W

**Waypoint**: An intermediate goal point along a path that the robot should pass through on its way to the final destination.

---

## Additional Resources

For more detailed definitions and mathematical formulations, refer to:
- [ROS 2 Documentation](https://docs.ros.org/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [OpenCV Documentation](https://docs.opencv.org/)

---

**Note**: This glossary covers terms specific to this chapter. For ROS 2 fundamentals, refer to Chapter 1's glossary.
