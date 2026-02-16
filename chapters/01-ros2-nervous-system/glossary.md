---
id: chapter_1_glossary
title: "ROS 2 Glossary"
chapter: chapter_1_ros2
---

# ROS 2 Glossary

A consolidated reference of key terms used throughout Chapter 1. Refer back to this glossary whenever you encounter unfamiliar terminology.

---

## Core Concepts

### Node
A single-purpose, modular process that performs a specific task. Nodes communicate with each other using topics, services, and actions. Think of nodes as the "neurons" in the robot's nervous system.

**Example**: A camera node captures images; a vision node processes them.

### Topic
A named channel for asynchronous, publish-subscribe communication. Publishers send messages to topics; subscribers receive them. Topics are ideal for streaming data like sensor readings.

**Example**: `/camera/image`, `/imu/data`, `/cmd_vel`

### Message
A data structure that defines the format of information sent over topics. ROS 2 provides standard messages (e.g., `std_msgs/String`) and you can create custom messages.

**Example**: `geometry_msgs/Twist` contains linear and angular velocity.

### Service
A synchronous request-response communication pattern. A client sends a request, waits for the server to process it, and receives a response. Best for one-time operations.

**Example**: `/add_two_ints` - client sends two numbers, server returns the sum.

### Action
An asynchronous communication pattern for long-running tasks. Actions provide goals, feedback during execution, and results upon completion. Actions can be canceled mid-execution.

**Example**: Navigate to a waypoint, receiving position updates along the way.

### Publisher
A node role that sends messages to a topic. A node can have multiple publishers for different topics.

### Subscriber
A node role that receives messages from a topic. A node can subscribe to multiple topics simultaneously.

### Client
A node role that sends requests to a service or action server.

### Server
A node role that receives requests, processes them, and returns responses (service) or results with feedback (action).

---

## ROS 2 Architecture

### DDS (Data Distribution Service)
The underlying communication protocol used by ROS 2. DDS provides reliable, real-time data distribution. You rarely interact with DDS directly, but it enables ROS 2's advanced features.

### QoS (Quality of Service)
Configuration settings that control message delivery reliability, history, and durability. QoS profiles include:
- **Reliable**: Guarantees delivery (may retry)
- **Best Effort**: Fast delivery, may lose messages
- **Transient Local**: Late subscribers receive last message

### Executor
A ROS 2 component that manages callback execution. Executors determine how and when node callbacks (timers, subscriptions, services) are processed.

### Callback
A function that executes in response to an event (message received, timer fired, service called). Callbacks are the primary way nodes respond to the world.

### Callback Group
A way to organize callbacks for concurrent or sequential execution. Types include:
- **Mutually Exclusive**: Only one callback runs at a time
- **Reentrant**: Multiple callbacks can run simultaneously

---

## Python-ROS 2 (rclpy)

### rclpy
The Python client library for ROS 2. `rclpy` provides classes and functions to create nodes, publishers, subscribers, services, and actions in Python.

### rclpy.init()
Initializes the ROS 2 Python client library. Must be called before creating any nodes.

### rclpy.spin()
Blocks and processes callbacks for a node. Keeps the node running until interrupted.

### rclpy.shutdown()
Cleanly shuts down the ROS 2 Python client library.

### Node (Python class)
The base class for all ROS 2 Python nodes. Your custom nodes inherit from `rclpy.node.Node`.

---

## Robot Description

### URDF (Unified Robot Description Format)
An XML format for describing a robot's physical structure. URDF defines links (body parts), joints (connections), visual appearance, and collision geometry.

### Link
A rigid body in a URDF model. Links have visual geometry (what you see) and collision geometry (for physics simulation). Examples: base_link, torso, head, arm_link.

### Joint
A connection between two links in URDF. Joint types include:
- **Fixed**: No movement
- **Revolute**: Rotation with limits
- **Continuous**: Unlimited rotation
- **Prismatic**: Linear sliding

### XACRO
A macro language for URDF that reduces repetition. XACRO files generate URDF files. Used for complex robots with repeated structures (e.g., left/right arms).

### TF2 (Transform Library)
A ROS 2 library for tracking coordinate frames over time. TF2 maintains the relationships between robot parts and the world.

### Frame
A coordinate system in 3D space. Robots have many frames: `base_link`, `camera_frame`, `world`, etc. TF2 transforms points between frames.

---

## Sensors

### IMU (Inertial Measurement Unit)
A sensor that measures acceleration and angular velocity. Used for orientation, balance, and motion detection. Contains accelerometers and gyroscopes.

### LIDAR (Light Detection and Ranging)
A sensor that measures distances using laser pulses. Produces point clouds or 2D scans. Used for mapping, obstacle detection, and navigation.

### Depth Camera
A camera that captures both color (RGB) and depth information. Examples: Intel RealSense, Microsoft Azure Kinect. Used for 3D perception and object recognition.

### Force/Torque Sensor
Measures forces and torques applied to a surface. Used in robot hands and feet for grip control and balance.

### Encoder
Measures rotation of a motor or wheel. Provides position and velocity feedback for motor control.

---

## Simulation

### Gazebo
A physics simulation environment for robots. Gazebo simulates sensors, actuators, and world physics. ROS 2 integrates tightly with Gazebo.

### RViz2
A 3D visualization tool for ROS 2. Displays sensor data, robot models, paths, and markers. Essential for debugging and monitoring.

### SDF (Simulation Description Format)
A format for describing simulated worlds and objects in Gazebo. More expressive than URDF for simulation-specific features.

---

## Development Tools

### colcon
The build tool for ROS 2 workspaces. `colcon build` compiles packages; `colcon test` runs tests.

### Package
A unit of ROS 2 software. Packages contain nodes, libraries, configuration, and launch files. Packages declare dependencies and can be shared.

### Workspace
A directory containing one or more ROS 2 packages. You source a workspace's setup file to use its packages.

### Launch File
A Python or XML file that starts multiple nodes with configuration. Launch files orchestrate complex robot systems.

### Bag (rosbag2)
A file format for recording and replaying ROS 2 messages. Bags are essential for debugging, testing, and data collection.

---

## Common Abbreviations

| Abbreviation | Full Name |
|--------------|-----------|
| ROS | Robot Operating System |
| DDS | Data Distribution Service |
| QoS | Quality of Service |
| URDF | Unified Robot Description Format |
| TF | Transform (TF2 = Transform 2) |
| IMU | Inertial Measurement Unit |
| LIDAR | Light Detection and Ranging |
| RGB | Red Green Blue (color) |
| SLAM | Simultaneous Localization and Mapping |
| MPC | Model Predictive Control |

---

**Next**: [Introduction](./introduction.md) | [B1: Introduction to ROS 2](./beginner/01-intro-to-ros2.md)
