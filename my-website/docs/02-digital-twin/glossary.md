---
id: chapter_2_glossary
title: "Digital Twin Glossary"
sidebar_position: 13
chapter: chapter_2_digital_twin
---

# Chapter 2 Glossary: Digital Twin & Simulation

> Quick reference for key terms used throughout the Digital Twin chapter.

---

## Core Concepts

### Digital Twin
A synchronized virtual replica of a physical robot that mirrors its state in real-time and enables testing, training, and prediction without risking the real hardware. The digital twin includes:
- Accurate kinematic and dynamic model (URDF)
- Physics simulation (Gazebo)
- Real-time data synchronization (ROS 2 bridge)

**Related**: Simulation, Physics Engine, Bridge Node
**Appears in**: B1, A1, A2

---

### Simulation
A computational model that replicates the behavior of a physical system (robot) without real hardware. In this chapter, we use **Gazebo**, which simulates:
- Joint dynamics and constraints
- Gravity and collisions
- Sensor outputs (IMU, camera, LiDAR)

**Distinct from**: Digital Twin (which adds synchronization layer)
**Related**: Physics Engine, Gazebo, Sensor Simulation
**Appears in**: B1, B2, I1

---

### Physics Engine
The computational component of a simulator that calculates forces, motion, and collisions. Gazebo uses **ODE (Open Dynamics Engine)** by default, which solves:
- Rigid body dynamics
- Joint constraints
- Contact detection and response

**Parameters**: Solver iterations, timestep size, friction coefficients
**Related**: Real-Time Factor, Physics Configuration
**Appears in**: I1, A1

---

## Gazebo Concepts

### Gazebo
An open-source robotics simulation platform that provides:
- 3D physics simulation with visual rendering
- ROS 2 integration via plugins
- Support for URDF/SDF robot descriptions
- Sensor simulation (cameras, LiDAR, IMU)

**Version in this chapter**: Gazebo Classic (11.x), not Ignition Gazebo
**Why Classic**: Mature, extensive ROS 2 documentation, extensive humanoid examples
**Related**: SDF, World File, Gazebo Plugins
**Appears in**: B2, I1, I2, A1, A2

---

### SDF (Simulation Description Format)
An XML-based format for describing simulation worlds in Gazebo. SDF defines:
- Physics parameters (timestep, solver, gravity)
- Static and dynamic models (ground, obstacles, robots)
- Lighting and scene properties
- Plugins for ROS 2 integration

**File extension**: `.world` (though technically SDF v1.6+)
**Structure**: `<sdf version="1.6"><world>...</world></sdf>`
**Related**: URDF, World File
**Appears in**: I1, I2

---

### World File
A Gazebo-specific SDF file that describes a complete simulation environment. Contains:
- Ground plane and obstacles
- Lighting configuration
- Physics settings
- Robot spawn location hints

**Example files**: `humanoid_lab.world`, `simple_lab.world`
**Created in**: I1 lesson
**Related**: SDF, Gazebo
**Appears in**: B2, I1, I2, A1, A2

---

### Gazebo Plugin
A loadable library that extends Gazebo with custom functionality. In this chapter:
- `libgazebo_ros_state.so`: Publishes model states to ROS 2
- `libgazebo_ros2_control.so`: Integrates ros2_control framework
- `libgazebo_ros_camera.so`: Simulates camera sensors
- `libgazebo_ros_imu_sensor.so`: Simulates IMU sensors

**Where defined**: In URDF `<gazebo>` tags or SDF `<plugin>` elements
**Related**: ROS 2, Sensor Simulation
**Appears in**: I1, I2, A2

---

## Robot Description

### URDF (Unified Robot Description Format)
An XML format describing robot structure, shared between simulation and physical hardware:
- Links (rigid bodies) and their geometry
- Joints (connections between links)
- Inertia properties (mass, moment of inertia)
- Collision and visual geometries
- Transmissions and ros2_control configuration

**Defined in**: Chapter 1 (prerequisite)
**Used in**: All tiers (B2 onward reference URDF)
**Related**: Gazebo, Simulation
**Appears in**: B1, I2, A2

---

### XACRO
A macro language for URDF that enables:
- Variable substitution
- Repeated patterns (e.g., left/right leg symmetry)
- Modular includes

**Not deeply covered**: Chapter 2 focuses on Gazebo, not XACRO syntax
**Mentioned in**: I2 (as alternative to direct URDF)
**Related**: URDF
**Appears in**: I2

---

### Inertia Matrix
A 3x3 matrix describing how mass is distributed around a link's center of mass:
```
Ixx  Ixy  Ixz
Ixy  Iyy  Iyz
Ixz  Iyz  Izz
```

**Critical for**: Physics realism; incorrect inertia causes unstable simulation
**Common mistake**: Zero or negative inertia values causing simulation explosion
**Related**: Dynamic Objects, Physics Engine
**Appears in**: I1, I2, A1

---

## Control & Dynamics

### Joint State
The current configuration of a robot's joints published on `/joint_states` topic. Contains:
- Joint names (e.g., `left_knee`)
- Positions (radians)
- Velocities (rad/s)
- Efforts (torques in Nm)

**Message type**: `sensor_msgs/JointState`
**Published by**: Gazebo (simulation), physical robot drivers (hardware)
**Subscribed by**: Controllers, monitoring nodes
**Related**: Joint Trajectory, Actuator
**Appears in**: B2, I2, A1, A2

---

### Joint Trajectory
A sequence of desired joint positions over time, sent as commands to a robot. Contains:
- Joint names
- Trajectory points with positions, velocities, accelerations
- Time-from-start for each point

**Message type**: `trajectory_msgs/JointTrajectory`
**Published by**: Control nodes (like `joint_commander.py`)
**Subscribed by**: `joint_trajectory_controller` (via ros2_control)
**Related**: Joint State, Control Loop
**Appears in**: I2, A1, A2

---

### ros2_control
A ROS 2 framework for managing actuator control. Provides:
- Standardized controller interface
- Hardware abstraction (works with Gazebo and real hardware)
- Joint control at configurable rates (e.g., 1000Hz)

**In simulation**: `gazebo_ros2_control` plugin translates commands to forces
**In hardware**: Drivers implement actual motor commands
**Configuration**: Via URDF `<ros2_control>` tags and YAML controller configs
**Related**: Joint Trajectory, Gazebo Plugin
**Appears in**: I2, A2

---

### Real-Time Factor (RTF)
The ratio of simulation time to wall-clock time:

```
RTF = Simulation Time / Real Time
```

- RTF = 1.0: Simulation runs at real speed
- RTF = 0.5: Simulation runs at half speed
- RTF = 2.0: Simulation runs twice as fast

**Target in chapter**: RTF >= 0.8 (for control stability)
**Measure with**: `gz stats` command
**Affected by**: Physics complexity, visual rendering, compute power
**Related**: Performance Optimization, Headless Mode
**Appears in**: B1, B2, I1, I2

---

## Communication & Synchronization

### ROS 2 Topic
A named channel for publishing and subscribing to messages. In digital twin:
- `/hw/joint_states`: Physical robot state
- `/sim/joint_states`: Simulation state
- `/cmd/joint_trajectory`: Commands (routed to hw or sim)
- `/bridge/latency`: Bridge performance metrics

**Topic Namespace Convention**:
- `/hw/`: Physical hardware
- `/sim/`: Simulation
- `/cmd/`: Commands
- `/bridge/`: Bridge internal

**Related**: Bridge Node, QoS, Latency
**Appears in**: B2, A1, A2

---

### Bridge Node
A ROS 2 node that synchronizes data between physical robot and simulation. The bridge:
- Subscribes to `/hw/joint_states` and `/sim/joint_states`
- Publishes mirrored/forwarded states to the other system
- Monitors latency and enforces safety gates
- Switches between modes (sim, live, mirror)

**Implemented in**: `bridge_node.py` (A2 lesson)
**Related**: Digital Twin, Synchronization, Latency
**Appears in**: A1, A2

---

### Bridge Mode
Operating mode for the bridge node:

**SIM Mode**: Commands route to simulation only (safe testing)
**LIVE Mode**: Commands route to physical hardware (with safety checks)
**MIRROR Mode**: Physical robot state mirrors to simulation (visualization)

**Controlled via**: ROS 2 parameters or service calls
**Default**: `mirror`
**Related**: Bridge Node, Safety Gate
**Appears in**: A1, A2

---

### Latency
The delay between when data is generated (e.g., sensor reading) and when it arrives at the destination. Measured in milliseconds.

**Sources of latency**:
- Sensor acquisition (5ms)
- Network transmission (10-20ms)
- Processing (1-5ms)
- Message serialization (1ms)

**Threshold in chapter**: 50ms (acceptable for 20Hz control loops)
**Monitored by**: `latency_monitor.py`
**Critical for**: Real-time control, safety
**Related**: Bridge Node, Network, QoS
**Appears in**: A1, A2

---

### Synchronization Pattern
A design pattern for bidirectional data flow between physical and virtual systems. Common patterns:

**State Mirror**: Physical state replicated to simulation (for visualization)
**Command Forward**: Simulation commands forwarded to hardware (for testing)
**Bidirectional Sync**: Full synchronization with conflict resolution (for digital twin)

**Described in**: A1 lesson
**Related**: Bridge Node, Data Flow
**Appears in**: A1, A2

---

### QoS (Quality of Service)
Configuration for how ROS 2 messages are delivered:
- **Reliability**: RELIABLE (guaranteed delivery) vs BEST_EFFORT (speed preferred)
- **History**: KEEP_LAST (latest N messages) vs KEEP_ALL
- **Durability**: VOLATILE (transient) vs TRANSIENT_LOCAL (persists)

**In digital twin**:
- Commands: RELIABLE (ensure delivery)
- Sensors: BEST_EFFORT (speed over guarantees)

**Related**: Topic, Network, Latency
**Appears in**: A1, A2

---

## Sensors & Perception

### Sensor Simulation
Virtual representation of robot sensors in Gazebo. Common sensors:
- **IMU** (Inertial Measurement Unit): Acceleration and angular velocity
- **Camera**: RGB images (raw rendering)
- **LiDAR**: Distance measurements (simulated ray-casting)
- **Force/Torque**: Joint efforts and reaction forces

**Simulated via**: Gazebo sensor plugins + ray tracing
**Accuracy**: Depends on physics fidelity and rendering quality
**Used for**: Algorithm testing, AI training
**Related**: Physics Engine, Gazebo Plugin
**Appears in**: B1, A2

---

### IMU (Inertial Measurement Unit)
A sensor providing:
- Linear acceleration (x, y, z) in m/s²
- Angular velocity (roll, pitch, yaw) in rad/s
- Orientation (quaternion)

**Useful for**: Balance control, motion estimation
**Simulated in**: Gazebo via `gazebo_ros_imu_sensor` plugin
**Published to**: `/imu/data` or `/sim/imu/data`
**Related**: Sensor Simulation, Bridge
**Appears in**: A1, A2

---

## Troubleshooting & Monitoring

### Headless Mode
Running Gazebo without graphics rendering to improve performance:

```bash
gzserver world.world &
```

**Benefits**: Higher RTF, suitable for cloud/CI/CD
**Tradeoff**: No visual feedback; use for data collection only
**Related**: Real-Time Factor, Performance
**Appears in**: B2, README.md

---

### Model Explosion
Physics instability where a model violently deforms or flies apart, usually caused by:
- Invalid inertia matrix (negative or zero values)
- Timestep too large
- Solver iterations too low
- Extreme initial conditions

**Fix**: Check inertia, reduce timestep, increase solver iterations
**Related**: Physics Engine, Inertia Matrix
**Appears in**: I1, I2

---

### Clock Skew
Mismatch between simulation time and wall-clock time, or between physical and virtual system clocks.

**Causes**: Different timestep rates, network delays, hardware timing variations
**Effects**: Synchronization errors, command rejection
**Handled by**: Bridge node clock synchronization logic
**Related**: Latency, Synchronization
**Appears in**: A1

---

### Watchdog
A monitoring mechanism that detects connection loss or timeouts. In this chapter:
- Monitors `/hw/joint_states` topic (physical robot)
- Monitors `/sim/joint_states` topic (simulation)
- Flags connection loss after 2 seconds of silence

**Implemented in**: `bridge_node.py`
**Action on timeout**: Log warning, disable live commands, fall back to simulation-only
**Related**: Bridge Node, Safety, Connection Management
**Appears in**: A2

---

## Advanced Topics

### Sim-to-Real Transfer
The process of transferring a policy (e.g., neural network) trained in simulation to a real robot. Challenges:
- **Domain gap**: Simulation doesn't perfectly match reality
- **Sensor noise**: Real sensors are noisier than simulated
- **Physics differences**: Friction, inertia may vary
- **Timing variations**: Real hardware has jitter

**Mitigation strategies**: Domain randomization, action smoothing, sim-to-real validation
**Related**: AI Training, Reinforcement Learning
**Appears in**: A2 (optional section)

---

### Domain Randomization
Training technique that randomizes simulation parameters to improve robustness:
- Physics: Friction, mass, damping
- Sensors: Noise, delay, dropout
- Dynamics: Joint play, actuator limits

**Purpose**: Make policies robust to real-world variations
**Used in**: Reinforcement learning training
**Related**: Sim-to-Real Transfer, AI Training
**Appears in**: A2

---

### Reinforcement Learning (RL)
Machine learning approach where an agent learns by interacting with an environment (simulation). The agent:
- Observes state (joint positions, sensor readings)
- Chooses action (joint target positions)
- Receives reward (e.g., forward progress, energy efficiency)
- Learns policy through trial and error

**Frameworks**: Stable Baselines 3, OpenAI Gym, RLlib
**Integration**: Via Gym environment wrapper around Gazebo
**Related**: AI Training, Sim-to-Real Transfer
**Appears in**: A2 (optional section)

---

### Streaming
Continuous transmission of sensor data to external systems (e.g., ML training backend). In this chapter:
- Collects joint states, IMU, camera data
- Batches samples into 32-sample groups
- Streams to HTTP endpoint at configurable rate

**Implemented in**: `sensor_streamer.py`
**Purpose**: Real-time data collection for distributed training
**Related**: AI Training, Data Pipeline
**Appears in**: A2

---

## Measurement & Performance

### Throughput
The amount of data or number of operations processed per unit time.

**In digital twin**:
- Sensor throughput: 100+ Hz for joint states
- Command throughput: 100+ Hz for control
- Network throughput: Bandwidth consumed by bridging

**Target**: > 1000 environment steps/second for efficient training
**Related**: Performance, Latency, RTF
**Appears in**: A2

---

### Violation (Threshold Violation)
When a metric exceeds a safety threshold. In this chapter:
- **Latency violation**: Current latency > 50ms threshold
- **RTF violation**: RTF &lt; 0.8 (physics may become unstable)

**Tracking**: Bridge node counts and logs violations
**Action**: Warn operator, may disable live control
**Related**: Safety, Monitoring, Latency
**Appears in**: A1, A2

---

## Summary Table

| Term | Appears In | Key Point |
|------|-----------|-----------|
| Digital Twin | B1, A1, A2 | Virtual replica synchronized with physical |
| Gazebo | B2-A2 | Physics simulation platform |
| SDF/World File | I1, I2 | XML description of simulation environment |
| URDF | B1, I2, A2 | Robot description format (shared with Chapter 1) |
| RTF | B1, B2, I1 | Simulation speed indicator (target &gt;= 0.8) |
| Bridge Node | A1, A2 | Component synchronizing physical and virtual |
| Latency | A1, A2 | Communication delay (target &lt;= 50ms) |
| ros2_control | I2, A2 | Standardized control framework |
| QoS | A1, A2 | Message delivery configuration |
| Watchdog | A2 | Connection monitoring mechanism |

---

## Cross-References

**Glossaries**:
- [Chapter 1 Glossary](../01-ros2-nervous-system/glossary.md) (URDF, ROS 2, topics)
- [Book Glossary](../../glossary.md) (if available)

**Appendices**:
- [Chapter 2 Troubleshooting](README.md#troubleshooting)
- [Quick Reference](README.md#quick-reference)

---

| Previous | Up | Next |
|----------|-----|------|
| [Chapter 1 Glossary](../01-ros2-nervous-system/glossary.md) | [Chapter 2 Overview](README.md) | [B1: Digital Twin Concepts](beginner/B1-digital-twin-concepts.md) |
