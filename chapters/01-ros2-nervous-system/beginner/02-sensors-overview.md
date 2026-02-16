---
id: b_lesson2_sensors
title: "Basic Sensors Overview"
tier: beginner
chapter: chapter_1_ros2
estimated_time: "1 hour"
prerequisites: ["b_lesson1_intro"]
---

# Basic Sensors Overview

## Learning Objectives

By the end of this lesson, you will be able to:

- **Identify** the four primary sensor types used in humanoid robots
- **Explain** the purpose and output of IMU, LIDAR, cameras, and force sensors
- **Describe** how each sensor contributes to robot perception
- **Recognize** ROS 2 message types for different sensor data

## Introduction

If ROS 2 is the nervous system of a robot, sensors are its eyes, ears, and touch. Without sensors, a robot would be blind to its environment—unable to see obstacles, sense orientation, or feel contact with objects.

In this lesson, we'll explore the four fundamental sensor types that give humanoid robots their perception capabilities. Understanding these sensors is essential before you build nodes that process their data.

Just as humans combine sight, hearing, touch, and balance to navigate the world, robots fuse data from multiple sensors to understand their environment.

---

## Sensor Types for Humanoid Robots

### Theory

A typical humanoid robot uses a combination of sensors:

```
┌─────────────────────────────────────────────────────────────┐
│                    HUMANOID ROBOT SENSORS                    │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│    👁️ CAMERAS (HEAD)                                         │
│    ├── RGB: Color images for recognition                    │
│    └── Depth: 3D perception, distance measurement          │
│                                                              │
│    📡 LIDAR (HEAD/CHEST)                                    │
│    └── 360° distance scanning, obstacle detection           │
│                                                              │
│    🧭 IMU (TORSO)                                            │
│    ├── Accelerometer: Linear acceleration                   │
│    ├── Gyroscope: Angular velocity                          │
│    └── Magnetometer: Orientation (compass)                  │
│                                                              │
│    ✋ FORCE SENSORS (HANDS/FEET)                             │
│    ├── Hands: Grip force, contact detection                 │
│    └── Feet: Balance, ground reaction forces                │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

Each sensor type provides unique information that, when combined, gives the robot comprehensive environmental awareness.

---

## IMU: Orientation and Motion

### Theory

An **IMU (Inertial Measurement Unit)** measures movement and orientation without external references. It combines:

| Component | Measures | Units | Use Case |
|-----------|----------|-------|----------|
| Accelerometer | Linear acceleration | m/s² | Detecting falls, impacts |
| Gyroscope | Angular velocity | rad/s | Rotation, turning |
| Magnetometer | Magnetic field | μT | Compass heading |

**Why IMU matters for humanoids:**
- **Balance control**: Detect when the robot is tilting
- **Fall detection**: Sense sudden accelerations indicating a fall
- **Motion estimation**: Track movement between other sensor updates
- **Head stabilization**: Keep cameras steady during walking

### IMU in ROS 2

IMU data uses the `sensor_msgs/msg/Imu` message type:

```
# sensor_msgs/msg/Imu

Header header                    # Timestamp and frame

geometry_msgs/Quaternion orientation           # Orientation estimate
float64[9] orientation_covariance              # Uncertainty

geometry_msgs/Vector3 angular_velocity         # Gyroscope data (rad/s)
float64[9] angular_velocity_covariance

geometry_msgs/Vector3 linear_acceleration      # Accelerometer data (m/s²)
float64[9] linear_acceleration_covariance
```

### Code Example: IMU Message Structure

```python
# Example IMU data structure
# Published to topic: /imu/data

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

# Example IMU reading
imu_msg = Imu()
imu_msg.header.stamp = node.get_clock().now().to_msg()
imu_msg.header.frame_id = "imu_link"

# Orientation (quaternion: x, y, z, w)
imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

# Angular velocity (rad/s)
imu_msg.angular_velocity = Vector3(x=0.01, y=-0.02, z=0.005)

# Linear acceleration (m/s²) - includes gravity!
imu_msg.linear_acceleration = Vector3(x=0.1, y=0.05, z=9.81)
```

---

## LIDAR: Distance and Mapping

### Theory

**LIDAR (Light Detection and Ranging)** uses laser pulses to measure distances. A LIDAR sensor:

1. Emits laser pulses in multiple directions
2. Measures time for each pulse to return
3. Calculates distance based on speed of light
4. Produces a "point cloud" or "scan" of the environment

**Types of LIDAR:**

| Type | Output | Range | Use Case |
|------|--------|-------|----------|
| 2D LIDAR | Single plane scan | 10-30m | Navigation, obstacle avoidance |
| 3D LIDAR | Full point cloud | 50-200m | Mapping, outdoor navigation |

**Why LIDAR matters for humanoids:**
- **Obstacle detection**: See walls, furniture, people
- **SLAM**: Simultaneous Localization and Mapping
- **Navigation**: Plan paths through environments
- **Safety**: Emergency stop when obstacles are too close

### LIDAR in ROS 2

2D LIDAR uses `sensor_msgs/msg/LaserScan`:

```
# sensor_msgs/msg/LaserScan

Header header                # Timestamp and frame

float32 angle_min            # Start angle (rad)
float32 angle_max            # End angle (rad)
float32 angle_increment      # Angle between readings (rad)

float32 time_increment       # Time between readings (s)
float32 scan_time            # Total scan time (s)

float32 range_min            # Minimum valid range (m)
float32 range_max            # Maximum valid range (m)

float32[] ranges             # Distance measurements (m)
float32[] intensities        # Signal strength (optional)
```

### Code Example: LaserScan Interpretation

```python
# Example: Interpreting LIDAR data
# Subscribed from topic: /scan

from sensor_msgs.msg import LaserScan
import math

def scan_callback(msg: LaserScan):
    """Process incoming LIDAR scan."""

    # Number of readings in this scan
    num_readings = len(msg.ranges)

    # Find the closest obstacle
    min_distance = float('inf')
    min_angle = 0.0

    for i, distance in enumerate(msg.ranges):
        # Skip invalid readings
        if distance < msg.range_min or distance > msg.range_max:
            continue

        if distance < min_distance:
            min_distance = distance
            # Calculate angle of this reading
            min_angle = msg.angle_min + i * msg.angle_increment

    # Convert angle to degrees for display
    angle_degrees = math.degrees(min_angle)

    print(f"Closest obstacle: {min_distance:.2f}m at {angle_degrees:.1f}°")
```

---

## Cameras: RGB and Depth

### Theory

Cameras provide the richest sensory data for robots. Modern humanoid robots typically use:

| Camera Type | Output | Resolution | Use Case |
|-------------|--------|------------|----------|
| RGB Camera | Color image | 1080p-4K | Object recognition, face detection |
| Depth Camera | Distance per pixel | 640x480 | 3D perception, grasping |
| Stereo Camera | Dual images | Varies | Depth estimation, navigation |

**Popular depth cameras:**
- Intel RealSense D435/D455
- Microsoft Azure Kinect
- Stereolabs ZED

**Why cameras matter for humanoids:**
- **Object recognition**: Identify what's in the environment
- **Face/gesture recognition**: Human-robot interaction
- **Visual servoing**: Guide arm movements to targets
- **SLAM**: Visual landmarks for localization

### Cameras in ROS 2

RGB images use `sensor_msgs/msg/Image`:

```
# sensor_msgs/msg/Image

Header header              # Timestamp and frame
uint32 height              # Image height (pixels)
uint32 width               # Image width (pixels)
string encoding            # Pixel format (rgb8, bgr8, mono8, etc.)
uint8 is_bigendian         # Endianness
uint32 step                # Row length in bytes
uint8[] data               # Actual pixel data
```

Depth images use the same message type with encoding like `16UC1` (16-bit unsigned, 1 channel) where pixel values represent millimeters.

### Code Example: Camera Image Info

```python
# Example: Processing camera image metadata
# Subscribed from topic: /camera/color/image_raw

from sensor_msgs.msg import Image

def image_callback(msg: Image):
    """Process incoming camera image."""

    print(f"Image received:")
    print(f"  Size: {msg.width} x {msg.height}")
    print(f"  Encoding: {msg.encoding}")
    print(f"  Data size: {len(msg.data)} bytes")

    # Common encodings:
    # - rgb8: 3 bytes per pixel (Red, Green, Blue)
    # - bgr8: 3 bytes per pixel (Blue, Green, Red) - OpenCV format
    # - mono8: 1 byte per pixel (grayscale)
    # - 16UC1: 2 bytes per pixel (depth in mm)

    if msg.encoding == 'rgb8':
        bytes_per_pixel = 3
        expected_size = msg.width * msg.height * bytes_per_pixel
        print(f"  Expected size: {expected_size} bytes")
```

---

## Force Sensors: Touch and Pressure

### Theory

**Force/Torque sensors** measure physical contact forces. In humanoid robots, they're critical for:

| Location | Purpose | Measurements |
|----------|---------|--------------|
| Hands/Grippers | Grip control | Force in x, y, z; Torque in x, y, z |
| Feet | Balance | Ground reaction force, center of pressure |
| Joints | Safety | Joint torques, collision detection |

**Why force sensors matter for humanoids:**
- **Grasping**: Apply appropriate grip force (don't crush objects!)
- **Balance**: Detect weight distribution, adjust posture
- **Safety**: Feel unexpected collisions, stop movement
- **Manipulation**: Feel when object is picked up or placed

### Force Sensors in ROS 2

Force/Torque data uses `geometry_msgs/msg/WrenchStamped`:

```
# geometry_msgs/msg/WrenchStamped

Header header              # Timestamp and frame

Wrench wrench:
  Vector3 force            # Force in x, y, z (Newtons)
  Vector3 torque           # Torque in x, y, z (Newton-meters)
```

### Code Example: Force Sensor Reading

```python
# Example: Monitoring grip force
# Subscribed from topic: /gripper/force

from geometry_msgs.msg import WrenchStamped
import math

# Grip force thresholds
MIN_GRIP_FORCE = 1.0   # N - minimum to hold object
MAX_GRIP_FORCE = 50.0  # N - maximum safe force

def force_callback(msg: WrenchStamped):
    """Monitor gripper force for safe grasping."""

    # Calculate total grip force magnitude
    fx = msg.wrench.force.x
    fy = msg.wrench.force.y
    fz = msg.wrench.force.z

    total_force = math.sqrt(fx**2 + fy**2 + fz**2)

    # Check force limits
    if total_force < MIN_GRIP_FORCE:
        print(f"WARNING: Grip too weak ({total_force:.1f}N) - object may slip!")
    elif total_force > MAX_GRIP_FORCE:
        print(f"WARNING: Grip too strong ({total_force:.1f}N) - may damage object!")
    else:
        print(f"Grip force OK: {total_force:.1f}N")
```

---

## ROS 2 Sensor Message Types Summary

### Common Message Packages

| Package | Contains | Example Messages |
|---------|----------|------------------|
| `std_msgs` | Basic types | String, Int32, Float64, Bool |
| `sensor_msgs` | Sensor data | Image, Imu, LaserScan, PointCloud2 |
| `geometry_msgs` | Geometric primitives | Pose, Twist, Vector3, Wrench |
| `nav_msgs` | Navigation | Odometry, Path, OccupancyGrid |

### Code Example: Examining Message Types

```bash
# List all sensor messages
ros2 interface list | grep sensor_msgs

# Show message definition
ros2 interface show sensor_msgs/msg/Imu

# Show message in a running system
ros2 topic echo /imu/data --once
```

---

## Diagrams

![Humanoid robot with sensor locations marked](../diagrams/humanoid-sensor-placement.svg)
*Figure 1: Humanoid Sensor Placement - Typical locations for cameras, LIDAR, IMU, and force sensors on a humanoid robot.*

---

## Hardware Notes

> **Simulation vs. Real Hardware**
>
> All sensors can be simulated in Gazebo with realistic noise models:
>
> | Sensor | Simulation Fidelity | Real-World Considerations |
> |--------|--------------------|-----------------------------|
> | IMU | High | Calibration drift, vibration noise |
> | LIDAR | High | Reflective surfaces, rain/fog |
> | Camera | Medium | Lighting conditions, motion blur |
> | Force | Medium | Sensor placement, cable routing |
>
> When moving to real robots:
> - **Calibrate** sensors before use
> - **Filter** noisy data appropriately
> - **Fuse** multiple sensors for reliability

---

## Summary

In this lesson, you learned:

- ✅ **IMU** measures orientation and motion using accelerometers, gyroscopes, and magnetometers
- ✅ **LIDAR** measures distances using laser pulses, essential for navigation and obstacle avoidance
- ✅ **Cameras** provide visual information in RGB and depth, enabling recognition and 3D perception
- ✅ **Force sensors** measure physical contact, critical for grasping and balance
- ✅ **ROS 2 message types** standardize sensor data for easy integration

---

## AI-Assisted Learning

<details>
<summary>Ask the AI Assistant</summary>

### Conceptual Questions
- "What's the difference between a 2D and 3D LIDAR? When would I use each?"
- "How does an IMU help a humanoid robot maintain balance while walking?"

### Debugging Help
- "My IMU data shows the robot is upside-down even though it's right-side-up. What could cause this?"
- "The depth camera returns all zeros. How do I troubleshoot this?"

### Extension Ideas
- "How would I fuse IMU and camera data to get better pose estimation?"
- "What sensors would I need to add for a robot to navigate outdoors?"

</details>

---

## Exercises

### Exercise 1: Sensor Identification (Easy)

**Description**: Given a robot task, identify which sensors would be most useful.

**Tasks**:
Match each task to the primary sensor:
1. Detect a door in front of the robot → ?
2. Know if the robot is tilting → ?
3. Pick up an egg without breaking it → ?
4. Recognize a person's face → ?

**Acceptance Criteria**: You can explain why each sensor is the best choice for each task.

<details>
<summary>Solution</summary>

1. Door detection → LIDAR (distance to obstacle) or Camera (visual recognition)
2. Tilting detection → IMU (orientation from gyroscope/accelerometer)
3. Egg grasping → Force sensor (measure and control grip force)
4. Face recognition → Camera (RGB image for visual processing)

</details>

### Exercise 2: Message Exploration (Medium)

**Description**: Use ROS 2 CLI to explore sensor message types.

**Tasks**:
1. Run `ros2 interface show sensor_msgs/msg/Imu` and identify the three main data fields
2. Run `ros2 interface show sensor_msgs/msg/LaserScan` and explain what `ranges` contains
3. Find the message type used for 3D point clouds

**Acceptance Criteria**: You can describe the structure of IMU and LaserScan messages.

---

## Navigation

| Previous | Up | Next |
|----------|-----|------|
| [B1: Introduction to ROS 2](./01-intro-to-ros2.md) | [Chapter 1 Home](../README.md) | [I1: Nodes, Topics, Services, and Actions](../intermediate/01-nodes-topics.md) |

---

## Next Steps

Continue to [I1: Nodes, Topics, Services, and Actions](../intermediate/01-nodes-topics.md) to start writing your own ROS 2 Python nodes.
