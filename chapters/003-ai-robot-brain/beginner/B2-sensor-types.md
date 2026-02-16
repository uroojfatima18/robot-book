# B2: Understanding Sensor Types

**The Eyes and Ears of Your Robot**

---

## Learning Objectives

By the end of this lesson, you will be able to:
- Identify the three main sensor categories for robotic perception
- Explain how RGB cameras, depth cameras, and LIDAR work
- Compare sensor trade-offs for different applications
- Choose appropriate sensors for specific robotics tasks

---

## Prerequisites

- Completed [B1: Introduction to Robotic Perception](B1-introduction-perception.md)
- Basic understanding of ROS 2 topics ([ROS 2 Refresher](refresher-ros2.md))

---

## Chapter 1 Refresher: ROS 2 Topics

> **Quick Review**: Sensors publish data to **topics** using **messages**. Subscribers receive this data for processing.
>
> Common sensor message types:
> - `sensor_msgs/Image` - Camera images
> - `sensor_msgs/LaserScan` - 2D LIDAR scans
> - `sensor_msgs/PointCloud2` - 3D point clouds
>
> See [ROS 2 Refresher](refresher-ros2.md) for more details.

---

## Theory: Sensor Categories

Robots perceive the world through three main sensor categories:

![Sensor Comparison](diagrams/sensor-comparison.svg)

*Alt-text: Comparison diagram showing three sensor types side by side. RGB Camera (left) captures color images with high resolution but no depth. Depth Camera (center) provides per-pixel distance measurements. LIDAR (right) gives accurate 3D point measurements with 360-degree coverage.*

### 1. RGB Cameras

**How they work**: Capture light intensity through a lens onto an image sensor (CCD or CMOS).

| Characteristic | Value |
|---------------|-------|
| **Output** | Color image (RGB pixels) |
| **ROS 2 Message** | `sensor_msgs/Image` |
| **Typical Resolution** | 640x480 to 4K |
| **Frame Rate** | 30-60 fps |
| **Range** | Limited by focus/lighting |
| **Cost** | Low ($10-$100) |

**Strengths**:
- Rich color and texture information
- High resolution
- Inexpensive
- Essential for object recognition

**Weaknesses**:
- No direct depth information
- Affected by lighting conditions
- Shadows cause problems

**Use Cases**: Object recognition, visual SLAM, lane detection, human-robot interaction

### 2. Depth Cameras

**Three main technologies**:

#### Stereo Cameras
Match features between two cameras to compute depth (like human vision).

#### Time-of-Flight (ToF)
Measure how long light takes to bounce back from objects.

#### Structured Light
Project a known pattern and analyze its deformation.

| Characteristic | Stereo | ToF | Structured Light |
|---------------|--------|-----|------------------|
| **Output** | Depth image | Depth image | Depth image |
| **Range** | 0.5-10m | 0.2-5m | 0.2-4m |
| **Resolution** | High | Medium | Medium |
| **Outdoor Use** | Good | Poor | Poor |
| **Cost** | Medium | Medium | Low-Medium |
| **Example** | ZED 2 | Azure Kinect | RealSense D435 |

**Common ROS 2 Messages**:
- `sensor_msgs/Image` (depth image, 16-bit or 32-bit float)
- `sensor_msgs/PointCloud2` (3D point cloud)

**Strengths**:
- Direct depth measurement
- Dense depth maps
- Good for manipulation and obstacle detection

**Weaknesses**:
- Limited range
- May struggle with transparent/reflective surfaces
- Indoor vs outdoor limitations

### 3. LIDAR (Light Detection and Ranging)

**How it works**: Rotating laser measures distance by time-of-flight, creating a 360° scan.

| Characteristic | 2D LIDAR | 3D LIDAR |
|---------------|----------|----------|
| **Output** | Planar scan | Point cloud |
| **ROS 2 Message** | `sensor_msgs/LaserScan` | `sensor_msgs/PointCloud2` |
| **Range** | 10-30m | 100-200m |
| **Accuracy** | ±2-3cm | ±2-3cm |
| **Points/sec** | 10k-40k | 100k-2M |
| **Cost** | $100-$500 | $1k-$75k |

**Strengths**:
- Very accurate distance measurements
- Works in any lighting
- Long range
- Essential for autonomous vehicles

**Weaknesses**:
- No color information
- Expensive (especially 3D)
- Moving parts can fail
- Sparse data compared to cameras

---

## Sensor Comparison Table

| Factor | RGB Camera | Depth Camera | LIDAR |
|--------|-----------|--------------|-------|
| **Color** | Yes | No | No |
| **Depth** | No* | Yes | Yes |
| **Range** | Variable | Short | Long |
| **Accuracy** | N/A | ±1-3cm | ±2-3cm |
| **Lighting** | Required | Indoor | Any |
| **Cost** | Low | Medium | High |
| **Data Rate** | High | Medium | Medium |
| **Best For** | Recognition | Manipulation | Navigation |

*Depth can be estimated from monocular images using AI, but not measured directly.

---

## Choosing Sensors for Your Application

### Indoor Navigation Robot
- **Primary**: 2D LIDAR for SLAM and obstacle avoidance
- **Secondary**: Depth camera for close obstacles
- **Optional**: RGB camera for visual features

### Manipulation Robot (Pick and Place)
- **Primary**: Depth camera for object localization
- **Secondary**: RGB camera for object recognition
- **Gripper**: Force/torque sensors

### Outdoor Autonomous Vehicle
- **Primary**: 3D LIDAR for navigation
- **Secondary**: Cameras for lane/sign detection
- **Additional**: Radar for all-weather operation

### Home Service Robot
- **Primary**: RGBD camera (combined RGB + depth)
- **Secondary**: 2D LIDAR for navigation
- **Optional**: Microphone array for voice commands

---

## Demonstration: Viewing Sensor Data Types

### Compare LaserScan vs PointCloud vs Image

```bash
# Terminal 1: Launch simulation
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: View LIDAR data structure
ros2 topic echo /scan --once | head -30

# Terminal 3: View camera info
ros2 topic info /camera/image_raw

# Terminal 4: Launch RViz2
ros2 launch turtlebot3_bringup rviz2.launch.py
```

In RViz2, add displays for:
- **LaserScan** (/scan) - See 2D distance ring
- **Image** (/camera/image_raw) - See camera view
- **PointCloud2** (if available) - See 3D points

---

## Hands-On Exercise

### Exercise B2.1: Sensor Message Analysis

With the simulation running, analyze each sensor type:

1. **LaserScan Analysis**:
   ```bash
   ros2 topic echo /scan --once
   ```
   Answer:
   - What is the `angle_increment` value?
   - How many ranges are in the array?
   - Calculate the field of view (angle_max - angle_min in degrees)

2. **Camera Analysis**:
   ```bash
   ros2 topic info /camera/image_raw --verbose
   ```
   Answer:
   - What is the image encoding?
   - What is the resolution (width x height)?
   - What is the publish rate?

### Exercise B2.2: Sensor Selection Challenge

For each scenario, choose the best sensor combination and justify:

| Scenario | Recommended Sensors | Why? |
|----------|-------------------|------|
| Warehouse robot avoiding workers | ? | ? |
| Drone mapping a building exterior | ? | ? |
| Robot arm sorting colored blocks | ? | ? |

<details>
<summary>Click for suggested answers</summary>

1. **Warehouse robot**: 2D LIDAR (primary) + RGB camera (person detection). LIDAR for reliable obstacle detection, camera for recognizing humans vs objects.

2. **Drone mapping**: 3D LIDAR or stereo camera. Needs outdoor operation and depth for 3D reconstruction.

3. **Sorting colored blocks**: RGBD camera. Needs both color (for sorting) and depth (for picking).

</details>

---

## AI Agent Assisted Prompts

### Prompt 1: Sensor Fusion Strategy
```
I have a mobile robot with a 2D LIDAR and an RGB camera. The LIDAR gives
accurate distances but no color. The camera gives color but no distance.
How can I combine these sensors to identify and locate objects by color?
What are the calibration steps needed?
```

### Prompt 2: Sensor Failure Handling
```
My robot's LIDAR can fail in dusty environments. What backup perception
strategies could I implement using only cameras? What are the limitations?
```

### Prompt 3: Cost-Effective Sensor Suite
```
I'm building a hobby mobile robot on a $200 sensor budget. What combination
of sensors would you recommend for indoor navigation and simple object
detection? Explain the trade-offs of your choices.
```

---

## Key Concepts Summary

| Sensor | Measures | Best For |
|--------|----------|----------|
| **RGB Camera** | Light intensity (color) | Recognition, tracking |
| **Depth Camera** | Per-pixel distance | Manipulation, close obstacles |
| **LIDAR** | Accurate 3D distances | Navigation, mapping |

---

## Summary

In this lesson, you learned:

1. **Three sensor categories**: RGB cameras, depth cameras, and LIDAR
2. **Trade-offs** between resolution, range, accuracy, and cost
3. **Depth camera technologies**: Stereo, ToF, and structured light
4. **Selection criteria** based on application requirements
5. **ROS 2 message types** for each sensor

---

## Next Steps

- **Next Lesson**: [B3: SLAM and Navigation Concepts](B3-slam-navigation-intro.md) - How robots map and navigate
- **Exercises**: Complete [Beginner Exercises](../exercises/beginner-exercises.md)
