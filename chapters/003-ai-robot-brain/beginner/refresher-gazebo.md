# Gazebo & Digital Twin Refresher (Chapter 2 Reference)

**Use this refresher when**: Lessons reference simulation concepts from Chapter 2

---

## Quick Reference: Gazebo Simulation

### What is a Digital Twin?
A **digital twin** is a virtual representation of a physical robot that:
- Mirrors the robot's sensors and actuators
- Provides realistic physics simulation
- Enables safe testing before real-world deployment

### Gazebo Architecture

```
┌─────────────────────────────────────────────────┐
│                  Gazebo Server                   │
│  ┌──────────┐  ┌──────────┐  ┌──────────────┐  │
│  │ Physics  │  │ Sensors  │  │   Plugins    │  │
│  │  Engine  │  │Simulation│  │ (ROS 2 etc.) │  │
│  └──────────┘  └──────────┘  └──────────────┘  │
└─────────────────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────┐
│               ROS 2 Interface                    │
│  /cmd_vel  /scan  /camera/image  /odom  /tf     │
└─────────────────────────────────────────────────┘
```

### Key Components

| Component | Purpose |
|-----------|---------|
| World file (.world) | Defines environment, physics, lighting |
| URDF/SDF | Robot model description |
| Plugins | Connect sensors/actuators to ROS 2 |
| ros_gz_bridge | Translates Gazebo↔ROS 2 messages |

---

## Essential Commands

```bash
# Launch Gazebo with ROS 2 integration
ros2 launch gazebo_ros gazebo.launch.py

# Spawn a robot model
ros2 run gazebo_ros spawn_entity.py -entity robot_name -file model.urdf

# List Gazebo topics (bridged to ROS 2)
ros2 topic list | grep -E "scan|image|odom"
```

---

## Simulated Sensors

### Camera
```yaml
# Publishes to: /camera/image_raw (sensor_msgs/Image)
# Provides: RGB images at specified resolution and FPS
```

### Depth Camera
```yaml
# Publishes to: /depth_camera/depth/image_raw (sensor_msgs/Image)
# Provides: Depth images (32-bit float, meters)
```

### LIDAR
```yaml
# Publishes to: /scan (sensor_msgs/LaserScan)
# Provides: 2D range measurements
```

### IMU
```yaml
# Publishes to: /imu (sensor_msgs/Imu)
# Provides: Orientation, angular velocity, linear acceleration
```

---

## Spawning a Robot

```python
# In a launch file
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        'gazebo_ros', 'gazebo.launch.py',
        launch_arguments={'world': 'my_world.world'}.items()
    )

    # Spawn robot
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-topic', 'robot_description']
    )

    return LaunchDescription([gazebo, spawn])
```

---

## Coordinate Frames in Simulation

```
world (fixed)
  └── odom (drifts over time)
        └── base_link (robot body)
              ├── base_scan (LIDAR)
              ├── camera_link (camera)
              └── imu_link (IMU)
```

The `odom` frame provides the robot's estimated position based on wheel odometry, which drifts over time. SLAM corrects this drift.

---

## Need More Detail?

See [Chapter 2: Digital Twin Simulation](../../002-digital-twin/README.md) for complete coverage.
