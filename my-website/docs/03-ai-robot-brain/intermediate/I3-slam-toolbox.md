---
id: i_lesson3_slam_toolbox
title: "SLAM Toolbox Configuration"
sidebar_position: 33
tier: intermediate
chapter: chapter_3_ai_brain
estimated_time: "60-90 minutes"
prerequisites: ["i_lesson2_tf2"]
---

# I3: SLAM Toolbox Configuration

**Building Maps with SLAM in ROS 2**

---

## Learning Objectives

By the end of this lesson, you will be able to:
- Configure and launch SLAM Toolbox
- Generate an occupancy grid map in simulation
- Save and load maps for navigation
- Tune SLAM parameters for different environments

---

## Prerequisites

- Completed [I2: TF2 Coordinate Frames](I2-tf2-coordinate-frames.md)
- SLAM Toolbox package installed
- Gazebo simulation environment ready

---

## Chapter 2 Refresher: Gazebo Simulation

> **Quick Review**: Gazebo provides a simulated environment with physics and sensors.
> Your robot's LIDAR publishes to `/scan` (sensor_msgs/LaserScan).
>
> Key topics for SLAM:
> - `/scan` - LIDAR data
> - `/odom` - Odometry (wheel-based position estimate)
> - `/tf` - Coordinate transforms
>
> See [Gazebo Refresher](../beginner/refresher-gazebo.md) for setup details.

---

## Theory: How SLAM Toolbox Works

### The SLAM Process

![SLAM Process Flow](diagrams/slam-process.svg)

*Alt-text: Flowchart showing SLAM Toolbox process. LIDAR scans enter scan matching, which compares against existing map. Pose graph optimization refines robot trajectory. Loop closure detection corrects drift when revisiting locations. Output is an occupancy grid map.*

### SLAM Toolbox Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| **Mapping (Online)** | Build map while exploring | First exploration |
| **Localization** | Use existing map, no updates | Production navigation |
| **Lifelong** | Continuous map updates | Long-term operation |

### Key Parameters

```yaml
# SLAM Toolbox parameters
slam_toolbox:
  ros__parameters:
    # Solver settings
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI

    # Map parameters
    resolution: 0.05              # 5cm per cell
    map_frame: map
    odom_frame: odom
    base_frame: base_link

    # Scan matching
    max_laser_range: 20.0         # meters
    minimum_travel_distance: 0.5  # Update threshold
    minimum_travel_heading: 0.5   # radians

    # Loop closure
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
```

---

## Code Example: SLAM Launch File

This launch file starts SLAM Toolbox with custom parameters:

```python
#!/usr/bin/env python3
"""
slam_launch.py

Launch file for SLAM Toolbox with Gazebo simulation.
Configures SLAM for indoor mapping with TurtleBot3.

Usage:
  ros2 launch <package_name> slam_launch.py

Dependencies: slam_toolbox, nav2_lifecycle_manager
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package paths
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # SLAM Toolbox parameters
    slam_params = {
        'use_sim_time': use_sim_time,
        'solver_plugin': 'solver_plugins::CeresSolver',
        'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
        'ceres_preconditioner': 'SCHUR_JACOBI',

        # Map settings
        'resolution': 0.05,
        'max_laser_range': 12.0,
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_frame': 'base_footprint',

        # Update thresholds
        'minimum_travel_distance': 0.3,
        'minimum_travel_heading': 0.3,

        # Scan matching
        'scan_topic': '/scan',
        'mode': 'mapping',

        # Loop closure
        'do_loop_closing': True,
        'loop_search_maximum_distance': 3.0,

        # Processing
        'transform_publish_period': 0.02,
        'map_update_interval': 2.0,
    }

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        slam_toolbox_node,
    ])
```

---

## Demonstration: Running SLAM

### Step 1: Launch Simulation

```bash
# Terminal 1: Launch Gazebo with TurtleBot3
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Step 2: Launch SLAM Toolbox

```bash
# Terminal 2: Start SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

Or use the custom launch file:
```bash
ros2 launch <your_package> slam_launch.py
```

### Step 3: Launch RViz2 for Visualization

```bash
# Terminal 3: RViz2 with SLAM display
ros2 launch slam_toolbox rviz.launch.py
```

Configure RViz2:
1. Set **Fixed Frame** to `map`
2. Add **Map** display → topic `/map`
3. Add **LaserScan** display → topic `/scan`
4. Add **TF** display

### Step 4: Drive the Robot to Explore

```bash
# Terminal 4: Keyboard control
ros2 run turtlebot3_teleop teleop_keyboard
```

Drive around the environment to build the map!

### Step 5: Save the Map

```bash
# Terminal 5: Save map when complete
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

This creates:
- `my_map.pgm` - The occupancy grid image
- `my_map.yaml` - Map metadata

---

## Understanding Map Output

### Map YAML File

```yaml
image: my_map.pgm
resolution: 0.050000       # Meters per pixel
origin: [-10.0, -10.0, 0]  # Map origin (x, y, yaw)
negate: 0
occupied_thresh: 0.65      # Above = occupied
free_thresh: 0.196         # Below = free
```

### Map Image (PGM)

| Pixel Value | Meaning |
|-------------|---------|
| 0 (black) | Occupied |
| 254 (white) | Free |
| 205 (gray) | Unknown |

---

## Tuning SLAM Parameters

### For Large Environments

```yaml
# Reduce update frequency
minimum_travel_distance: 0.5
minimum_travel_heading: 0.5
map_update_interval: 5.0
```

### For Small/Cluttered Environments

```yaml
# Higher resolution, more frequent updates
resolution: 0.025
minimum_travel_distance: 0.2
minimum_travel_heading: 0.2
```

### For Reliable Loop Closure

```yaml
# Increase search range
loop_search_maximum_distance: 5.0
loop_search_space_dimension: 100
```

---

## Key Concepts Summary

| Concept | Description |
|---------|-------------|
| **SLAM Toolbox** | ROS 2 package for 2D SLAM |
| **Async Mode** | Non-blocking SLAM updates |
| **Occupancy Grid** | 2D map with occupied/free/unknown cells |
| **Loop Closure** | Corrects drift when revisiting locations |
| **PGM/YAML** | Map file formats |

---

## Hands-On Exercise

### Exercise I3.1: Custom SLAM Parameters

1. Launch SLAM with default parameters
2. Note the map quality and update frequency
3. Modify these parameters and observe the difference:
   - `resolution`: Try 0.025 vs 0.10
   - `max_laser_range`: Try 8.0 vs 20.0
   - `minimum_travel_distance`: Try 0.1 vs 1.0

### Exercise I3.2: Map a Complete Environment

1. Launch SLAM in the TurtleBot3 world
2. Systematically drive through all areas
3. Return to the start (trigger loop closure)
4. Save the map
5. Inspect the PGM file in an image viewer

---

## AI Agent Assisted Prompts

### Prompt 1: SLAM Quality Issues
```
My SLAM-generated map has "double walls" - walls appear twice slightly offset.
What could cause this? How can I fix it through SLAM parameter tuning or
better sensor data?
```

### Prompt 2: Large-Scale Mapping
```
I need to map a large warehouse (50m x 100m). What SLAM Toolbox configuration
would you recommend? Should I use a different approach like multi-session
mapping?
```

### Prompt 3: Dynamic Environments
```
My robot operates in an environment where furniture moves occasionally.
How should I configure SLAM to handle these changes? What about using
lifelong mapping mode?
```

---

## Summary

In this lesson, you learned:

1. **SLAM Toolbox** provides 2D SLAM for ROS 2
2. Key **modes**: mapping, localization, and lifelong
3. How to **configure parameters** for different environments
4. How to **save and load maps** using map_saver
5. **Map formats**: PGM image + YAML metadata

---

## Next Steps

- **Next Lesson**: [I4: Nav2 Basics](I4-nav2-basics.md) - Using your map for navigation
- **Exercises**: Generate maps of different environments
