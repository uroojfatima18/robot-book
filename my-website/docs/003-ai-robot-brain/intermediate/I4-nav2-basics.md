---
id: i_lesson4_nav2
title: "Nav2 Basics"
sidebar_position: 34
tier: intermediate
chapter: chapter_3_ai_brain
estimated_time: "60-90 minutes"
prerequisites: ["i_lesson3_slam_toolbox"]
---

# I4: Nav2 Basics

**Autonomous Navigation with the ROS 2 Navigation Stack**

---

## Learning Objectives

By the end of this lesson, you will be able to:
- Launch Nav2 with a pre-built map
- Send navigation goals programmatically
- Monitor navigation status and handle results
- Configure basic Nav2 parameters

---

## Prerequisites

- Completed [I3: SLAM Toolbox Configuration](I3-slam-toolbox.md)
- A saved map from SLAM (PGM + YAML files)
- Nav2 packages installed

---

## Theory: Nav2 Architecture

### Core Components

| Component | Package | Purpose |
|-----------|---------|---------|
| **Map Server** | nav2_map_server | Loads and publishes the map |
| **AMCL** | nav2_amcl | Localizes robot on map |
| **Planner Server** | nav2_planner | Computes global paths |
| **Controller Server** | nav2_controller | Follows paths (local planning) |
| **Behavior Server** | nav2_behaviors | Recovery behaviors |
| **BT Navigator** | nav2_bt_navigator | Coordinates via behavior tree |

### Navigation Flow

```
Goal Received → Global Planner → Path → Controller → cmd_vel → Robot Moves
                     ↑                      ↓
                Costmap          Obstacle Detected?
                                        ↓
                               Recovery Behaviors
```

### Key Topics

| Topic | Message Type | Direction |
|-------|--------------|-----------|
| `/goal_pose` | geometry_msgs/PoseStamped | Input |
| `/plan` | nav_msgs/Path | Output |
| `/cmd_vel` | geometry_msgs/Twist | Output |
| `/map` | nav_msgs/OccupancyGrid | Input |
| `/scan` | sensor_msgs/LaserScan | Input |

---

## Code Example: Navigation Goal Sender

This node sends navigation goals and monitors progress:

```python
#!/usr/bin/env python3
"""
nav2_goal_sender.py

Sends navigation goals to Nav2 and monitors progress.
Demonstrates Nav2 action client usage.

ROS 2 Actions:
  Client: /navigate_to_pose (nav2_msgs/NavigateToPose)

Dependencies: rclpy, nav2_msgs, geometry_msgs, rclpy.action

Usage:
  ros2 run <package_name> nav2_goal_sender
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration


class Nav2GoalSender(Node):
    """ROS 2 node that sends navigation goals to Nav2."""

    def __init__(self):
        super().__init__('nav2_goal_sender')

        # Nav2 action client
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server available!')

    def send_goal(self, x: float, y: float, yaw: float = 0.0):
        """Send a navigation goal to Nav2.

        Args:
            x: Target X position in map frame (meters)
            y: Target Y position in map frame (meters)
            yaw: Target orientation (radians)
        """
        goal_msg = NavigateToPose.Goal()

        # Create target pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        import math
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)

        self.get_logger().info(f'Sending goal: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')

        # Send goal with callbacks
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            return

        self.get_logger().info('Goal accepted!')

        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose.position

        # Log progress periodically
        self.get_logger().info(
            f'Current position: ({current_pose.x:.2f}, {current_pose.y:.2f})'
        )

    def result_callback(self, future):
        """Handle navigation result."""
        result = future.result().result

        self.get_logger().info('Navigation completed!')

        # Optionally send another goal or shutdown
        # self.send_goal(0.0, 0.0)  # Return home


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalSender()

    # Send example goal
    node.send_goal(x=2.0, y=1.0, yaw=0.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Code Example: Navigation Launch File

```python
#!/usr/bin/env python3
"""
navigation_launch.py

Launch file for Nav2 navigation stack.
Loads map and starts all navigation components.

Usage:
  ros2 launch <package_name> navigation_launch.py map:=/path/to/map.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch arguments
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file')

    # Default Nav2 params file
    default_params = os.path.join(
        nav2_bringup_dir, 'params', 'nav2_params.yaml'
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'map',
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 parameters file'
        ),

        # Include Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items()
        ),
    ])
```

---

## Demonstration: Running Navigation

### Step 1: Launch Simulation

```bash
# Terminal 1: Launch Gazebo
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Step 2: Launch Navigation

```bash
# Terminal 2: Launch Nav2 with map
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    map:=/path/to/your/map.yaml \
    use_sim_time:=true
```

### Step 3: Set Initial Pose in RViz2

1. Click **2D Pose Estimate**
2. Click on map where robot actually is
3. Drag to set orientation

### Step 4: Send Goals

**Option A: RViz2**
1. Click **Nav2 Goal**
2. Click destination on map

**Option B: Code**
```bash
# Terminal 3: Run goal sender
python3 nav2_goal_sender.py
```

**Option C: Command Line**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}}"
```

---

## Monitoring Navigation

### Key Topics to Watch

```bash
# See the computed path
ros2 topic echo /plan

# See velocity commands
ros2 topic echo /cmd_vel

# See robot position
ros2 topic echo /amcl_pose
```

### Navigation Status

```bash
# Check if Nav2 nodes are active
ros2 lifecycle list /controller_server
ros2 lifecycle list /planner_server
```

---

## Key Concepts Summary

| Concept | Description |
|---------|-------------|
| **NavigateToPose** | Action to send navigation goals |
| **AMCL** | Particle filter localization |
| **Planner Server** | Computes global path |
| **Controller Server** | Follows path locally |
| **Behavior Tree** | Coordinates navigation logic |

---

## Hands-On Exercise

### Exercise I4.1: Navigate to Multiple Waypoints

Modify `nav2_goal_sender.py` to:
1. Store a list of waypoints
2. Navigate to each waypoint in sequence
3. Wait for each goal to complete before sending the next

### Exercise I4.2: Handle Navigation Failures

Add error handling for:
1. Goal rejection (obstacle at goal location)
2. Navigation timeout (robot stuck)
3. Path planning failure (no path exists)

---

## AI Agent Assisted Prompts

### Prompt 1: Navigation Tuning
```
My robot reaches its navigation goals but stops about 30cm short. What Nav2
parameters control goal tolerance? How do I make the robot get closer to the
exact goal position?
```

### Prompt 2: Dynamic Obstacles
```
My robot navigates around static obstacles but collides with moving people.
How does Nav2 handle dynamic obstacles? What parameters affect reaction time?
```

### Prompt 3: Multi-Robot Navigation
```
I'm deploying multiple robots in the same environment. How do I configure
Nav2 to avoid robot-robot collisions? What about coordinating paths?
```

---

## Summary

In this lesson, you learned:

1. **Nav2 components**: Map Server, AMCL, Planner, Controller, Behaviors
2. How to **launch Nav2** with a pre-built map
3. How to **send navigation goals** programmatically
4. How to **monitor navigation** status and results
5. Basic **Nav2 action client** usage

---

## Next Steps

**Congratulations!** You've completed the Intermediate tier!

- **Test yourself**: Complete [Intermediate Exercises](../exercises/intermediate-exercises.md)
- **Ready for more?**: Continue to [Advanced Tier](../advanced/A1-costmap-configuration.md)

You can now build perception pipelines, manage transforms, run SLAM, and navigate autonomously!
