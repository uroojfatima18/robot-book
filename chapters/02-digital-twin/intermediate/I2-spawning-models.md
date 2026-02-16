# I2: Spawning and Controlling Models

> Spawn URDF humanoid models and control joints programmatically.

## Learning Objectives

By the end of this lesson, you will be able to:
- Spawn URDF/XACRO models into Gazebo via ROS 2
- Use the spawn_entity service
- Create launch files for simulation
- Control joints with ros2_control
- Publish joint commands from Python

---

## Prerequisites

- Completed I1 (Building Worlds)
- Valid URDF model from Chapter 1
- Understanding of ROS 2 launch files

---

## Introduction

In I1, you created empty worlds. Now you'll populate them with your humanoid robot and make it move. This involves:

1. **Spawning**: Placing the robot model in the simulation
2. **Controlling**: Sending joint commands via ROS 2

---

## Part 1: Spawning Models

### Method 1: Command Line Spawning

The simplest way to spawn a model:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Start Gazebo with your world
ros2 launch gazebo_ros gazebo.launch.py world:=simple_lab.world &

# Wait for Gazebo to initialize
sleep 5

# Spawn the robot
ros2 run gazebo_ros spawn_entity.py \
  -entity humanoid \
  -file /path/to/humanoid.urdf \
  -x 0 -y 0 -z 1.0
```

**Spawn Arguments**:

| Argument | Purpose | Example |
|----------|---------|---------|
| `-entity` | Name in Gazebo | `humanoid` |
| `-file` | Path to URDF | `/home/user/robot.urdf` |
| `-topic` | Robot description topic | `/robot_description` |
| `-x, -y, -z` | Initial position | `0 0 1.0` |
| `-R, -P, -Y` | Initial rotation (radians) | `0 0 1.57` |

### Method 2: From Robot Description Topic

If your URDF is published to a topic:

```bash
# Spawn from topic
ros2 run gazebo_ros spawn_entity.py \
  -entity humanoid \
  -topic /robot_description \
  -x 0 -y 0 -z 1.0
```

### Method 3: Launch File (Recommended)

Create `spawn_humanoid.launch.py`:

```python
#!/usr/bin/env python3
"""
Launch file to spawn humanoid robot in Gazebo.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_lab.world',
        description='World file to load'
    )

    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='1.0')

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Robot state publisher (for TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open('/path/to/humanoid.urdf').read()
        }]
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid',
            '-topic', '/robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z')
        ],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

Launch with:

```bash
ros2 launch your_package spawn_humanoid.launch.py world:=simple_lab.world z:=1.5
```

---

## Part 2: Verifying the Spawn

### Check Model States

```bash
# List all models
ros2 topic echo /gazebo/model_states --once
```

Expected output:
```yaml
name:
- ground_plane
- humanoid
pose:
- position: {x: 0.0, y: 0.0, z: 0.0}
  ...
```

### Check Joint States

```bash
# Monitor joint positions
ros2 topic echo /joint_states
```

You should see all joints with positions, velocities, and efforts.

---

## Part 3: Controlling Joints

### Understanding ros2_control

ros2_control provides a standardized interface for robot control:

```
┌──────────────────────────────────────────────────────┐
│                    Your Controller                    │
│            (publishes to /joint_trajectory)           │
└─────────────────────────┬────────────────────────────┘
                          │
                          ▼
┌──────────────────────────────────────────────────────┐
│              ros2_control Manager                     │
│         (handles control loop at 1000Hz)              │
└─────────────────────────┬────────────────────────────┘
                          │
                          ▼
┌──────────────────────────────────────────────────────┐
│           Gazebo ros2_control Plugin                  │
│          (applies forces to simulation)               │
└──────────────────────────────────────────────────────┘
```

### URDF Controller Configuration

Add to your URDF for ros2_control:

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="left_hip_yaw">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- Repeat for each joint -->
</ros2_control>
```

### Gazebo Plugin

Add to your URDF's `<robot>` tag:

```xml
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <robot_param>robot_description</robot_param>
    <robot_param_node>robot_state_publisher</robot_param_node>
    <parameters>$(find your_package)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

### Controller Configuration

Create `controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - left_hip_yaw
      - left_hip_roll
      - left_hip_pitch
      - left_knee
      - left_ankle_pitch
      - left_ankle_roll
      # ... add all joints

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity
```

---

## Part 4: Python Joint Commander

Create `joint_commander.py` to send joint commands:

```python
#!/usr/bin/env python3
"""
Joint Commander - Send position commands to humanoid joints.

Usage:
    ros2 run your_package joint_commander.py

Publishes to:
    /joint_trajectory_controller/joint_trajectory
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math


class JointCommander(Node):
    """Node for commanding humanoid joint positions."""

    def __init__(self):
        super().__init__('joint_commander')

        # Joint names (must match URDF)
        self.joint_names = [
            'left_hip_yaw',
            'left_hip_roll',
            'left_hip_pitch',
            'left_knee',
            'left_ankle_pitch',
            'left_ankle_roll',
            'right_hip_yaw',
            'right_hip_roll',
            'right_hip_pitch',
            'right_knee',
            'right_ankle_pitch',
            'right_ankle_roll',
        ]

        # Publisher
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Timer for periodic commands
        self.timer = self.create_timer(2.0, self.send_command)
        self.position_index = 0

        self.get_logger().info('Joint Commander initialized')

    def send_command(self):
        """Send a joint trajectory command."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()

        # Alternate between two poses
        if self.position_index % 2 == 0:
            # Standing pose
            point.positions = [0.0] * len(self.joint_names)
        else:
            # Slight knee bend
            point.positions = [
                0.0,   # left_hip_yaw
                0.0,   # left_hip_roll
                -0.2,  # left_hip_pitch (bend forward)
                0.4,   # left_knee (bend)
                -0.2,  # left_ankle_pitch
                0.0,   # left_ankle_roll
                0.0,   # right_hip_yaw
                0.0,   # right_hip_roll
                -0.2,  # right_hip_pitch
                0.4,   # right_knee
                -0.2,  # right_ankle_pitch
                0.0,   # right_ankle_roll
            ]

        # Time to reach position
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(f'Sent command (pose {self.position_index % 2})')
        self.position_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()

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

### Run the Commander

```bash
# Terminal 1: Launch simulation
ros2 launch your_package spawn_humanoid.launch.py

# Terminal 2: Start controllers
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active joint_trajectory_controller

# Terminal 3: Run commander
ros2 run your_package joint_commander.py
```

---

## Part 5: Direct Topic Publishing

For quick testing without ros2_control:

```bash
# Publish a single joint command
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{
    joint_names: ['left_knee', 'right_knee'],
    points: [
      {
        positions: [0.5, 0.5],
        time_from_start: {sec: 1, nanosec: 0}
      }
    ]
  }"
```

---

## Troubleshooting

### Model Doesn't Spawn

```bash
# Check for errors
ros2 topic echo /rosout

# Verify URDF is valid
check_urdf humanoid.urdf
```

### Joints Don't Move

1. Check controller is active:
   ```bash
   ros2 control list_controllers
   ```

2. Verify joint names match:
   ```bash
   ros2 topic echo /joint_states
   ```

3. Check command topic:
   ```bash
   ros2 topic info /joint_trajectory_controller/joint_trajectory
   ```

### Robot Falls Over

- Check inertia values in URDF
- Verify foot friction coefficients
- Start with robot at rest before commanding

---

## What's Next?

In the Advanced tier, you'll connect simulation to a real robot for digital twin operation.

**Next**: [A1: Digital Twin Architecture](../advanced/A1-data-synchronization.md)

---

## AI Agent Assisted Prompts

### Spawn Debugging
```
I'm trying to spawn my humanoid URDF in Gazebo but getting "Service call failed".
The spawn_entity.py output shows no errors. Walk me through debugging steps
including: checking the URDF, verifying Gazebo services, and testing spawn.
```

### Controller Setup
```
Create a complete ros2_control configuration for a 20-DOF humanoid robot with:
- Position control for all joints
- Velocity limits (2 rad/s)
- Effort limits (50 Nm)
Include the URDF additions, controller YAML, and launch file modifications.
```

### Motion Sequence
```
Write a Python ROS 2 node that commands a humanoid to perform a simple squat:
1. Start standing
2. Bend knees to 45 degrees over 2 seconds
3. Hold for 1 second
4. Return to standing over 2 seconds
5. Repeat

Include proper trajectory interpolation and joint limits checking.
```

---

## Summary

- Spawn models with `spawn_entity.py` or launch files
- ros2_control provides standardized joint control
- Configure controllers in YAML, reference in URDF
- Use JointTrajectory messages for position commands
- Verify with `/joint_states` topic echo

---

| Previous | Up | Next |
|----------|-----|------|
| [I1: Building Worlds](I1-building-worlds.md) | [Intermediate Tier](../README.md#intermediate-tier) | [A1: Digital Twin Architecture](../advanced/A1-data-synchronization.md) |
