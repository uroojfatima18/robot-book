# Code Standards Contract: Chapter 1

**Branch**: `001-ros2-chapter` | **Date**: 2025-12-20
**Purpose**: Define coding standards for all examples in Chapter 1

## Python Code Standards

### File Header

All Python files MUST start with:

```python
#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Physical AI & Humanoid Robotics Textbook
#
# Chapter 1: The Robotic Nervous System (ROS 2)
# Lesson: {Lesson ID} - {Lesson Title}
# Example: {Brief description}
```

### Import Order

```python
# Standard library imports
import sys
import time

# Third-party imports
import numpy as np  # if needed

# ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

# Message imports
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
```

### Node Class Template

```python
class MinimalPublisher(Node):
    """
    A minimal ROS 2 publisher node for demonstration.

    This node publishes String messages to the 'topic' topic
    at a rate of 1 Hz.

    Attributes:
        publisher_: The publisher object
        timer: Timer for periodic publishing
        count: Message counter
    """

    def __init__(self):
        super().__init__('minimal_publisher')

        # Declare parameters (if any)
        self.declare_parameter('publish_rate', 1.0)

        # Get parameters
        rate = self.get_parameter('publish_rate').value

        # Create publisher
        self.publisher_ = self.create_publisher(
            String,
            'topic',
            10  # QoS depth
        )

        # Create timer
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        # Initialize state
        self.count = 0

        self.get_logger().info('MinimalPublisher initialized')

    def timer_callback(self):
        """Timer callback to publish messages."""
        msg = String()
        msg.data = f'Hello ROS 2: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1
```

### Main Function Template

```python
def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)

    node = MinimalPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Bash Command Standards

### Single Commands

```bash
# Verify ROS 2 installation
$ ros2 --version

# Output:
# ros2 0.9.0
```

### Multi-line Commands

```bash
# Source the ROS 2 environment
$ source /opt/ros/humble/setup.bash

# Run the talker demo
$ ros2 run demo_nodes_cpp talker

# In another terminal, run the listener
$ ros2 run demo_nodes_cpp listener
```

### Installation Commands

```bash
# Update package lists
sudo apt update

# Install ROS 2 Humble desktop
sudo apt install ros-humble-desktop

# Add to bashrc for persistent sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## URDF/XML Standards

### File Header

```xml
<?xml version="1.0"?>
<!--
  Chapter 1: The Robotic Nervous System (ROS 2)
  URDF: Basic Humanoid Robot
  Description: Minimal humanoid structure for RViz visualization
-->
<robot name="humanoid_basic" xmlns:xacro="http://www.ros.org/wiki/xacro">
```

### Link Definition

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.2 0.3 0.1"/>
    </geometry>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.3 0.1"/>
    </geometry>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </collision>
</link>
```

### Joint Definition

```xml
<joint name="torso_joint" type="fixed">
  <parent link="base_link"/>
  <child link="torso"/>
  <origin xyz="0 0 0.15" rpy="0 0 0"/>
</joint>
```

---

## Launch File Standards (Python)

```python
#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Chapter 1: The Robotic Nervous System (ROS 2)
# Launch file: talker_listener_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for talker and listener demo."""
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            output='screen',
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener',
            output='screen',
        ),
    ])
```

---

## Naming Conventions

| Type | Convention | Example |
|------|------------|---------|
| Python files | snake_case | `minimal_publisher.py` |
| Node names | snake_case | `minimal_publisher` |
| Topic names | snake_case with slashes | `/sensor/imu` |
| Service names | snake_case with slashes | `/add_two_ints` |
| Action names | snake_case | `fibonacci` |
| Parameter names | snake_case | `publish_rate` |
| URDF link names | snake_case | `base_link` |
| URDF joint names | snake_case | `torso_joint` |

---

## Error Handling

### ROS 2 Nodes

```python
def timer_callback(self):
    """Timer callback with error handling."""
    try:
        msg = String()
        msg.data = self.generate_message()
        self.publisher_.publish(msg)
    except Exception as e:
        self.get_logger().error(f'Failed to publish: {e}')
```

### Main Function

```python
def main(args=None):
    """Main with comprehensive error handling."""
    rclpy.init(args=args)

    node = None
    try:
        node = ExampleNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().fatal(f'Unhandled exception: {e}')
        raise
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()
```

---

## Documentation Standards

### Inline Comments

```python
# Create publisher with QoS depth of 10
# This means up to 10 messages can be queued
self.publisher_ = self.create_publisher(String, 'topic', 10)
```

### Docstrings

```python
def publish_message(self, content: str) -> bool:
    """
    Publish a message to the topic.

    Args:
        content: The string content to publish

    Returns:
        True if publish succeeded, False otherwise

    Raises:
        RuntimeError: If publisher is not initialized
    """
```

---

## Testing Requirements

All code examples MUST be tested with:

1. **Syntax validation**: `python3 -m py_compile {file}`
2. **Import validation**: `python3 -c "import {module}"`
3. **Runtime validation**: Execute and verify expected output

```bash
# Syntax check
$ python3 -m py_compile minimal_publisher.py

# Import check (with ROS 2 sourced)
$ python3 -c "from minimal_publisher import MinimalPublisher"

# Runtime test
$ python3 minimal_publisher.py
# [INFO] [minimal_publisher]: Publishing: "Hello ROS 2: 0"
# [INFO] [minimal_publisher]: Publishing: "Hello ROS 2: 1"
```

---

## Safety Requirements

Per Constitution Principle VII (Safety & Ethics First):

1. **No unsafe robot commands** - All examples run in simulation only
2. **Simulation validation** - Code tested in Gazebo before publishing
3. **Safety warnings** - Include warnings before potentially harmful operations

```python
# WARNING: This example controls robot motion.
# Always test in simulation before running on real hardware.
# Ensure emergency stop is accessible when running on physical robots.
```
