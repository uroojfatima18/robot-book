# Contract: Code Example Standards

**Date**: 2025-12-27
**Feature**: 003-isaac-ai-brain

## Purpose

This contract defines coding standards for all code examples in Chapter 3, ensuring ROS 2 compliance and educational clarity.

---

## Python/ROS 2 Code Standards

### File Header

```python
#!/usr/bin/env python3
"""
[Brief description of what this code does]

Author: Physical AI Textbook
License: Apache 2.0
ROS 2 Version: Humble/Iron
"""
```

### Import Organization

```python
# Standard library
import sys
from typing import List, Optional

# Third-party
import numpy as np
import cv2

# ROS 2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
```

### Node Class Structure

```python
class ExampleNode(Node):
    """
    Node description for RAG indexing.
    
    Subscriptions:
        - /topic_name (MessageType): Description
    
    Publishers:
        - /output_topic (MessageType): Description
    
    Parameters:
        - param_name (type): Description
    """
    
    def __init__(self):
        super().__init__('example_node')
        
        # Declare parameters
        self.declare_parameter('param_name', 'default_value')
        
        # Create subscribers
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10  # QoS depth
        )
        
        # Create publishers
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.get_logger().info('Node initialized')
    
    def callback(self, msg):
        """Process incoming message."""
        # Implementation with inline comments
        pass
```

### Main Entry Point

```python
def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    
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

## YAML Configuration Standards

```yaml
# Configuration for [component]
# Used in: [lesson reference]

node_name:
  ros__parameters:
    # Parameter group description
    parameter_name: value  # Inline explanation
    
    # Another group
    nested:
      setting: value
```

---

## Launch File Standards

```python
#!/usr/bin/env python3
"""
Launch file for [purpose].

Usage:
    ros2 launch [package] [launch_file].launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Define nodes
    example_node = Node(
        package='package_name',
        executable='node_name',
        name='node_name',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time,
        example_node,
    ])
```

---

## Validation Requirements

| Criterion | Requirement |
|-----------|-------------|
| Syntax | Must pass `python3 -m py_compile` |
| Style | Must follow PEP 8 basics |
| Comments | Key lines annotated for learners |
| Error Handling | Include try/except for common failures |
| Logging | Use ROS 2 logger, not print() |
| Type Hints | Encouraged but not required |
| Docstrings | Required for classes and main functions |

---

## Code Example Checklist

For each code example, verify:

- [ ] File header present with description
- [ ] Imports organized correctly
- [ ] Node follows standard structure
- [ ] Inline comments explain key steps
- [ ] Expected output documented
- [ ] Dependencies listed
- [ ] Passes syntax validation
- [ ] Uses ROS 2 logging (not print)
