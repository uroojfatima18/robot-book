# ROS 2 Refresher (Chapter 1 Reference)

**Use this refresher when**: Lessons reference ROS 2 concepts from Chapter 1

---

## Quick Reference: ROS 2 Core Concepts

### Nodes
A **node** is an executable that uses ROS 2 to communicate with other nodes. Each node should have a single, modular purpose.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Node started!')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Topics
**Topics** are named buses for nodes to exchange messages asynchronously (publish/subscribe).

```python
# Publisher
self.publisher = self.create_publisher(String, 'topic_name', 10)

# Subscriber
self.subscription = self.create_subscription(
    String, 'topic_name', self.callback, 10)
```

### Messages
**Messages** are structured data types. Common types include:
- `std_msgs/String` - Simple text
- `geometry_msgs/Twist` - Velocity commands
- `sensor_msgs/Image` - Camera images
- `sensor_msgs/LaserScan` - LIDAR data

### Services
**Services** are synchronous request/response communication:

```python
# Service client
self.client = self.create_client(ServiceType, 'service_name')
```

### Launch Files
Launch files start multiple nodes with parameters:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='my_package', executable='my_node'),
    ])
```

---

## Essential Commands

```bash
# List running nodes
ros2 node list

# List topics
ros2 topic list

# Echo topic data
ros2 topic echo /topic_name

# Get topic info
ros2 topic info /topic_name

# Run a node
ros2 run package_name node_name

# Launch multiple nodes
ros2 launch package_name launch_file.py
```

---

## Need More Detail?

See [Chapter 1: ROS 2 Fundamentals](../../001-ros2-fundamentals/README.md) for complete coverage.
