---
id: i_lesson1_camera_depth
title: "Camera and Depth Data Processing"
sidebar_position: 31
tier: intermediate
chapter: chapter_3_ai_brain
estimated_time: "60-90 minutes"
prerequisites: ["b_lesson3_slam_nav"]
---

# I1: Camera and Depth Data Processing

**Building Your First Perception Pipeline in ROS 2**

---

## Learning Objectives

By the end of this lesson, you will be able to:
- Subscribe to camera topics and process image data
- Use cv_bridge to convert between ROS 2 and OpenCV formats
- Process depth images to extract distance measurements
- Create a perception node that publishes processed data

---

## Prerequisites

- Completed Beginner tier (B1-B3)
- ROS 2 Humble or Iron installed
- OpenCV and cv_bridge packages installed
- Gazebo simulation environment set up

---

## ROS 2 Refresher: Nodes and Topics

> **Quick Review**: A **node** subscribes to topics, processes data, and publishes results.
>
> ```python
> self.create_subscription(MessageType, 'topic', callback, qos)
> self.create_publisher(MessageType, 'topic', qos)
> ```
>
> See [ROS 2 Refresher](../beginner/refresher-ros2.md) for more details.

---

## Theory: ROS 2 Image Pipeline

### Image Message Structure

ROS 2 uses `sensor_msgs/msg/Image` for camera data:

```python
# sensor_msgs/msg/Image structure
header:
  stamp: builtin_interfaces/Time  # When image was captured
  frame_id: string                # Camera frame (e.g., "camera_link")
height: uint32                    # Image height in pixels
width: uint32                     # Image width in pixels
encoding: string                  # Pixel format (e.g., "rgb8", "bgr8", "32FC1")
is_bigendian: uint8               # Endianness
step: uint32                      # Row length in bytes
data: uint8[]                     # Actual pixel data
```

### Common Encodings

| Encoding | Description | Use Case |
|----------|-------------|----------|
| `rgb8` | 8-bit RGB (3 channels) | Color cameras |
| `bgr8` | 8-bit BGR (OpenCV default) | Color cameras |
| `mono8` | 8-bit grayscale | Stereo cameras |
| `16UC1` | 16-bit unsigned depth | Depth in millimeters |
| `32FC1` | 32-bit float depth | Depth in meters |

### The cv_bridge Library

**cv_bridge** converts between ROS 2 Image messages and OpenCV (NumPy) arrays:

```python
from cv_bridge import CvBridge

bridge = CvBridge()

# ROS 2 Image → OpenCV
cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')

# OpenCV → ROS 2 Image
ros_image = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
```

---

## Code Example: Camera Subscriber

This node subscribes to a camera topic and displays the image:

```python
#!/usr/bin/env python3
"""
camera_subscriber.py

Subscribes to camera image topic and displays using OpenCV.
Demonstrates basic image subscription and cv_bridge usage.

ROS 2 Topics:
  Subscribed: /camera/image_raw (sensor_msgs/Image)

Dependencies: rclpy, sensor_msgs, cv_bridge, opencv-python
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraSubscriber(Node):
    """ROS 2 node that subscribes to camera images and displays them."""

    def __init__(self):
        super().__init__('camera_subscriber')

        # Create CV bridge for ROS 2 <-> OpenCV conversion
        self.bridge = CvBridge()

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10  # QoS depth
        )

        self.get_logger().info('Camera subscriber started. Waiting for images...')

    def image_callback(self, msg: Image):
        """Process incoming camera image.

        Args:
            msg: ROS 2 Image message from camera
        """
        try:
            # Convert ROS 2 Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Log image info (first time only to avoid spam)
            if not hasattr(self, '_logged_info'):
                self.get_logger().info(
                    f'Receiving images: {msg.width}x{msg.height}, '
                    f'encoding: {msg.encoding}'
                )
                self._logged_info = True

            # Display the image
            cv2.imshow('Camera View', cv_image)
            cv2.waitKey(1)  # Required for OpenCV window update

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Expected Output**:
```
[INFO] [camera_subscriber]: Camera subscriber started. Waiting for images...
[INFO] [camera_subscriber]: Receiving images: 640x480, encoding: rgb8
```

An OpenCV window will display the live camera feed.

---

## Code Example: Depth Processor

This node processes depth images to find the closest obstacle:

```python
#!/usr/bin/env python3
"""
depth_processor.py

Processes depth camera images to find closest obstacle.
Publishes minimum distance as a Float32 message.

ROS 2 Topics:
  Subscribed: /depth_camera/depth/image_raw (sensor_msgs/Image)
  Published:  /closest_obstacle (std_msgs/Float32)

Dependencies: rclpy, sensor_msgs, std_msgs, cv_bridge, numpy
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np


class DepthProcessor(Node):
    """ROS 2 node that processes depth images to find obstacles."""

    def __init__(self):
        super().__init__('depth_processor')

        # Parameters
        self.declare_parameter('min_valid_depth', 0.1)  # meters
        self.declare_parameter('max_valid_depth', 10.0)  # meters

        self.min_depth = self.get_parameter('min_valid_depth').value
        self.max_depth = self.get_parameter('max_valid_depth').value

        # CV Bridge
        self.bridge = CvBridge()

        # Subscriber for depth images
        self.depth_sub = self.create_subscription(
            Image,
            '/depth_camera/depth/image_raw',
            self.depth_callback,
            10
        )

        # Publisher for closest obstacle distance
        self.distance_pub = self.create_publisher(Float32, '/closest_obstacle', 10)

        self.get_logger().info(
            f'Depth processor started. Valid range: {self.min_depth}-{self.max_depth}m'
        )

    def depth_callback(self, msg: Image):
        """Process depth image to find closest obstacle.

        Args:
            msg: ROS 2 depth Image message (32FC1 encoding, values in meters)
        """
        try:
            # Convert to numpy array
            depth_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Handle NaN and infinite values
            depth_array = np.nan_to_num(depth_array, nan=np.inf, posinf=np.inf)

            # Filter to valid depth range
            valid_mask = (depth_array > self.min_depth) & (depth_array < self.max_depth)

            if np.any(valid_mask):
                # Find minimum distance (closest obstacle)
                min_distance = float(np.min(depth_array[valid_mask]))

                # Publish result
                distance_msg = Float32()
                distance_msg.data = min_distance
                self.distance_pub.publish(distance_msg)

                # Log periodically
                if not hasattr(self, '_count'):
                    self._count = 0
                self._count += 1
                if self._count % 30 == 0:  # Log every ~1 second at 30fps
                    self.get_logger().info(f'Closest obstacle: {min_distance:.2f}m')
            else:
                self.get_logger().warn('No valid depth readings in range')

        except Exception as e:
            self.get_logger().error(f'Depth processing failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessor()

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

**Expected Output**:
```
[INFO] [depth_processor]: Depth processor started. Valid range: 0.1-10.0m
[INFO] [depth_processor]: Closest obstacle: 1.45m
[INFO] [depth_processor]: Closest obstacle: 1.42m
```

---

## Demonstration: Running the Examples

### Step 1: Launch Simulation

```bash
# Terminal 1: Launch TurtleBot3 with sensors
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Step 2: Run Camera Subscriber

```bash
# Terminal 2: Run camera subscriber
cd chapters/003-ai-robot-brain/intermediate/code
python3 camera_subscriber.py
```

### Step 3: Run Depth Processor

```bash
# Terminal 3: Run depth processor
python3 depth_processor.py
```

### Step 4: Monitor Output

```bash
# Terminal 4: Monitor closest obstacle topic
ros2 topic echo /closest_obstacle
```

### Step 5: Drive Robot to Test

```bash
# Terminal 5: Keyboard control
ros2 run turtlebot3_teleop teleop_keyboard
```

Drive toward obstacles and watch the distance change!

---

## Key Concepts Summary

| Concept | Description |
|---------|-------------|
| **sensor_msgs/Image** | ROS 2 message type for camera data |
| **cv_bridge** | Converts ROS 2 ↔ OpenCV formats |
| **Encoding** | Pixel format (rgb8, bgr8, 32FC1, etc.) |
| **Depth Image** | Each pixel = distance in meters or mm |
| **QoS** | Quality of Service settings for reliability |

---

## Hands-On Exercise

### Exercise I1.1: Add Edge Detection

Modify `camera_subscriber.py` to add edge detection:

1. Convert the image to grayscale
2. Apply Canny edge detection
3. Display both original and edge images side by side

```python
# Hint: Use these OpenCV functions
gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 50, 150)
combined = np.hstack([cv_image, cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)])
```

### Exercise I1.2: Region-Based Depth Analysis

Modify `depth_processor.py` to find the closest obstacle in different regions:

1. Divide the depth image into left, center, and right thirds
2. Find the minimum distance in each region
3. Publish as a custom message or log all three values

This is useful for obstacle avoidance (knowing which direction is clearest).

---

## AI Agent Assisted Prompts

### Prompt 1: Image Processing Enhancement
```
I have a camera subscriber node that receives RGB images. I want to add
real-time object detection using a pre-trained model like YOLO or MobileNet.
What are the steps to integrate this with ROS 2? What are the performance
considerations for running at 30fps?
```

### Prompt 2: Depth Filtering Strategies
```
My depth camera returns noisy data with many outliers. What filtering
strategies can I use to clean up the depth data? Compare median filtering,
bilateral filtering, and temporal filtering for real-time use.
```

### Prompt 3: Multi-Camera Setup
```
I have two cameras on my robot - one front-facing and one rear-facing.
How should I structure my ROS 2 nodes to handle both cameras? Should I
use one node with two subscribers or two separate nodes?
```

---

## Summary

In this lesson, you learned:

1. **sensor_msgs/Image** structure and common encodings
2. **cv_bridge** for ROS 2 ↔ OpenCV conversion
3. How to **subscribe to camera topics** and process images
4. How to **process depth images** to find obstacles
5. Basic **image processing** with OpenCV in ROS 2

---

## Next Steps

- **Next Lesson**: [I2: TF2 Coordinate Frames](I2-tf2-coordinate-frames.md) - Managing transforms for multi-sensor robots
- **Exercises**: Try the coding exercises above before continuing
