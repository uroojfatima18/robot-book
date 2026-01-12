#!/usr/bin/env python3
"""
depth_processor.py

Processes depth camera images to find closest obstacle.
Publishes minimum distance as a Float32 message.

ROS 2 Topics:
  Subscribed: /depth_camera/depth/image_raw (sensor_msgs/Image)
  Published:  /closest_obstacle (std_msgs/Float32)

Dependencies: rclpy, sensor_msgs, std_msgs, cv_bridge, numpy

Usage:
  ros2 run <package_name> depth_processor
  # Or run directly: python3 depth_processor.py

Parameters:
  min_valid_depth (float): Minimum valid depth in meters (default: 0.1)
  max_valid_depth (float): Maximum valid depth in meters (default: 10.0)
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
