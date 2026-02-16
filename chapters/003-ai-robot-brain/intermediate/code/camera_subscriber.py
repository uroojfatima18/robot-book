#!/usr/bin/env python3
"""
camera_subscriber.py

Subscribes to camera image topic and displays using OpenCV.
Demonstrates basic image subscription and cv_bridge usage.

ROS 2 Topics:
  Subscribed: /camera/image_raw (sensor_msgs/Image)

Dependencies: rclpy, sensor_msgs, cv_bridge, opencv-python

Usage:
  ros2 run <package_name> camera_subscriber
  # Or run directly: python3 camera_subscriber.py
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
