#!/usr/bin/env python3
"""
tf2_broadcaster.py

Broadcasts static and dynamic transforms for robot sensors.
Demonstrates TF2 transform publishing.

ROS 2 TF:
  Published: /tf (for dynamic) or /tf_static (for static)

Dependencies: rclpy, geometry_msgs, tf2_ros

Usage:
  ros2 run <package_name> tf2_broadcaster
  # Or run directly: python3 tf2_broadcaster.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import math


class TF2Broadcaster(Node):
    """ROS 2 node that broadcasts coordinate frame transforms."""

    def __init__(self):
        super().__init__('tf2_broadcaster')

        # Static transform broadcaster (for fixed sensor mounts)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Dynamic transform broadcaster (for moving parts)
        self.dynamic_broadcaster = TransformBroadcaster(self)

        # Publish static transforms once
        self.publish_static_transforms()

        # Timer for dynamic transforms (e.g., rotating sensor)
        self.timer = self.create_timer(0.1, self.publish_dynamic_transforms)
        self.angle = 0.0

        self.get_logger().info('TF2 broadcaster started')

    def publish_static_transforms(self):
        """Publish static transforms for fixed sensor mounts."""

        # Camera mount: 10cm forward, 30cm up from base_link
        camera_tf = TransformStamped()
        camera_tf.header.stamp = self.get_clock().now().to_msg()
        camera_tf.header.frame_id = 'base_link'
        camera_tf.child_frame_id = 'camera_link'

        # Translation (meters)
        camera_tf.transform.translation.x = 0.1
        camera_tf.transform.translation.y = 0.0
        camera_tf.transform.translation.z = 0.3

        # Rotation (quaternion - no rotation)
        camera_tf.transform.rotation.x = 0.0
        camera_tf.transform.rotation.y = 0.0
        camera_tf.transform.rotation.z = 0.0
        camera_tf.transform.rotation.w = 1.0

        # LIDAR mount: centered, 15cm up
        lidar_tf = TransformStamped()
        lidar_tf.header.stamp = self.get_clock().now().to_msg()
        lidar_tf.header.frame_id = 'base_link'
        lidar_tf.child_frame_id = 'base_scan'

        lidar_tf.transform.translation.x = 0.0
        lidar_tf.transform.translation.y = 0.0
        lidar_tf.transform.translation.z = 0.15

        lidar_tf.transform.rotation.x = 0.0
        lidar_tf.transform.rotation.y = 0.0
        lidar_tf.transform.rotation.z = 0.0
        lidar_tf.transform.rotation.w = 1.0

        # Publish both static transforms
        self.static_broadcaster.sendTransform([camera_tf, lidar_tf])
        self.get_logger().info('Published static transforms for camera_link and base_scan')

    def publish_dynamic_transforms(self):
        """Publish dynamic transforms for moving parts."""

        # Example: A rotating sensor platform
        sensor_tf = TransformStamped()
        sensor_tf.header.stamp = self.get_clock().now().to_msg()
        sensor_tf.header.frame_id = 'base_link'
        sensor_tf.child_frame_id = 'rotating_sensor'

        # Position: 20cm forward
        sensor_tf.transform.translation.x = 0.2
        sensor_tf.transform.translation.y = 0.0
        sensor_tf.transform.translation.z = 0.1

        # Rotation: slowly rotating around Z axis
        self.angle += 0.05  # radians per update
        if self.angle > 2 * math.pi:
            self.angle -= 2 * math.pi

        # Convert yaw angle to quaternion
        sensor_tf.transform.rotation.x = 0.0
        sensor_tf.transform.rotation.y = 0.0
        sensor_tf.transform.rotation.z = math.sin(self.angle / 2)
        sensor_tf.transform.rotation.w = math.cos(self.angle / 2)

        self.dynamic_broadcaster.sendTransform(sensor_tf)


def main(args=None):
    rclpy.init(args=args)
    node = TF2Broadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
