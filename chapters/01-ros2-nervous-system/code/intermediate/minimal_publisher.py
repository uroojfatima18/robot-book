#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Physical AI & Humanoid Robotics Textbook
#
# Chapter 1: The Robotic Nervous System (ROS 2)
# Lesson: I1 - Nodes, Topics, Services, and Actions
# Example: Minimal Publisher Node

"""
ROS 2 Minimal Publisher Example

This node demonstrates the fundamental publisher pattern in ROS 2:
1. Create a node by subclassing rclpy.node.Node
2. Create a publisher with create_publisher()
3. Use a timer to publish messages periodically
4. Spin the node to process callbacks

Usage:
    # Terminal 1: Run the publisher
    python3 minimal_publisher.py

    # Terminal 2: Echo the messages
    ros2 topic echo /chatter

    # Terminal 3: Check the topic info
    ros2 topic info /chatter
    ros2 topic hz /chatter
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal ROS 2 publisher node.

    This node publishes String messages to the /chatter topic at 2 Hz.
    Each message contains a greeting and an incrementing counter.
    """

    def __init__(self):
        """Initialize the publisher node."""
        # Call parent constructor with node name
        super().__init__('minimal_publisher')

        # Create publisher
        # Arguments: message_type, topic_name, queue_size
        self.publisher = self.create_publisher(
            String,      # Message type
            'chatter',   # Topic name
            10           # QoS queue depth
        )

        # Create timer for periodic publishing
        # Arguments: period_in_seconds, callback_function
        timer_period = 0.5  # 2 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize counter
        self.count = 0

        # Log startup message
        self.get_logger().info('Minimal publisher started, publishing to /chatter')

    def timer_callback(self):
        """
        Timer callback - called every timer_period seconds.

        Creates a String message with a counter and publishes it.
        """
        # Create message
        msg = String()
        msg.data = f'Hello, ROS 2! Count: {self.count}'

        # Publish message
        self.publisher.publish(msg)

        # Log what we published
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter
        self.count += 1


def main(args=None):
    """
    Main entry point.

    Initializes rclpy, creates the node, spins it, and cleans up.
    """
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    node = MinimalPublisher()

    try:
        # Spin the node - this blocks and processes callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
