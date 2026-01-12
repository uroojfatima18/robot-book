#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Physical AI & Humanoid Robotics Textbook
#
# Chapter 1: The Robotic Nervous System (ROS 2)
# Lesson: I1 - Nodes, Topics, Services, and Actions
# Example: Minimal Subscriber Node

"""
ROS 2 Minimal Subscriber Example

This node demonstrates the fundamental subscriber pattern in ROS 2:
1. Create a node by subclassing rclpy.node.Node
2. Create a subscription with create_subscription()
3. Define a callback function to process incoming messages
4. Spin the node to receive and process messages

Usage:
    # Terminal 1: Run the publisher first
    python3 minimal_publisher.py

    # Terminal 2: Run the subscriber
    python3 minimal_subscriber.py

    # The subscriber will display messages as they arrive
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A minimal ROS 2 subscriber node.

    This node subscribes to String messages from the /chatter topic.
    When a message arrives, the callback logs its contents.
    """

    def __init__(self):
        """Initialize the subscriber node."""
        # Call parent constructor with node name
        super().__init__('minimal_subscriber')

        # Create subscription
        # Arguments: message_type, topic_name, callback, queue_size
        self.subscription = self.create_subscription(
            String,                 # Message type
            'chatter',              # Topic name
            self.listener_callback, # Callback function
            10                      # QoS queue depth
        )

        # Prevent unused variable warning
        self.subscription  # noqa: B018

        # Track message count for statistics
        self.message_count = 0

        # Log startup message
        self.get_logger().info('Minimal subscriber started, listening on /chatter')

    def listener_callback(self, msg: String):
        """
        Subscription callback - called when a message is received.

        Args:
            msg: The received String message
        """
        self.message_count += 1

        # Log the received message
        self.get_logger().info(
            f'Received [{self.message_count}]: "{msg.data}"'
        )


def main(args=None):
    """
    Main entry point.

    Initializes rclpy, creates the node, spins it, and cleans up.
    """
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    node = MinimalSubscriber()

    try:
        # Spin the node - this blocks and processes callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.get_logger().info(
            f'Keyboard interrupt. Received {node.message_count} messages total.'
        )
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
