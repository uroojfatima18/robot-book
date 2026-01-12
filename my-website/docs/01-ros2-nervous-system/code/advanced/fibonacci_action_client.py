#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Physical AI & Humanoid Robotics Textbook
#
# Chapter 1: The Robotic Nervous System (ROS 2)
# Lesson: A2 - Advanced ROS 2 Patterns & AI Integration
# Example: Fibonacci Action Client

"""
Fibonacci Action Client

This client sends goals to the Fibonacci action server and
handles feedback and results.

Prerequisites:
    sudo apt install ros-humble-action-tutorials-interfaces

Usage:
    # Terminal 1: Start the server first
    python3 fibonacci_action_server.py

    # Terminal 2: Run this client
    python3 fibonacci_action_client.py

    # Or with custom order:
    python3 fibonacci_action_client.py --order 15
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):
    """Action client for Fibonacci computation."""

    def __init__(self):
        super().__init__('fibonacci_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

        # Track goal handle for potential cancellation
        self._goal_handle = None

        self.get_logger().info('Fibonacci action client initialized')

    def send_goal(self, order: int):
        """
        Send a Fibonacci goal to the server.

        Args:
            order: The order of the Fibonacci sequence to compute
        """
        # Create goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: order={order}')

        # Send goal asynchronously with feedback callback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Add callback for when goal is accepted/rejected
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle the server's response to our goal request.

        Args:
            future: Future containing the goal handle
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by server')
            rclpy.shutdown()
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted by server')

        # Request the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Handle the final result from the action server.

        Args:
            future: Future containing the result
        """
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Goal succeeded!')
            self.get_logger().info(f'Result sequence: {result.sequence}')
            self.get_logger().info(f'Sequence length: {len(result.sequence)}')
        elif status == 5:  # CANCELED
            self.get_logger().info(f'Goal was canceled')
            self.get_logger().info(f'Partial result: {result.sequence}')
        elif status == 6:  # ABORTED
            self.get_logger().error(f'Goal was aborted')
        else:
            self.get_logger().warn(f'Goal finished with status: {status}')

        # Shutdown after receiving result
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """
        Handle feedback messages during execution.

        Args:
            feedback_msg: The feedback message from the server
        """
        feedback = feedback_msg.feedback
        sequence = feedback.partial_sequence

        self.get_logger().info(
            f'Feedback: {len(sequence)} numbers computed, '
            f'latest: {sequence[-1] if sequence else "N/A"}'
        )

    def cancel_goal(self):
        """Request cancellation of the current goal."""
        if self._goal_handle is not None:
            self.get_logger().info('Canceling goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        """Handle cancellation response."""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().warn('Goal cancellation failed')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    client = FibonacciActionClient()

    # Parse command line arguments for order
    order = 10  # default
    if len(sys.argv) > 1:
        if sys.argv[1] == '--order' and len(sys.argv) > 2:
            try:
                order = int(sys.argv[2])
            except ValueError:
                print(f'Invalid order: {sys.argv[2]}, using default 10')

    # Send goal
    client.send_goal(order)

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('Keyboard interrupt')
        client.cancel_goal()


if __name__ == '__main__':
    main()
