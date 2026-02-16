#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Physical AI & Humanoid Robotics Textbook
#
# Chapter 1: The Robotic Nervous System (ROS 2)
# Lesson: A2 - Advanced ROS 2 Patterns & AI Integration
# Example: Fibonacci Action Server

"""
Fibonacci Action Server

This action server computes Fibonacci sequences up to a requested order.
It demonstrates:
- Goal handling and acceptance
- Feedback publishing during execution
- Result generation upon completion
- Cancellation handling

Prerequisites:
    sudo apt install ros-humble-action-tutorials-interfaces

Usage:
    # Terminal 1: Start the server
    python3 fibonacci_action_server.py

    # Terminal 2: Test with CLI
    ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 10}"

    # Terminal 3: Monitor feedback
    ros2 action info /fibonacci
"""

import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):
    """Action server that computes Fibonacci sequences with feedback."""

    def __init__(self):
        super().__init__('fibonacci_action_server')

        # Create callback group for concurrent handling
        # ReentrantCallbackGroup allows multiple callbacks to execute simultaneously
        self.callback_group = ReentrantCallbackGroup()

        # Create action server with all callbacks
        self._action_server = ActionServer(
            self,
            Fibonacci,                      # Action type
            'fibonacci',                    # Action name
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Fibonacci action server is ready')
        self.get_logger().info('Send goals with: ros2 action send_goal /fibonacci '
                               'action_tutorials_interfaces/action/Fibonacci "{order: 10}"')

    def goal_callback(self, goal_request):
        """
        Decide whether to accept or reject incoming goals.

        Called immediately when a new goal is received.
        This should be fast - don't do heavy computation here.

        Args:
            goal_request: The goal request message

        Returns:
            GoalResponse.ACCEPT or GoalResponse.REJECT
        """
        self.get_logger().info(f'Received goal request: order={goal_request.order}')

        # Validate the request
        if goal_request.order < 0:
            self.get_logger().warn('Rejecting goal: order must be non-negative')
            return GoalResponse.REJECT

        if goal_request.order > 100:
            self.get_logger().warn('Rejecting goal: order too large (max 100)')
            return GoalResponse.REJECT

        self.get_logger().info('Goal accepted')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Handle cancellation requests.

        Called when a client requests cancellation.

        Args:
            goal_handle: The goal being cancelled

        Returns:
            CancelResponse.ACCEPT or CancelResponse.REJECT
        """
        self.get_logger().info('Received cancel request')
        # Generally, accept all cancellation requests for safety
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Execute the Fibonacci computation.

        This is the main execution logic. It runs in a separate thread
        and can be async for non-blocking operations.

        Args:
            goal_handle: Handle to the current goal

        Returns:
            The result message
        """
        self.get_logger().info('Executing goal...')

        # Initialize Fibonacci sequence
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Compute Fibonacci sequence with feedback
        for i in range(1, goal_handle.request.order):
            # IMPORTANT: Check for cancellation at each step
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')

                # Return partial result
                result = Fibonacci.Result()
                result.sequence = feedback_msg.partial_sequence
                return result

            # Compute next Fibonacci number
            next_number = (
                feedback_msg.partial_sequence[-1] +
                feedback_msg.partial_sequence[-2]
            )
            feedback_msg.partial_sequence.append(next_number)

            # Publish feedback
            self.get_logger().info(
                f'Feedback: {len(feedback_msg.partial_sequence)} numbers computed'
            )
            goal_handle.publish_feedback(feedback_msg)

            # Simulate computation time (in real robots, this would be actual work)
            time.sleep(0.5)

        # Mark goal as succeeded
        goal_handle.succeed()

        # Create and return result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence

        self.get_logger().info(f'Goal succeeded with {len(result.sequence)} numbers')
        self.get_logger().debug(f'Final sequence: {result.sequence}')

        return result


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = FibonacciActionServer()

    # Use MultiThreadedExecutor to handle concurrent goals
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
