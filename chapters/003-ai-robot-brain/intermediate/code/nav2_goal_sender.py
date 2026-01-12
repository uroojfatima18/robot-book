#!/usr/bin/env python3
"""
nav2_goal_sender.py

Sends navigation goals to Nav2 and monitors progress.
Demonstrates Nav2 action client usage.

ROS 2 Actions:
  Client: /navigate_to_pose (nav2_msgs/NavigateToPose)

Dependencies: rclpy, nav2_msgs, geometry_msgs, rclpy.action

Usage:
  ros2 run <package_name> nav2_goal_sender
  # Or run directly: python3 nav2_goal_sender.py

Example:
  # Modify the goal coordinates in main() and run
  python3 nav2_goal_sender.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


class Nav2GoalSender(Node):
    """ROS 2 node that sends navigation goals to Nav2."""

    def __init__(self):
        super().__init__('nav2_goal_sender')

        # Nav2 action client
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server available!')

    def send_goal(self, x: float, y: float, yaw: float = 0.0):
        """Send a navigation goal to Nav2.

        Args:
            x: Target X position in map frame (meters)
            y: Target Y position in map frame (meters)
            yaw: Target orientation (radians)
        """
        goal_msg = NavigateToPose.Goal()

        # Create target pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)

        self.get_logger().info(f'Sending goal: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')

        # Send goal with callbacks
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            return

        self.get_logger().info('Goal accepted!')

        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose.position

        # Log progress periodically
        self.get_logger().info(
            f'Current position: ({current_pose.x:.2f}, {current_pose.y:.2f})'
        )

    def result_callback(self, future):
        """Handle navigation result."""
        result = future.result().result

        self.get_logger().info('Navigation completed!')

        # Optionally send another goal or shutdown
        # self.send_goal(0.0, 0.0)  # Return home


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalSender()

    # Send example goal - modify these coordinates for your map
    node.send_goal(x=2.0, y=1.0, yaw=0.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
