#!/usr/bin/env python3
"""
Joint Commander - Send position commands to humanoid joints.

This node demonstrates how to control a humanoid robot's joints
by publishing JointTrajectory messages.

Usage:
    ros2 run your_package joint_commander.py

    # Or run directly
    python3 joint_commander.py

Topics Published:
    /joint_trajectory_controller/joint_trajectory (trajectory_msgs/JointTrajectory)

Parameters:
    joint_names: List of joint names to control (default: example joints)
    command_rate: Rate to send commands in Hz (default: 0.5)

Example Commands:
    # Set single joint
    ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
        trajectory_msgs/msg/JointTrajectory \
        "{joint_names: ['left_knee'], points: [{positions: [0.5], time_from_start: {sec: 1}}]}"

Author: Robot Book Chapter 2
"""

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration


class JointCommander(Node):
    """
    ROS 2 node for commanding humanoid joint positions.

    This node alternates between predefined poses to demonstrate
    joint control via JointTrajectory messages.
    """

    def __init__(self):
        super().__init__('joint_commander')

        # ============================================
        # Parameters
        # ============================================

        self.declare_parameter('joint_names', [
            'left_hip_yaw',
            'left_hip_roll',
            'left_hip_pitch',
            'left_knee',
            'left_ankle_pitch',
            'left_ankle_roll',
            'right_hip_yaw',
            'right_hip_roll',
            'right_hip_pitch',
            'right_knee',
            'right_ankle_pitch',
            'right_ankle_roll',
        ])

        self.declare_parameter('command_rate', 0.5)  # Hz
        self.declare_parameter('trajectory_duration', 1.0)  # seconds

        self.joint_names = self.get_parameter('joint_names').value
        command_rate = self.get_parameter('command_rate').value
        self.trajectory_duration = self.get_parameter('trajectory_duration').value

        # ============================================
        # Publishers and Subscribers
        # ============================================

        # QoS for reliable command delivery
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            qos
        )

        # Optional: Subscribe to current joint states for feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.current_positions = {}

        # ============================================
        # Predefined Poses
        # ============================================

        self.poses = self._define_poses()
        self.current_pose_index = 0

        # ============================================
        # Timer for Periodic Commands
        # ============================================

        period = 1.0 / command_rate if command_rate > 0 else 2.0
        self.timer = self.create_timer(period, self.send_next_pose)

        self.get_logger().info(
            f'Joint Commander initialized with {len(self.joint_names)} joints'
        )
        self.get_logger().info(f'Available poses: {list(self.poses.keys())}')

    def _define_poses(self) -> dict:
        """
        Define named poses for the humanoid.

        Returns:
            Dictionary mapping pose names to joint position lists.
        """
        n_joints = len(self.joint_names)

        poses = {
            'stand': [0.0] * n_joints,

            'slight_squat': [
                0.0,   # left_hip_yaw
                0.0,   # left_hip_roll
                -0.15, # left_hip_pitch
                0.3,   # left_knee
                -0.15, # left_ankle_pitch
                0.0,   # left_ankle_roll
                0.0,   # right_hip_yaw
                0.0,   # right_hip_roll
                -0.15, # right_hip_pitch
                0.3,   # right_knee
                -0.15, # right_ankle_pitch
                0.0,   # right_ankle_roll
            ],

            'deep_squat': [
                0.0,   # left_hip_yaw
                0.0,   # left_hip_roll
                -0.4,  # left_hip_pitch
                0.8,   # left_knee
                -0.4,  # left_ankle_pitch
                0.0,   # left_ankle_roll
                0.0,   # right_hip_yaw
                0.0,   # right_hip_roll
                -0.4,  # right_hip_pitch
                0.8,   # right_knee
                -0.4,  # right_ankle_pitch
                0.0,   # right_ankle_roll
            ],

            'left_weight_shift': [
                0.0,   # left_hip_yaw
                -0.1,  # left_hip_roll (shift left)
                0.0,   # left_hip_pitch
                0.0,   # left_knee
                0.0,   # left_ankle_pitch
                0.1,   # left_ankle_roll
                0.0,   # right_hip_yaw
                -0.1,  # right_hip_roll
                0.0,   # right_hip_pitch
                0.0,   # right_knee
                0.0,   # right_ankle_pitch
                0.1,   # right_ankle_roll
            ],
        }

        # Ensure all poses have correct number of joints
        for name, positions in poses.items():
            if len(positions) != n_joints:
                self.get_logger().warn(
                    f'Pose {name} has {len(positions)} joints, expected {n_joints}'
                )
                poses[name] = positions[:n_joints] + [0.0] * (n_joints - len(positions))

        return poses

    def joint_state_callback(self, msg: JointState):
        """Store current joint positions for feedback."""
        for name, position in zip(msg.name, msg.position):
            self.current_positions[name] = position

    def send_next_pose(self):
        """Send the next pose in the sequence."""
        pose_names = list(self.poses.keys())
        pose_name = pose_names[self.current_pose_index % len(pose_names)]
        positions = self.poses[pose_name]

        self.send_trajectory(positions)
        self.get_logger().info(f'Commanded pose: {pose_name}')

        self.current_pose_index += 1

    def send_trajectory(self, positions: List[float], duration: float = None):
        """
        Send a joint trajectory command.

        Args:
            positions: Target positions for each joint (radians)
            duration: Time to reach target (seconds), uses parameter if None
        """
        if len(positions) != len(self.joint_names):
            self.get_logger().error(
                f'Position count mismatch: {len(positions)} vs {len(self.joint_names)}'
            )
            return

        if duration is None:
            duration = self.trajectory_duration

        # Create message
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)  # Zero velocity at target
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9)
        )

        msg.points.append(point)

        self.publisher.publish(msg)

    def go_to_pose(self, pose_name: str, duration: float = None):
        """
        Command the robot to a named pose.

        Args:
            pose_name: Name of the pose (must be in self.poses)
            duration: Time to reach pose
        """
        if pose_name not in self.poses:
            self.get_logger().error(f'Unknown pose: {pose_name}')
            self.get_logger().info(f'Available: {list(self.poses.keys())}')
            return

        self.send_trajectory(self.poses[pose_name], duration)
        self.get_logger().info(f'Going to pose: {pose_name}')


def main(args=None):
    """Entry point for the joint commander node."""
    rclpy.init(args=args)

    node = JointCommander()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
