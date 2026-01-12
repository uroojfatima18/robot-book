#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Physical AI & Humanoid Robotics Textbook
#
# Chapter 1: The Robotic Nervous System (ROS 2)
# Lesson: I2 - Python ROS Bridge (rclpy)
# Example: Launch file for talker/listener demo

"""
ROS 2 Launch File Example - Talker/Listener with Parameters

This launch file demonstrates:
1. Launching multiple nodes
2. Declaring and using launch arguments
3. Setting node parameters
4. Topic remapping
5. Namespace configuration

Usage:
    # Basic usage (uses defaults)
    ros2 launch <package_name> talker_listener_launch.py

    # With custom arguments
    ros2 launch <package_name> talker_listener_launch.py rate:=5.0 prefix:=robot1

    # With simulation mode disabled
    ros2 launch <package_name> talker_listener_launch.py use_sim:=false

    # View all available arguments
    ros2 launch <package_name> talker_listener_launch.py --show-args

Note: To use this launch file, it must be installed as part of a ROS 2 package.
For standalone testing, you can run the Python file directly:
    python3 talker_listener_launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate the launch description for talker/listener demo.

    Returns:
        LaunchDescription: The complete launch description
    """

    # ========== DECLARE LAUNCH ARGUMENTS ==========

    # Publishing rate argument
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='2.0',
        description='Publishing rate in Hz for the talker node'
    )

    # Namespace prefix argument
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Namespace prefix for all nodes (e.g., "robot1")'
    )

    # Topic name argument
    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='chatter',
        description='Topic name for communication'
    )

    # Simulation mode argument
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Whether to use simulation time'
    )

    # ========== LOG CONFIGURATION ==========

    log_config = LogInfo(
        msg=[
            'Launching talker/listener with: ',
            'rate=', LaunchConfiguration('rate'),
            ', prefix=', LaunchConfiguration('prefix'),
            ', topic=', LaunchConfiguration('topic')
        ]
    )

    # ========== NODE DEFINITIONS ==========

    # Talker node - publishes messages
    talker_node = Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='talker',
        namespace=LaunchConfiguration('prefix'),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim')
        }],
        remappings=[
            # Remap default 'chatter' topic to configured topic
            ('chatter', LaunchConfiguration('topic'))
        ],
        output='screen',
        emulate_tty=True,  # Enables colored output
    )

    # Listener node - subscribes to messages
    listener_node = Node(
        package='demo_nodes_cpp',
        executable='listener',
        name='listener',
        namespace=LaunchConfiguration('prefix'),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim')
        }],
        remappings=[
            ('chatter', LaunchConfiguration('topic'))
        ],
        output='screen',
        emulate_tty=True,
    )

    # ========== BUILD LAUNCH DESCRIPTION ==========

    return LaunchDescription([
        # Declare all arguments first
        rate_arg,
        prefix_arg,
        topic_arg,
        use_sim_arg,

        # Log the configuration
        log_config,

        # Launch the nodes
        talker_node,
        listener_node,
    ])


# Allow running this file directly for testing
if __name__ == '__main__':
    # This allows testing the launch file structure
    ld = generate_launch_description()
    print("Launch description generated successfully!")
    print(f"Contains {len(ld.entities)} entities")

    # List the entities
    for i, entity in enumerate(ld.entities):
        print(f"  {i+1}. {type(entity).__name__}")
