#!/usr/bin/env python3
"""
Adaptive Robotics Demo Launch File

Chapter 5: Launches behavior switcher with TurtleBot3 for demonstration.

Usage:
    ros2 launch adaptive_robotics adaptive_demo.launch.py

With custom parameters:
    ros2 launch adaptive_robotics adaptive_demo.launch.py \\
        activate_threshold:=0.4 \\
        deactivate_threshold:=0.6
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for adaptive robotics demo."""

    # Declare launch arguments
    activate_threshold_arg = DeclareLaunchArgument(
        'activate_threshold',
        default_value='0.5',
        description='Distance (m) to activate avoid behavior'
    )

    deactivate_threshold_arg = DeclareLaunchArgument(
        'deactivate_threshold',
        default_value='0.7',
        description='Distance (m) to deactivate avoid behavior'
    )

    default_behavior_arg = DeclareLaunchArgument(
        'default_behavior',
        default_value='explore',
        description='Default behavior when no triggers active'
    )

    decision_rate_arg = DeclareLaunchArgument(
        'decision_rate',
        default_value='10.0',
        description='Decision loop rate in Hz'
    )

    # Log startup info
    startup_info = LogInfo(
        msg=[
            'Starting Adaptive Robotics Demo\n',
            '  Activate threshold: ', LaunchConfiguration('activate_threshold'), 'm\n',
            '  Deactivate threshold: ', LaunchConfiguration('deactivate_threshold'), 'm\n',
            '  Default behavior: ', LaunchConfiguration('default_behavior'), '\n',
            '  Decision rate: ', LaunchConfiguration('decision_rate'), ' Hz'
        ]
    )

    # Behavior switcher node
    behavior_switcher_node = Node(
        package='adaptive_robotics',
        executable='behavior_switcher',
        name='behavior_switcher',
        output='screen',
        parameters=[{
            'activate_threshold': LaunchConfiguration('activate_threshold'),
            'deactivate_threshold': LaunchConfiguration('deactivate_threshold'),
            'default_behavior': LaunchConfiguration('default_behavior'),
            'decision_rate': LaunchConfiguration('decision_rate'),
        }]
    )

    return LaunchDescription([
        # Launch arguments
        activate_threshold_arg,
        deactivate_threshold_arg,
        default_behavior_arg,
        decision_rate_arg,

        # Info
        startup_info,

        # Nodes
        behavior_switcher_node,
    ])
