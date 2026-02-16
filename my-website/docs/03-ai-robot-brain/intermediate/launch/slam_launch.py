#!/usr/bin/env python3
"""
slam_launch.py

Launch file for SLAM Toolbox with Gazebo simulation.
Configures SLAM for indoor mapping with TurtleBot3.

Usage:
  ros2 launch <package_name> slam_launch.py
  ros2 launch <package_name> slam_launch.py use_sim_time:=false

Dependencies: slam_toolbox, nav2_lifecycle_manager
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # SLAM Toolbox parameters
    slam_params = {
        'use_sim_time': use_sim_time,
        'solver_plugin': 'solver_plugins::CeresSolver',
        'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
        'ceres_preconditioner': 'SCHUR_JACOBI',

        # Map settings
        'resolution': 0.05,
        'max_laser_range': 12.0,
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_frame': 'base_footprint',

        # Update thresholds
        'minimum_travel_distance': 0.3,
        'minimum_travel_heading': 0.3,

        # Scan matching
        'scan_topic': '/scan',
        'mode': 'mapping',

        # Loop closure
        'do_loop_closing': True,
        'loop_search_maximum_distance': 3.0,

        # Processing
        'transform_publish_period': 0.02,
        'map_update_interval': 2.0,
    }

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        slam_toolbox_node,
    ])
