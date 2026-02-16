#!/usr/bin/env python3
"""
navigation_launch.py

Launch file for Nav2 navigation stack.
Loads map and starts all navigation components.

Usage:
  ros2 launch <package_name> navigation_launch.py map:=/path/to/map.yaml
  ros2 launch <package_name> navigation_launch.py map:=/path/to/map.yaml use_sim_time:=false

Dependencies: nav2_bringup
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch arguments
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file')

    # Default Nav2 params file
    default_params = os.path.join(
        nav2_bringup_dir, 'params', 'nav2_params.yaml'
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'map',
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 parameters file'
        ),

        # Include Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items()
        ),
    ])
