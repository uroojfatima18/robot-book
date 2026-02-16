#!/usr/bin/env python3
"""
Spawn Humanoid Launch File - Chapter 2 Intermediate

Launches Gazebo with a world file and spawns a humanoid robot model.

Usage:
    ros2 launch your_package spawn_humanoid.launch.py
    ros2 launch your_package spawn_humanoid.launch.py world:=custom.world z:=1.5

Arguments:
    world: World file to load (default: simple_lab.world)
    urdf:  Path to robot URDF file
    x, y, z: Initial spawn position
    yaw: Initial rotation around Z axis
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for humanoid spawn."""

    # Package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # ============================================
    # Launch Arguments
    # ============================================

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_lab.world',
        description='Name of the world file to load'
    )

    urdf_arg = DeclareLaunchArgument(
        'urdf',
        default_value='',
        description='Path to robot URDF file (required)'
    )

    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='X position for spawning'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Y position for spawning'
    )

    z_arg = DeclareLaunchArgument(
        'z',
        default_value='1.0',
        description='Z position for spawning (height)'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Yaw rotation for spawning (radians)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # ============================================
    # Gazebo Launch
    # ============================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true',
        }.items()
    )

    # ============================================
    # Robot State Publisher
    # ============================================

    # Read URDF content using xacro (handles both .urdf and .xacro)
    robot_description_content = Command([
        'xacro ',
        LaunchConfiguration('urdf')
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ============================================
    # Spawn Entity
    # ============================================

    # Delay spawn to ensure Gazebo is ready
    spawn_entity = TimerAction(
        period=3.0,  # Wait 3 seconds for Gazebo to initialize
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_humanoid',
                output='screen',
                arguments=[
                    '-entity', 'humanoid',
                    '-topic', '/robot_description',
                    '-x', LaunchConfiguration('x'),
                    '-y', LaunchConfiguration('y'),
                    '-z', LaunchConfiguration('z'),
                    '-Y', LaunchConfiguration('yaw'),
                ]
            )
        ]
    )

    # ============================================
    # Joint State Broadcaster (for ros2_control)
    # ============================================

    # Optional: Load controllers if using ros2_control
    # Uncomment and configure as needed
    #
    # load_joint_state_broadcaster = TimerAction(
    #     period=5.0,
    #     actions=[
    #         Node(
    #             package='controller_manager',
    #             executable='spawner',
    #             arguments=['joint_state_broadcaster'],
    #             output='screen',
    #         )
    #     ]
    # )

    # ============================================
    # Return Launch Description
    # ============================================

    return LaunchDescription([
        # Arguments
        world_arg,
        urdf_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        use_sim_time_arg,

        # Nodes
        gazebo,
        robot_state_publisher,
        spawn_entity,

        # Optional: Controller loading
        # load_joint_state_broadcaster,
    ])
