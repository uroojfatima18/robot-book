---
id: i3_launch_pipeline_startup
title: Launch Files and Pipeline Startup
tier: intermediate
chapter: chapter_4_workflow
estimated_time: 60-90 minutes
prerequisites: ["i1_ros2_state_machines", "i2_multi_node_coordination"]
---

# I3: Launch Files and Pipeline Startup

## Learning Objectives

By the end of this lesson, you will be able to:
- Create complex ROS 2 launch files for multi-node workflows
- Manage node dependencies and startup order
- Pass parameters to nodes via launch files
- Implement conditional node launching
- Debug launch file issues

## Introduction

Launch files are essential for starting complex robotic workflows. Instead of manually starting each node in separate terminals, a single launch file can orchestrate the entire system startup with proper dependencies, parameters, and error handling.

In this lesson, you'll learn to create production-ready launch files that start complete workflows with a single command.

## Basic Launch File Structure

Let's start with a simple launch file:

### Code Example: Basic Launch File

```python
# basic_workflow_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for basic workflow"""

    # Define nodes
    waypoint_manager = Node(
        package='workflow_orchestration',
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen'
    )

    navigation_controller = Node(
        package='workflow_orchestration',
        executable='navigation_controller',
        name='navigation_controller',
        output='screen'
    )

    status_monitor = Node(
        package='workflow_orchestration',
        executable='status_monitor',
        name='status_monitor',
        output='screen'
    )

    return LaunchDescription([
        waypoint_manager,
        navigation_controller,
        status_monitor,
    ])
```

**To run:**
```bash
ros2 launch workflow_orchestration basic_workflow_launch.py
```

## Launch File with Parameters

Pass parameters to nodes:

### Code Example: Parameterized Launch File

```python
# parameterized_workflow_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Launch workflow with configurable parameters"""

    # Declare launch arguments
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value='',
        description='Path to waypoint YAML file'
    )

    loop_waypoints_arg = DeclareLaunchArgument(
        'loop_waypoints',
        default_value='false',
        description='Whether to loop through waypoints'
    )

    # Get launch configurations
    waypoint_file = LaunchConfiguration('waypoint_file')
    loop_waypoints = LaunchConfiguration('loop_waypoints')

    # Waypoint manager with parameters
    waypoint_manager = Node(
        package='workflow_orchestration',
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen',
        parameters=[{
            'waypoint_file': waypoint_file,
            'loop_waypoints': loop_waypoints,
        }]
    )

    # Navigation controller
    navigation_controller = Node(
        package='workflow_orchestration',
        executable='navigation_controller',
        name='navigation_controller',
        output='screen'
    )

    # Status monitor
    status_monitor = Node(
        package='workflow_orchestration',
        executable='status_monitor',
        name='status_monitor',
        output='screen'
    )

    return LaunchDescription([
        waypoint_file_arg,
        loop_waypoints_arg,
        waypoint_manager,
        navigation_controller,
        status_monitor,
    ])
```

**To run with parameters:**
```bash
# With default parameters
ros2 launch workflow_orchestration parameterized_workflow_launch.py

# With custom parameters
ros2 launch workflow_orchestration parameterized_workflow_launch.py \
    waypoint_file:=/path/to/waypoints.yaml \
    loop_waypoints:=true
```

## Launch File with YAML Configuration

Load parameters from YAML files:

### Code Example: YAML Configuration

**config/workflow_params.yaml:**
```yaml
waypoint_manager:
  ros__parameters:
    waypoint_file: ""
    loop_waypoints: false
    publish_rate: 1.0

navigation_controller:
  ros__parameters:
    timeout_seconds: 60.0
    retry_attempts: 3

status_monitor:
  ros__parameters:
    update_rate: 5.0
    log_level: "info"
```

**Launch file:**
```python
# yaml_config_launch.py
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Launch workflow with YAML configuration"""

    # Get path to config file
    config_file = PathJoinSubstitution([
        FindPackageShare('workflow_orchestration'),
        'config',
        'workflow_params.yaml'
    ])

    # Nodes with YAML config
    waypoint_manager = Node(
        package='workflow_orchestration',
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen',
        parameters=[config_file]
    )

    navigation_controller = Node(
        package='workflow_orchestration',
        executable='navigation_controller',
        name='navigation_controller',
        output='screen',
        parameters=[config_file]
    )

    status_monitor = Node(
        package='workflow_orchestration',
        executable='status_monitor',
        name='status_monitor',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        waypoint_manager,
        navigation_controller,
        status_monitor,
    ])
```

## Complete Workflow Launch File

Integrate with Nav2 and Gazebo:

### Code Example: Complete Workflow Launch

```python
# complete_workflow_launch.py
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Complete workflow with simulation and navigation"""

    # Declare arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Whether to use simulation'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_world',
        description='Gazebo world to load'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map file'
    )

    # Get configurations
    use_sim = LaunchConfiguration('use_sim')
    world = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')

    # Include Gazebo launch (conditional)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world,
        }.items(),
        condition=IfCondition(use_sim)
    )

    # Include Nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim,
        }.items()
    )

    # Workflow nodes (delayed to allow Nav2 to start)
    waypoint_manager = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='Starting waypoint manager...'),
            Node(
                package='workflow_orchestration',
                executable='waypoint_manager',
                name='waypoint_manager',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim,
                }]
            )
        ]
    )

    navigation_controller = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='Starting navigation controller...'),
            Node(
                package='workflow_orchestration',
                executable='navigation_controller',
                name='navigation_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim,
                }]
            )
        ]
    )

    status_monitor = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='Starting status monitor...'),
            Node(
                package='workflow_orchestration',
                executable='status_monitor',
                name='status_monitor',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim,
                }]
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        use_sim_arg,
        world_arg,
        map_arg,

        # Launch files
        gazebo_launch,
        nav2_launch,

        # Workflow nodes (delayed)
        waypoint_manager,
        navigation_controller,
        status_monitor,
    ])
```

**To run:**
```bash
# With simulation
ros2 launch workflow_orchestration complete_workflow_launch.py \
    map:=/path/to/map.yaml

# Without simulation (real robot)
ros2 launch workflow_orchestration complete_workflow_launch.py \
    use_sim:=false \
    map:=/path/to/map.yaml
```

## Conditional Node Launching

Launch nodes based on conditions:

### Code Example: Conditional Launching

```python
# conditional_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch with conditional nodes"""

    # Declare arguments
    enable_monitoring_arg = DeclareLaunchArgument(
        'enable_monitoring',
        default_value='true',
        description='Enable status monitoring'
    )

    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='false',
        description='Enable RViz visualization'
    )

    # Get configurations
    enable_monitoring = LaunchConfiguration('enable_monitoring')
    enable_visualization = LaunchConfiguration('enable_visualization')

    # Always-on nodes
    waypoint_manager = Node(
        package='workflow_orchestration',
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen'
    )

    navigation_controller = Node(
        package='workflow_orchestration',
        executable='navigation_controller',
        name='navigation_controller',
        output='screen'
    )

    # Conditional nodes
    status_monitor = Node(
        package='workflow_orchestration',
        executable='status_monitor',
        name='status_monitor',
        output='screen',
        condition=IfCondition(enable_monitoring)
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/path/to/config.rviz'],
        condition=IfCondition(enable_visualization)
    )

    return LaunchDescription([
        enable_monitoring_arg,
        enable_visualization_arg,
        waypoint_manager,
        navigation_controller,
        status_monitor,
        rviz,
    ])
```

## Node Namespacing

Run multiple instances with namespaces:

### Code Example: Namespaced Nodes

```python
# multi_robot_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch multiple robot workflows with namespaces"""

    # Robot 1 workflow
    robot1_waypoint_manager = Node(
        package='workflow_orchestration',
        executable='waypoint_manager',
        name='waypoint_manager',
        namespace='robot1',
        output='screen',
        parameters=[{
            'waypoint_file': '/path/to/robot1_waypoints.yaml',
        }]
    )

    robot1_controller = Node(
        package='workflow_orchestration',
        executable='navigation_controller',
        name='navigation_controller',
        namespace='robot1',
        output='screen'
    )

    # Robot 2 workflow
    robot2_waypoint_manager = Node(
        package='workflow_orchestration',
        executable='waypoint_manager',
        name='waypoint_manager',
        namespace='robot2',
        output='screen',
        parameters=[{
            'waypoint_file': '/path/to/robot2_waypoints.yaml',
        }]
    )

    robot2_controller = Node(
        package='workflow_orchestration',
        executable='navigation_controller',
        name='navigation_controller',
        namespace='robot2',
        output='screen'
    )

    return LaunchDescription([
        robot1_waypoint_manager,
        robot1_controller,
        robot2_waypoint_manager,
        robot2_controller,
    ])
```

## Debugging Launch Files

Common debugging techniques:

### 1. Check Syntax
```bash
# Validate launch file syntax
python3 your_launch_file.py
```

### 2. Verbose Output
```bash
# Run with verbose output
ros2 launch --debug workflow_orchestration your_launch_file.py
```

### 3. Check Node Status
```bash
# List running nodes
ros2 node list

# Check node info
ros2 node info /waypoint_manager
```

### 4. Monitor Logs
```bash
# View logs in real-time
ros2 run rqt_console rqt_console
```

## Best Practices

### 1. Launch File Organization
- One launch file per workflow
- Use includes for reusable components
- Keep launch files in `launch/` directory
- Name files descriptively: `<workflow>_launch.py`

### 2. Parameter Management
- Use YAML files for complex configurations
- Provide sensible defaults
- Document all parameters
- Validate parameter values in nodes

### 3. Startup Sequencing
- Use `TimerAction` for delayed starts
- Wait for dependencies before starting nodes
- Log startup progress
- Handle startup failures gracefully

### 4. Testing
- Test launch files in isolation
- Verify all nodes start successfully
- Check parameter passing
- Test conditional logic

## Hardware Notes

> **Simulation vs. Real Hardware**: Launch files work identically in simulation and on real hardware. Use the `use_sim_time` parameter to switch between simulation time and wall clock time. Always test launch files in simulation before deploying to real robots.

## Summary

- Launch files orchestrate complex multi-node workflows
- Use parameters and YAML files for configuration
- Implement conditional launching for flexibility
- Use namespaces for multi-robot systems
- Test thoroughly and handle startup failures

## Exercises

1. **Create Modular Launch File** (Intermediate)
   - Split workflow into multiple launch files
   - Use includes to compose complete system
   - Pass parameters between launch files
   - Acceptance Criteria: Can launch workflow with single command

2. **Add Health Checks** (Advanced)
   - Implement node health monitoring
   - Restart failed nodes automatically
   - Log health status
   - Acceptance Criteria: System recovers from node failures

## Next Steps

You've completed the Intermediate tier! You can now:
- Implement state machines in ROS 2
- Coordinate multiple nodes
- Create production-ready launch files

Continue to the [Advanced Tier](../advanced/README.md) to learn about production-ready fault tolerance, watchdogs, and continuous operation.
