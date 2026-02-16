#!/usr/bin/env python3
"""
Behavior Switcher Node

Chapter 5: Adaptive Robotics
Switches robot behavior based on sensor input with hysteresis for stability.

This node:
- Subscribes to LIDAR sensor data (/scan)
- Evaluates trigger rules against sensor readings
- Selects appropriate behavior based on active triggers
- Publishes velocity commands (/cmd_vel)

Usage:
    ros2 run adaptive_robotics behavior_switcher

With custom parameters:
    ros2 run adaptive_robotics behavior_switcher --ros-args \\
        -p activate_threshold:=0.4 \\
        -p deactivate_threshold:=0.6
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from typing import Dict, List, Optional


class HysteresisThreshold:
    """
    Implements hysteresis for stable threshold crossing.

    Uses two thresholds to prevent oscillation:
    - activate_threshold: Value that triggers activation
    - deactivate_threshold: Value that triggers deactivation

    The gap between them is the "dead band" where no switching occurs.

    Example:
        >>> trigger = HysteresisThreshold(activate=0.5, deactivate=0.7)
        >>> trigger.update(0.4)  # Below activate
        True
        >>> trigger.update(0.6)  # In dead band, stays active
        True
        >>> trigger.update(0.8)  # Above deactivate
        False
    """

    def __init__(self, activate: float, deactivate: float):
        """
        Initialize hysteresis thresholds.

        Args:
            activate: Threshold to trigger activation (lower bound)
            deactivate: Threshold to trigger deactivation (upper bound)

        Raises:
            ValueError: If activate >= deactivate
        """
        if activate >= deactivate:
            raise ValueError(
                f"activate ({activate}) must be less than deactivate ({deactivate})"
            )
        self.activate = activate
        self.deactivate = deactivate
        self.is_active = False

    def update(self, value: float) -> bool:
        """
        Update state based on new sensor value.

        Args:
            value: Current sensor reading

        Returns:
            True if trigger is active, False otherwise
        """
        if not self.is_active and value < self.activate:
            # Below activate threshold - trigger ON
            self.is_active = True
        elif self.is_active and value > self.deactivate:
            # Above deactivate threshold - trigger OFF
            self.is_active = False
        # In dead band - no change

        return self.is_active

    def reset(self):
        """Reset to initial inactive state."""
        self.is_active = False

    def __repr__(self) -> str:
        return (
            f"HysteresisThreshold(activate={self.activate}, "
            f"deactivate={self.deactivate}, is_active={self.is_active})"
        )


class BehaviorSwitcher(Node):
    """
    ROS 2 node that switches robot behaviors based on sensor input.

    Behaviors:
        - idle: Robot stopped
        - explore: Moving forward
        - avoid: Turning away from obstacle
        - backup: Reversing

    Topics:
        - Subscribes: /scan (sensor_msgs/LaserScan)
        - Publishes: /cmd_vel (geometry_msgs/Twist)

    Parameters:
        - default_behavior (str): Behavior when no triggers active
        - activate_threshold (float): Distance to trigger avoid (meters)
        - deactivate_threshold (float): Distance to return to explore (meters)
        - decision_rate (float): How often to evaluate triggers (Hz)
    """

    def __init__(self):
        super().__init__('behavior_switcher')

        # Declare parameters with defaults
        self.declare_parameter('default_behavior', 'explore')
        self.declare_parameter('activate_threshold', 0.5)
        self.declare_parameter('deactivate_threshold', 0.7)
        self.declare_parameter('decision_rate', 10.0)
        self.declare_parameter('emergency_threshold', 0.2)

        # Get parameters
        self.default_behavior = self.get_parameter('default_behavior').value
        activate = self.get_parameter('activate_threshold').value
        deactivate = self.get_parameter('deactivate_threshold').value
        emergency = self.get_parameter('emergency_threshold').value

        # Initialize state
        self.current_behavior = self.default_behavior
        self.previous_behavior = self.default_behavior
        self.latest_distance = float('inf')
        self.decision_count = 0

        # Initialize triggers with hysteresis
        self.obstacle_trigger = HysteresisThreshold(activate, deactivate)
        self.emergency_trigger = HysteresisThreshold(emergency, activate)

        # Define behaviors
        self.behaviors: Dict[str, Dict[str, float]] = {
            'idle': {
                'linear_x': 0.0,
                'angular_z': 0.0,
                'description': 'Robot stopped'
            },
            'explore': {
                'linear_x': 0.2,
                'angular_z': 0.0,
                'description': 'Moving forward'
            },
            'avoid': {
                'linear_x': 0.0,
                'angular_z': 0.5,
                'description': 'Turning away from obstacle'
            },
            'backup': {
                'linear_x': -0.1,
                'angular_z': 0.0,
                'description': 'Reversing'
            }
        }

        # Create subscriber for LIDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Create publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create timer for decision loop
        rate = self.get_parameter('decision_rate').value
        self.timer = self.create_timer(1.0 / rate, self.decision_loop)

        self.get_logger().info(
            f'Behavior Switcher started.\n'
            f'  Default behavior: {self.default_behavior}\n'
            f'  Obstacle trigger: activate={activate}m, deactivate={deactivate}m\n'
            f'  Emergency trigger: activate={emergency}m'
        )

    def scan_callback(self, msg: LaserScan):
        """
        Process incoming LIDAR scan data.

        Extracts the minimum distance in the front 60-degree arc
        of the robot (30 degrees each side of center).

        Args:
            msg: LaserScan message from /scan topic
        """
        # TurtleBot3 LIDAR has 360 readings, index 0 is directly ahead
        # Check front 60 degrees: -30 to +30 degrees
        front_left = list(msg.ranges[0:30])
        front_right = list(msg.ranges[330:360])
        front_ranges = front_left + front_right

        # Filter out invalid readings
        valid_ranges = [
            r for r in front_ranges
            if r > msg.range_min and r < msg.range_max
        ]

        if valid_ranges:
            self.latest_distance = min(valid_ranges)
        else:
            # No valid readings - assume clear
            self.latest_distance = float('inf')

    def decision_loop(self):
        """
        Main decision loop - evaluate triggers and execute behavior.

        Called at the decision_rate frequency (default 10 Hz).
        Evaluates triggers in priority order and selects behavior.
        """
        self.decision_count += 1
        self.previous_behavior = self.current_behavior

        # Evaluate triggers in priority order (highest first)
        # Priority 1: Emergency (very close obstacle)
        if self.emergency_trigger.update(self.latest_distance):
            self.current_behavior = 'backup'
        # Priority 2: Obstacle avoidance
        elif self.obstacle_trigger.update(self.latest_distance):
            self.current_behavior = 'avoid'
        # Priority 3: Default behavior
        else:
            self.current_behavior = self.default_behavior

        # Log behavior change
        if self.current_behavior != self.previous_behavior:
            self.get_logger().info(
                f'[{self.decision_count}] Behavior switch: '
                f'{self.previous_behavior} -> {self.current_behavior} '
                f'(distance: {self.latest_distance:.3f}m)'
            )

        # Execute current behavior
        self.execute_behavior()

    def execute_behavior(self):
        """
        Publish velocity command for current behavior.

        Looks up the current behavior in the behaviors dict and
        publishes the corresponding Twist message to /cmd_vel.
        """
        behavior = self.behaviors.get(
            self.current_behavior,
            self.behaviors['idle']
        )

        cmd = Twist()
        cmd.linear.x = behavior['linear_x']
        cmd.angular.z = behavior['angular_z']

        self.cmd_pub.publish(cmd)

    def get_status(self) -> Dict:
        """
        Get current status of the behavior switcher.

        Returns:
            Dictionary with current state information
        """
        return {
            'current_behavior': self.current_behavior,
            'latest_distance': self.latest_distance,
            'decision_count': self.decision_count,
            'obstacle_trigger_active': self.obstacle_trigger.is_active,
            'emergency_trigger_active': self.emergency_trigger.is_active
        }


def main(args=None):
    """Main entry point for behavior switcher node."""
    rclpy.init(args=args)
    node = BehaviorSwitcher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down behavior switcher...')
    finally:
        # Stop robot before shutting down
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.get_logger().info('Robot stopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
