#!/usr/bin/env python3
"""
Digital Twin Bridge Node - Chapter 2 Advanced

Bidirectional synchronization between physical and simulated robots.

Modes:
    - sim: Commands go to simulation only
    - live: Commands go to physical robot only
    - mirror: Physical state mirrors to simulation

Topics Subscribed:
    /hw/joint_states - Physical robot state
    /sim/joint_states - Simulation state
    /cmd/joint_trajectory - Incoming commands

Topics Published:
    /sim/joint_states_mirrored - Mirrored state (in mirror mode)
    /hw/joint_trajectory - Forwarded commands (in live mode)
    /bridge/latency - Latency metrics
    /bridge/status - Bridge status

Parameters:
    mode (str): Operation mode - 'sim', 'live', or 'mirror'
    latency_threshold_ms (float): Latency warning threshold (default: 50.0)
    sync_rate_hz (float): State synchronization rate (default: 100.0)

Author: Robot Book Chapter 2
License: Apache 2.0
"""

import threading
from enum import Enum
from dataclasses import dataclass
from typing import Optional
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64, String
from rcl_interfaces.msg import ParameterDescriptor


class BridgeMode(Enum):
    """Operation modes for the bridge."""
    SIM = 'sim'           # Commands to simulation only
    LIVE = 'live'         # Commands to physical robot
    MIRROR = 'mirror'     # Physical mirrors to simulation


@dataclass
class LatencyStats:
    """Latency measurement statistics."""
    current_ms: float = 0.0
    average_ms: float = 0.0
    max_ms: float = 0.0
    samples: int = 0
    threshold_violations: int = 0


class BridgeNode(Node):
    """
    Digital Twin Bridge Node.

    Handles bidirectional synchronization between physical robot
    and Gazebo simulation with latency monitoring.

    The bridge supports three operational modes:
    - SIM: All commands route to simulation for testing
    - LIVE: Commands route to physical robot (with safety checks)
    - MIRROR: Physical robot state mirrors to simulation for visualization

    Latency is monitored continuously with configurable threshold alerts.
    """

    def __init__(self):
        super().__init__('bridge_node')

        # ============================================
        # Parameters
        # ============================================

        self.declare_parameter(
            'mode', 'mirror',
            ParameterDescriptor(description='Operation mode: sim, live, mirror')
        )
        self.declare_parameter(
            'latency_threshold_ms', 50.0,
            ParameterDescriptor(description='Latency warning threshold in ms')
        )
        self.declare_parameter(
            'sync_rate_hz', 100.0,
            ParameterDescriptor(description='State synchronization rate')
        )
        self.declare_parameter(
            'max_joint_velocity', 2.0,
            ParameterDescriptor(description='Max joint velocity for safety (rad/s)')
        )
        self.declare_parameter(
            'max_joint_position', 3.14,
            ParameterDescriptor(description='Max joint position limit (rad)')
        )

        mode_str = self.get_parameter('mode').value
        self.mode = BridgeMode(mode_str)
        self.latency_threshold = self.get_parameter('latency_threshold_ms').value
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').value
        self.max_joint_position = self.get_parameter('max_joint_position').value

        # ============================================
        # State
        # ============================================

        self.latency_stats = LatencyStats()
        self.last_hw_msg: Optional[JointState] = None
        self.last_sim_msg: Optional[JointState] = None
        self.last_hw_time: float = 0.0
        self.last_sim_time: float = 0.0
        self.connected = {'hw': False, 'sim': False}

        self._lock = threading.Lock()

        # ============================================
        # QoS Profiles
        # ============================================

        # Reliable for commands - ensures delivery
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Best effort for high-frequency sensor data - prioritizes speed
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Callback group for parallel execution
        self.cb_group = ReentrantCallbackGroup()

        # ============================================
        # Subscribers
        # ============================================

        self.hw_joint_sub = self.create_subscription(
            JointState,
            '/hw/joint_states',
            self.hw_joint_callback,
            sensor_qos,
            callback_group=self.cb_group
        )

        self.sim_joint_sub = self.create_subscription(
            JointState,
            '/sim/joint_states',
            self.sim_joint_callback,
            sensor_qos,
            callback_group=self.cb_group
        )

        self.cmd_sub = self.create_subscription(
            JointTrajectory,
            '/cmd/joint_trajectory',
            self.cmd_callback,
            reliable_qos,
            callback_group=self.cb_group
        )

        # ============================================
        # Publishers
        # ============================================

        self.sim_joint_pub = self.create_publisher(
            JointState,
            '/sim/joint_states_mirrored',
            sensor_qos
        )

        self.hw_cmd_pub = self.create_publisher(
            JointTrajectory,
            '/hw/joint_trajectory',
            reliable_qos
        )

        self.sim_cmd_pub = self.create_publisher(
            JointTrajectory,
            '/sim/joint_trajectory',
            reliable_qos
        )

        self.latency_pub = self.create_publisher(
            Float64,
            '/bridge/latency',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/bridge/status',
            10
        )

        # ============================================
        # Timers
        # ============================================

        self.latency_timer = self.create_timer(
            0.1,  # 10 Hz
            self.publish_latency,
            callback_group=self.cb_group
        )

        self.watchdog_timer = self.create_timer(
            1.0,  # 1 Hz
            self.watchdog_check,
            callback_group=self.cb_group
        )

        self.status_timer = self.create_timer(
            0.5,  # 2 Hz
            self.publish_status,
            callback_group=self.cb_group
        )

        # ============================================
        # Startup
        # ============================================

        self.get_logger().info(f'Bridge Node initialized in {self.mode.value} mode')
        self.get_logger().info(f'Latency threshold: {self.latency_threshold} ms')

    # ============================================
    # Callbacks
    # ============================================

    def hw_joint_callback(self, msg: JointState):
        """Handle physical robot joint states."""
        with self._lock:
            self.last_hw_msg = msg
            self.last_hw_time = time.time()
            self.connected['hw'] = True

        # Calculate latency
        latency = self._calculate_latency(msg)
        self._update_latency_stats(latency)

        # Mirror to simulation if in mirror mode
        if self.mode == BridgeMode.MIRROR:
            self.sim_joint_pub.publish(msg)

    def sim_joint_callback(self, msg: JointState):
        """Handle simulation joint states."""
        with self._lock:
            self.last_sim_msg = msg
            self.last_sim_time = time.time()
            self.connected['sim'] = True

    def cmd_callback(self, msg: JointTrajectory):
        """Handle incoming joint trajectory commands."""
        # Safety validation
        if not self._validate_command(msg):
            self.get_logger().warn('Command rejected by safety check')
            return

        # Route based on mode
        if self.mode == BridgeMode.SIM:
            self.sim_cmd_pub.publish(msg)
            self.get_logger().debug('Command routed to simulation')

        elif self.mode == BridgeMode.LIVE:
            if self._pre_live_check():
                self.hw_cmd_pub.publish(msg)
                self.get_logger().debug('Command routed to hardware')
            else:
                self.get_logger().warn('Live command blocked - safety check failed')

        elif self.mode == BridgeMode.MIRROR:
            # In mirror mode, commands go to simulation for testing
            self.sim_cmd_pub.publish(msg)
            self.get_logger().debug('Command routed to simulation (mirror mode)')

    # ============================================
    # Latency Monitoring
    # ============================================

    def _calculate_latency(self, msg: JointState) -> float:
        """Calculate message latency in milliseconds."""
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return 0.0

        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9

        latency_ms = (now - msg_time) * 1000
        return max(0.0, latency_ms)  # Clamp negative values

    def _update_latency_stats(self, latency_ms: float):
        """Update running latency statistics."""
        with self._lock:
            self.latency_stats.current_ms = latency_ms
            self.latency_stats.samples += 1

            # Running average
            n = self.latency_stats.samples
            self.latency_stats.average_ms = (
                (self.latency_stats.average_ms * (n - 1) + latency_ms) / n
            )

            # Track maximum
            if latency_ms > self.latency_stats.max_ms:
                self.latency_stats.max_ms = latency_ms

            # Track threshold violations
            if latency_ms > self.latency_threshold:
                self.latency_stats.threshold_violations += 1
                self.get_logger().warn(
                    f'Latency threshold exceeded: {latency_ms:.1f}ms > {self.latency_threshold}ms'
                )

    def publish_latency(self):
        """Publish current latency to monitoring topic."""
        msg = Float64()
        with self._lock:
            msg.data = self.latency_stats.current_ms
        self.latency_pub.publish(msg)

    # ============================================
    # Safety Checks
    # ============================================

    def _validate_command(self, msg: JointTrajectory) -> bool:
        """Validate command before forwarding."""
        # Check for empty trajectory
        if not msg.points:
            self.get_logger().warn('Empty trajectory rejected')
            return False

        # Check joint names present
        if not msg.joint_names:
            self.get_logger().warn('No joint names in trajectory')
            return False

        # Check position limits
        for point in msg.points:
            for pos in point.positions:
                if abs(pos) > self.max_joint_position:
                    self.get_logger().warn(f'Position {pos} exceeds limit {self.max_joint_position}')
                    return False

            # Check velocity limits if provided
            if point.velocities:
                for vel in point.velocities:
                    if abs(vel) > self.max_joint_velocity:
                        self.get_logger().warn(f'Velocity {vel} exceeds limit {self.max_joint_velocity}')
                        return False

        return True

    def _pre_live_check(self) -> bool:
        """Additional checks before sending to live robot."""
        with self._lock:
            # Check hardware connection
            if not self.connected['hw']:
                self.get_logger().error('Hardware not connected')
                return False

            # Check latency is acceptable
            if self.latency_stats.current_ms > self.latency_threshold * 2:
                self.get_logger().error('Latency too high for live control')
                return False

        return True

    # ============================================
    # Watchdog
    # ============================================

    def watchdog_check(self):
        """Check connection status periodically."""
        now = time.time()
        timeout = 2.0  # seconds

        with self._lock:
            # Check hardware timeout
            if self.connected['hw'] and (now - self.last_hw_time) > timeout:
                self.connected['hw'] = False
                self.get_logger().warn('Hardware connection lost')

            # Check simulation timeout
            if self.connected['sim'] and (now - self.last_sim_time) > timeout:
                self.connected['sim'] = False
                self.get_logger().warn('Simulation connection lost')

    def publish_status(self):
        """Publish bridge status."""
        with self._lock:
            status = {
                'mode': self.mode.value,
                'hw_connected': self.connected['hw'],
                'sim_connected': self.connected['sim'],
                'latency_ms': round(self.latency_stats.current_ms, 2),
                'latency_avg_ms': round(self.latency_stats.average_ms, 2),
                'violations': self.latency_stats.threshold_violations
            }

        msg = String()
        msg.data = str(status)
        self.status_pub.publish(msg)

    # ============================================
    # Mode Control
    # ============================================

    def set_mode(self, mode: str) -> bool:
        """Change operation mode."""
        try:
            new_mode = BridgeMode(mode)
            self.mode = new_mode
            self.get_logger().info(f'Mode changed to: {mode}')
            return True
        except ValueError:
            self.get_logger().error(f'Invalid mode: {mode}')
            return False

    def get_status(self) -> dict:
        """Get current bridge status."""
        with self._lock:
            return {
                'mode': self.mode.value,
                'hw_connected': self.connected['hw'],
                'sim_connected': self.connected['sim'],
                'latency_current_ms': self.latency_stats.current_ms,
                'latency_average_ms': self.latency_stats.average_ms,
                'latency_max_ms': self.latency_stats.max_ms,
                'samples': self.latency_stats.samples,
                'threshold_violations': self.latency_stats.threshold_violations
            }


def main(args=None):
    """Entry point for the bridge node."""
    rclpy.init(args=args)

    node = BridgeNode()

    # Use multi-threaded executor for parallel callbacks
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
