#!/usr/bin/env python3
"""
Latency Monitor - Track and visualize bridge latency.

Subscribes to /bridge/latency and provides:
- Console output with warnings
- Latency history for analysis
- Threshold alerts with configurable severity

Topics Subscribed:
    /bridge/latency - Latency measurements from bridge node

Parameters:
    threshold_ms (float): Warning threshold in milliseconds (default: 50.0)
    window_size (int): Rolling window size for statistics (default: 100)
    report_interval (float): Seconds between stats reports (default: 5.0)
    alert_on_consecutive (int): Alert after N consecutive violations (default: 3)

Author: Robot Book Chapter 2
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from rcl_interfaces.msg import ParameterDescriptor
from collections import deque
import statistics
import time


class LatencyMonitor(Node):
    """
    Monitor and analyze bridge latency.

    This node provides real-time latency monitoring with:
    - Rolling statistics (mean, std, min, max)
    - Threshold violation alerts
    - Consecutive violation detection
    - Periodic reporting

    Use this node to diagnose digital twin synchronization issues
    and ensure latency stays within acceptable bounds (<50ms default).
    """

    def __init__(self):
        super().__init__('latency_monitor')

        # ============================================
        # Parameters
        # ============================================

        self.declare_parameter(
            'threshold_ms', 50.0,
            ParameterDescriptor(description='Latency warning threshold in ms')
        )
        self.declare_parameter(
            'window_size', 100,
            ParameterDescriptor(description='Rolling window size for statistics')
        )
        self.declare_parameter(
            'report_interval', 5.0,
            ParameterDescriptor(description='Seconds between stats reports')
        )
        self.declare_parameter(
            'alert_on_consecutive', 3,
            ParameterDescriptor(description='Alert after N consecutive violations')
        )

        self.threshold = self.get_parameter('threshold_ms').value
        window_size = self.get_parameter('window_size').value
        report_interval = self.get_parameter('report_interval').value
        self.alert_threshold = self.get_parameter('alert_on_consecutive').value

        # ============================================
        # State
        # ============================================

        # History buffer for rolling statistics
        self.history = deque(maxlen=window_size)

        # Tracking counters
        self.total_samples = 0
        self.violation_count = 0
        self.consecutive_violations = 0
        self.max_consecutive = 0

        # Time tracking
        self.start_time = time.time()
        self.last_violation_time = 0.0

        # Alert state
        self.in_alert_state = False

        # ============================================
        # Subscribers
        # ============================================

        self.latency_sub = self.create_subscription(
            Float64,
            '/bridge/latency',
            self.latency_callback,
            10
        )

        # ============================================
        # Publishers
        # ============================================

        self.alert_pub = self.create_publisher(
            String,
            '/bridge/latency_alert',
            10
        )

        self.stats_pub = self.create_publisher(
            String,
            '/bridge/latency_stats',
            10
        )

        # ============================================
        # Timers
        # ============================================

        self.timer = self.create_timer(report_interval, self.report_stats)

        # ============================================
        # Startup
        # ============================================

        self.get_logger().info(
            f'Latency Monitor started (threshold: {self.threshold}ms, '
            f'window: {window_size}, alert after: {self.alert_threshold} consecutive)'
        )

    def latency_callback(self, msg: Float64):
        """Process latency measurement."""
        latency = msg.data
        self.history.append(latency)
        self.total_samples += 1

        # Check for threshold violation
        if latency > self.threshold:
            self.violation_count += 1
            self.consecutive_violations += 1
            self.last_violation_time = time.time()

            # Update max consecutive
            if self.consecutive_violations > self.max_consecutive:
                self.max_consecutive = self.consecutive_violations

            # Log warning
            self.get_logger().warn(
                f'LATENCY ALERT: {latency:.1f}ms > {self.threshold}ms '
                f'(consecutive: {self.consecutive_violations}, '
                f'total violations: {self.violation_count})'
            )

            # Check for sustained violation alert
            if self.consecutive_violations >= self.alert_threshold:
                self._trigger_alert(latency)
        else:
            # Reset consecutive counter on good reading
            if self.consecutive_violations > 0:
                self.get_logger().info(
                    f'Latency recovered: {latency:.1f}ms '
                    f'(was {self.consecutive_violations} consecutive violations)'
                )
            self.consecutive_violations = 0

            # Clear alert state if applicable
            if self.in_alert_state:
                self._clear_alert(latency)

    def _trigger_alert(self, latency: float):
        """Trigger sustained latency alert."""
        if not self.in_alert_state:
            self.in_alert_state = True

            alert_msg = String()
            alert_msg.data = (
                f'SUSTAINED_HIGH_LATENCY: {latency:.1f}ms for '
                f'{self.consecutive_violations} consecutive samples'
            )
            self.alert_pub.publish(alert_msg)

            self.get_logger().error(
                f'ALERT: Sustained high latency detected! '
                f'{self.consecutive_violations} consecutive violations. '
                f'Current: {latency:.1f}ms, Threshold: {self.threshold}ms'
            )

    def _clear_alert(self, latency: float):
        """Clear the alert state."""
        self.in_alert_state = False

        alert_msg = String()
        alert_msg.data = f'LATENCY_RECOVERED: {latency:.1f}ms'
        self.alert_pub.publish(alert_msg)

        self.get_logger().info('Alert cleared: Latency returned to acceptable levels')

    def report_stats(self):
        """Report latency statistics."""
        if not self.history:
            self.get_logger().info('No latency data yet')
            return

        # Calculate statistics
        avg = statistics.mean(self.history)
        std = statistics.stdev(self.history) if len(self.history) > 1 else 0
        min_val = min(self.history)
        max_val = max(self.history)

        # Calculate percentiles
        sorted_history = sorted(self.history)
        p95_idx = int(len(sorted_history) * 0.95)
        p99_idx = int(len(sorted_history) * 0.99)
        p95 = sorted_history[p95_idx] if p95_idx < len(sorted_history) else max_val
        p99 = sorted_history[p99_idx] if p99_idx < len(sorted_history) else max_val

        # Calculate violation rate
        violation_rate = (self.violation_count / self.total_samples * 100) if self.total_samples > 0 else 0

        # Runtime
        runtime = time.time() - self.start_time

        # Log report
        self.get_logger().info(
            f'Latency Stats [{runtime:.0f}s]: '
            f'avg={avg:.1f}ms, std={std:.1f}ms, '
            f'min={min_val:.1f}ms, max={max_val:.1f}ms, '
            f'p95={p95:.1f}ms, p99={p99:.1f}ms, '
            f'violations={self.violation_count} ({violation_rate:.1f}%)'
        )

        # Publish stats
        stats_msg = String()
        stats_msg.data = str({
            'average_ms': round(avg, 2),
            'std_ms': round(std, 2),
            'min_ms': round(min_val, 2),
            'max_ms': round(max_val, 2),
            'p95_ms': round(p95, 2),
            'p99_ms': round(p99, 2),
            'samples': self.total_samples,
            'violations': self.violation_count,
            'violation_rate_pct': round(violation_rate, 2),
            'max_consecutive': self.max_consecutive,
            'runtime_sec': round(runtime, 1)
        })
        self.stats_pub.publish(stats_msg)

    def get_summary(self) -> dict:
        """Get summary of latency monitoring session."""
        if not self.history:
            return {'status': 'no_data'}

        avg = statistics.mean(self.history)
        runtime = time.time() - self.start_time
        violation_rate = (self.violation_count / self.total_samples * 100) if self.total_samples > 0 else 0

        return {
            'runtime_sec': runtime,
            'total_samples': self.total_samples,
            'average_latency_ms': avg,
            'max_latency_ms': max(self.history),
            'violations': self.violation_count,
            'violation_rate_pct': violation_rate,
            'max_consecutive_violations': self.max_consecutive,
            'meets_threshold': violation_rate < 5  # <5% violations is acceptable
        }


def main(args=None):
    """Entry point for the latency monitor."""
    rclpy.init(args=args)
    node = LatencyMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Print summary on exit
        summary = node.get_summary()
        node.get_logger().info(f'Final Summary: {summary}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
