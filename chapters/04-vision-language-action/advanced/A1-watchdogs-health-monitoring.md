---
id: a1_watchdogs_health_monitoring
title: Watchdogs and Health Monitoring
tier: advanced
chapter: chapter_4_workflow
estimated_time: 60-90 minutes
prerequisites: ["i1_ros2_state_machines", "i2_multi_node_coordination", "i3_launch_pipeline_startup"]
---

# A1: Watchdogs and Health Monitoring

## Learning Objectives

By the end of this lesson, you will be able to:
- Implement heartbeat-based watchdog patterns
- Detect node failures and communication timeouts
- Build automatic recovery mechanisms
- Monitor system health in real-time
- Handle graceful shutdowns and restarts

## Introduction

In production robotics, components can fail unexpectedly - nodes crash, sensors disconnect, networks drop packets. Watchdogs are essential for detecting these failures quickly and triggering recovery actions automatically.

In this lesson, you'll implement production-ready watchdog patterns that monitor node health, detect failures within seconds, and recover automatically without human intervention.

## Heartbeat Pattern

The heartbeat pattern is the foundation of watchdog monitoring:

### Code Example: Basic Heartbeat

```python
# heartbeat_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

class HeartbeatNode(Node):
    """Node that publishes periodic heartbeats"""

    def __init__(self):
        super().__init__('heartbeat_node')

        # Declare parameters
        self.declare_parameter('heartbeat_rate', 2.0)  # Hz

        # Get parameters
        rate = self.get_parameter('heartbeat_rate').value

        # Publisher
        self.heartbeat_pub = self.create_publisher(
            Header,
            '/heartbeat',
            10
        )

        # Timer for heartbeat
        self.timer = self.create_timer(1.0 / rate, self.publish_heartbeat)

        self.sequence = 0
        self.get_logger().info(f'Heartbeat node started at {rate} Hz')

    def publish_heartbeat(self):
        """Publish heartbeat message"""
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = self.get_name()

        self.heartbeat_pub.publish(msg)
        self.sequence += 1

        if self.sequence % 10 == 0:
            self.get_logger().info(
                f'Heartbeat #{self.sequence}',
                throttle_duration_sec=5.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Example: Watchdog Monitor

```python
# watchdog_monitor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String
from datetime import datetime, timedelta

class WatchdogMonitor(Node):
    """Monitors heartbeats and detects failures"""

    def __init__(self):
        super().__init__('watchdog_monitor')

        # Parameters
        self.declare_parameter('timeout_seconds', 5.0)
        self.declare_parameter('monitored_nodes', ['heartbeat_node'])

        self.timeout = self.get_parameter('timeout_seconds').value
        self.monitored_nodes = self.get_parameter('monitored_nodes').value

        # Track last heartbeat time for each node
        self.last_heartbeat = {}
        self.node_status = {}

        # Subscriber
        self.heartbeat_sub = self.create_subscription(
            Header,
            '/heartbeat',
            self.heartbeat_callback,
            10
        )

        # Publisher for alerts
        self.alert_pub = self.create_publisher(
            String,
            '/watchdog_alerts',
            10
        )

        # Timer to check for timeouts
        self.check_timer = self.create_timer(1.0, self.check_timeouts)

        self.get_logger().info(
            f'Watchdog monitoring {len(self.monitored_nodes)} nodes '
            f'with {self.timeout}s timeout'
        )

    def heartbeat_callback(self, msg):
        """Handle heartbeat message"""
        node_name = msg.frame_id
        current_time = datetime.now()

        # Update last heartbeat time
        self.last_heartbeat[node_name] = current_time

        # Update status if it was failed
        if self.node_status.get(node_name) == 'FAILED':
            self.get_logger().info(f'Node {node_name} recovered')
            self.node_status[node_name] = 'HEALTHY'

            # Publish recovery alert
            alert = String()
            alert.data = f'RECOVERED: {node_name}'
            self.alert_pub.publish(alert)

    def check_timeouts(self):
        """Check for timeout failures"""
        current_time = datetime.now()
        timeout_delta = timedelta(seconds=self.timeout)

        for node_name in self.monitored_nodes:
            if node_name not in self.last_heartbeat:
                # Never received heartbeat
                if self.node_status.get(node_name) != 'UNKNOWN':
                    self.get_logger().warn(
                        f'Node {node_name} never sent heartbeat'
                    )
                    self.node_status[node_name] = 'UNKNOWN'
                continue

            # Check if heartbeat is stale
            time_since_heartbeat = current_time - self.last_heartbeat[node_name]

            if time_since_heartbeat > timeout_delta:
                if self.node_status.get(node_name) != 'FAILED':
                    self.get_logger().error(
                        f'Node {node_name} FAILED - no heartbeat for '
                        f'{time_since_heartbeat.total_seconds():.1f}s'
                    )
                    self.node_status[node_name] = 'FAILED'

                    # Publish failure alert
                    alert = String()
                    alert.data = f'FAILED: {node_name}'
                    self.alert_pub.publish(alert)
            else:
                # Node is healthy
                if self.node_status.get(node_name) != 'HEALTHY':
                    self.node_status[node_name] = 'HEALTHY'

    def get_status_summary(self):
        """Get summary of all node statuses"""
        summary = {}
        for node in self.monitored_nodes:
            summary[node] = self.node_status.get(node, 'UNKNOWN')
        return summary

def main(args=None):
    rclpy.init(args=args)
    node = WatchdogMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Automatic Recovery

Implement automatic node restart on failure:

### Code Example: Recovery Manager

```python
# recovery_manager.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time

class RecoveryManager(Node):
    """Manages automatic recovery of failed nodes"""

    def __init__(self):
        super().__init__('recovery_manager')

        # Parameters
        self.declare_parameter('max_restart_attempts', 3)
        self.declare_parameter('restart_delay', 5.0)

        self.max_attempts = self.get_parameter('max_restart_attempts').value
        self.restart_delay = self.get_parameter('restart_delay').value

        # Track restart attempts
        self.restart_count = {}
        self.last_restart_time = {}

        # Subscribe to watchdog alerts
        self.alert_sub = self.create_subscription(
            String,
            '/watchdog_alerts',
            self.alert_callback,
            10
        )

        # Publisher for recovery status
        self.status_pub = self.create_publisher(
            String,
            '/recovery_status',
            10
        )

        self.get_logger().info('Recovery Manager initialized')

    def alert_callback(self, msg):
        """Handle watchdog alerts"""
        alert_type, node_name = msg.data.split(': ')

        if alert_type == 'FAILED':
            self.handle_node_failure(node_name)
        elif alert_type == 'RECOVERED':
            self.handle_node_recovery(node_name)

    def handle_node_failure(self, node_name):
        """Handle node failure with automatic restart"""
        self.get_logger().warn(f'Handling failure of {node_name}')

        # Check restart attempts
        attempts = self.restart_count.get(node_name, 0)

        if attempts >= self.max_attempts:
            self.get_logger().error(
                f'Max restart attempts ({self.max_attempts}) exceeded for {node_name}'
            )
            self.publish_status(f'GAVE_UP: {node_name}')
            return

        # Check if we need to wait before restarting
        last_restart = self.last_restart_time.get(node_name)
        if last_restart:
            time_since_restart = time.time() - last_restart
            if time_since_restart < self.restart_delay:
                self.get_logger().info(
                    f'Waiting {self.restart_delay - time_since_restart:.1f}s '
                    f'before restarting {node_name}'
                )
                return

        # Attempt restart
        self.restart_node(node_name)

    def restart_node(self, node_name):
        """Restart a failed node"""
        self.get_logger().info(f'Attempting to restart {node_name}')

        try:
            # This is a simplified example - in production, you'd use
            # proper process management (systemd, supervisor, etc.)
            subprocess.Popen([
                'ros2', 'run', 'workflow_orchestration', node_name
            ])

            # Update tracking
            self.restart_count[node_name] = self.restart_count.get(node_name, 0) + 1
            self.last_restart_time[node_name] = time.time()

            self.publish_status(
                f'RESTARTED: {node_name} (attempt {self.restart_count[node_name]})'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to restart {node_name}: {e}')
            self.publish_status(f'RESTART_FAILED: {node_name}')

    def handle_node_recovery(self, node_name):
        """Handle node recovery - reset restart counter"""
        self.get_logger().info(f'{node_name} recovered, resetting restart counter')
        self.restart_count[node_name] = 0
        self.publish_status(f'HEALTHY: {node_name}')

    def publish_status(self, status):
        """Publish recovery status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RecoveryManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Communication Timeout Detection

Monitor topic communication:

### Code Example: Topic Watchdog

```python
# topic_watchdog.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from datetime import datetime, timedelta

class TopicWatchdog(Node):
    """Monitors topic communication and detects stale data"""

    def __init__(self):
        super().__init__('topic_watchdog')

        # Parameters
        self.declare_parameter('monitored_topics', ['/current_waypoint'])
        self.declare_parameter('timeout_seconds', 10.0)

        self.monitored_topics = self.get_parameter('monitored_topics').value
        self.timeout = self.get_parameter('timeout_seconds').value

        # Track last message time
        self.last_message_time = {}
        self.topic_status = {}

        # Create subscribers for monitored topics
        self.subscribers = []
        for topic in self.monitored_topics:
            sub = self.create_subscription(
                PoseStamped,  # Adjust type as needed
                topic,
                lambda msg, t=topic: self.message_callback(msg, t),
                10
            )
            self.subscribers.append(sub)

        # Publisher for alerts
        self.alert_pub = self.create_publisher(
            String,
            '/topic_watchdog_alerts',
            10
        )

        # Timer to check for stale data
        self.check_timer = self.create_timer(1.0, self.check_staleness)

        self.get_logger().info(
            f'Topic watchdog monitoring {len(self.monitored_topics)} topics'
        )

    def message_callback(self, msg, topic_name):
        """Handle message on monitored topic"""
        self.last_message_time[topic_name] = datetime.now()

        if self.topic_status.get(topic_name) == 'STALE':
            self.get_logger().info(f'Topic {topic_name} data fresh again')
            self.topic_status[topic_name] = 'FRESH'

    def check_staleness(self):
        """Check for stale topic data"""
        current_time = datetime.now()
        timeout_delta = timedelta(seconds=self.timeout)

        for topic in self.monitored_topics:
            if topic not in self.last_message_time:
                if self.topic_status.get(topic) != 'NO_DATA':
                    self.get_logger().warn(f'No data received on {topic}')
                    self.topic_status[topic] = 'NO_DATA'
                continue

            time_since_message = current_time - self.last_message_time[topic]

            if time_since_message > timeout_delta:
                if self.topic_status.get(topic) != 'STALE':
                    self.get_logger().error(
                        f'Topic {topic} data is STALE '
                        f'({time_since_message.total_seconds():.1f}s old)'
                    )
                    self.topic_status[topic] = 'STALE'

                    # Publish alert
                    alert = String()
                    alert.data = f'STALE: {topic}'
                    self.alert_pub.publish(alert)

def main(args=None):
    rclpy.init(args=args)
    node = TopicWatchdog()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing Failure Scenarios

Test your watchdog system:

```bash
# Terminal 1: Start watchdog monitor
ros2 run workflow_orchestration watchdog_monitor

# Terminal 2: Start heartbeat node
ros2 run workflow_orchestration heartbeat_node

# Terminal 3: Monitor alerts
ros2 topic echo /watchdog_alerts

# Terminal 4: Kill heartbeat node to test failure detection
# (Find process and kill it)
ps aux | grep heartbeat_node
kill <PID>

# Watch Terminal 3 for failure alert
```

## Best Practices

### 1. Timeout Configuration
- Set timeout > 3x heartbeat period
- Account for network jitter
- Different timeouts for different criticality levels
- Make timeouts configurable

### 2. Recovery Strategy
- Limit restart attempts to prevent infinite loops
- Exponential backoff for repeated failures
- Alert operators after max attempts
- Log all recovery actions

### 3. Monitoring
- Log all heartbeats (with throttling)
- Track failure patterns
- Monitor recovery success rate
- Expose metrics for dashboards

### 4. Testing
- Test with simulated failures
- Test recovery under load
- Test cascading failures
- Test false positive scenarios

## Hardware Notes

> **Simulation vs. Real Hardware**: Watchdogs work identically in simulation and reality, but network reliability differs. Real systems need more conservative timeouts and may require redundant communication paths. Always test watchdog behavior under realistic network conditions.

## Summary

- Watchdogs detect failures through heartbeat monitoring
- Automatic recovery restarts failed nodes
- Topic watchdogs detect stale data
- Configure timeouts appropriately for your system
- Test all failure scenarios thoroughly

## Exercises

1. **Implement Resource Watchdog** (Advanced)
   - Monitor CPU and memory usage
   - Alert when thresholds exceeded
   - Implement resource-based recovery
   - Acceptance Criteria: Detects resource exhaustion within 5 seconds

2. **Add Cascading Failure Detection** (Expert)
   - Detect when multiple nodes fail together
   - Identify root cause node
   - Implement coordinated recovery
   - Acceptance Criteria: Recovers from cascading failures

## Next Steps

Continue to [A2: Supervisor Nodes and System Monitoring](./A2-supervisor-system-monitoring.md) to learn centralized health monitoring and recovery coordination.
