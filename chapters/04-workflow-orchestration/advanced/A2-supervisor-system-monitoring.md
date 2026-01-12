---
id: a2_supervisor_system_monitoring
title: Supervisor Nodes and System Monitoring
tier: advanced
chapter: chapter_4_workflow
estimated_time: 60-90 minutes
prerequisites: ["a1_watchdogs_health_monitoring"]
---

# A2: Supervisor Nodes and System Monitoring

## Learning Objectives

By the end of this lesson, you will be able to:
- Design and implement supervisor node architecture
- Monitor multiple components from a central location
- Coordinate recovery actions across the system
- Implement health reporting and diagnostics
- Build monitoring dashboards for operators

## Introduction

While watchdogs monitor individual components, a supervisor node provides centralized oversight of the entire system. It tracks component health, coordinates recovery actions, and provides a unified view of system status for operators and other systems.

In this lesson, you'll build a production-ready supervisor that monitors your workflow, detects complex failure patterns, and coordinates system-wide recovery.

## Supervisor Architecture

A supervisor node has several responsibilities:

```
Supervisor Node
├── Component Registry (tracks all monitored components)
├── Health Aggregator (collects health from all sources)
├── Failure Analyzer (identifies patterns and root causes)
├── Recovery Coordinator (orchestrates recovery actions)
└── Status Reporter (provides unified system status)
```

### Code Example: Supervisor Node

```python
# supervisor_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from datetime import datetime, timedelta
import json

class ComponentHealth:
    """Track health of a single component"""
    def __init__(self, name):
        self.name = name
        self.status = 'UNKNOWN'
        self.last_update = None
        self.error_count = 0
        self.last_error = None
        self.uptime_start = datetime.now()

    def update_status(self, status, message=''):
        """Update component status"""
        self.last_update = datetime.now()

        if status != self.status:
            if status == 'ERROR':
                self.error_count += 1
                self.last_error = message

        self.status = status

    def get_uptime(self):
        """Get component uptime in seconds"""
        return (datetime.now() - self.uptime_start).total_seconds()

    def is_healthy(self):
        """Check if component is healthy"""
        return self.status in ['OK', 'WARN']

class SupervisorNode(Node):
    """Centralized system health monitoring and coordination"""

    def __init__(self):
        super().__init__('supervisor_node')

        # Parameters
        self.declare_parameter('monitored_components', [
            'waypoint_manager',
            'navigation_controller',
            'status_monitor'
        ])
        self.declare_parameter('health_check_rate', 1.0)  # Hz
        self.declare_parameter('stale_threshold', 10.0)  # seconds

        self.components = self.get_parameter('monitored_components').value
        self.check_rate = self.get_parameter('health_check_rate').value
        self.stale_threshold = self.get_parameter('stale_threshold').value

        # Component tracking
        self.component_health = {
            name: ComponentHealth(name) for name in self.components
        }

        # Subscribers
        self.watchdog_sub = self.create_subscription(
            String,
            '/watchdog_alerts',
            self.watchdog_callback,
            10
        )

        self.recovery_sub = self.create_subscription(
            String,
            '/recovery_status',
            self.recovery_callback,
            10
        )

        # Publishers
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )

        self.system_status_pub = self.create_publisher(
            String,
            '/system_status',
            10
        )

        # Timer for health checks
        self.health_timer = self.create_timer(
            1.0 / self.check_rate,
            self.check_system_health
        )

        # Timer for diagnostics publishing
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

        self.get_logger().info(
            f'Supervisor monitoring {len(self.components)} components'
        )

    def watchdog_callback(self, msg):
        """Handle watchdog alerts"""
        alert_type, component = msg.data.split(': ')

        if component in self.component_health:
            if alert_type == 'FAILED':
                self.component_health[component].update_status(
                    'ERROR',
                    'Watchdog timeout'
                )
                self.get_logger().error(f'Component {component} failed')
            elif alert_type == 'RECOVERED':
                self.component_health[component].update_status('OK')
                self.get_logger().info(f'Component {component} recovered')

    def recovery_callback(self, msg):
        """Handle recovery status updates"""
        status_type, component = msg.data.split(': ')

        if component in self.component_health:
            if status_type == 'RESTARTED':
                self.get_logger().info(f'Component {component} restarted')
            elif status_type == 'GAVE_UP':
                self.component_health[component].update_status(
                    'ERROR',
                    'Recovery failed - max attempts exceeded'
                )
                self.get_logger().error(
                    f'Recovery gave up on {component}'
                )

    def check_system_health(self):
        """Check overall system health"""
        current_time = datetime.now()
        stale_delta = timedelta(seconds=self.stale_threshold)

        # Check for stale component data
        for name, health in self.component_health.items():
            if health.last_update:
                time_since_update = current_time - health.last_update
                if time_since_update > stale_delta:
                    if health.status != 'STALE':
                        health.update_status('STALE', 'No recent updates')
                        self.get_logger().warn(
                            f'Component {name} data is stale'
                        )

        # Determine overall system status
        system_status = self.get_system_status()

        # Publish system status
        status_msg = String()
        status_msg.data = json.dumps({
            'overall': system_status,
            'components': {
                name: health.status
                for name, health in self.component_health.items()
            },
            'timestamp': current_time.isoformat()
        })
        self.system_status_pub.publish(status_msg)

    def get_system_status(self):
        """Determine overall system status"""
        statuses = [h.status for h in self.component_health.values()]

        if 'ERROR' in statuses:
            return 'DEGRADED'
        elif 'STALE' in statuses:
            return 'DEGRADED'
        elif 'WARN' in statuses:
            return 'WARNING'
        elif all(s == 'OK' for s in statuses):
            return 'HEALTHY'
        else:
            return 'UNKNOWN'

    def publish_diagnostics(self):
        """Publish diagnostic information"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # Overall system diagnostic
        system_diag = DiagnosticStatus()
        system_diag.name = 'System'
        system_diag.hardware_id = 'workflow_orchestration'

        system_status = self.get_system_status()
        if system_status == 'HEALTHY':
            system_diag.level = DiagnosticStatus.OK
            system_diag.message = 'All components healthy'
        elif system_status == 'WARNING':
            system_diag.level = DiagnosticStatus.WARN
            system_diag.message = 'Some components have warnings'
        elif system_status == 'DEGRADED':
            system_diag.level = DiagnosticStatus.ERROR
            system_diag.message = 'System degraded - some components failed'
        else:
            system_diag.level = DiagnosticStatus.STALE
            system_diag.message = 'System status unknown'

        # Add component count
        healthy_count = sum(
            1 for h in self.component_health.values() if h.is_healthy()
        )
        system_diag.values.append(KeyValue(
            key='healthy_components',
            value=f'{healthy_count}/{len(self.components)}'
        ))

        diag_array.status.append(system_diag)

        # Individual component diagnostics
        for name, health in self.component_health.items():
            comp_diag = DiagnosticStatus()
            comp_diag.name = f'Component/{name}'
            comp_diag.hardware_id = name

            if health.status == 'OK':
                comp_diag.level = DiagnosticStatus.OK
                comp_diag.message = 'Component healthy'
            elif health.status == 'WARN':
                comp_diag.level = DiagnosticStatus.WARN
                comp_diag.message = 'Component has warnings'
            elif health.status == 'ERROR':
                comp_diag.level = DiagnosticStatus.ERROR
                comp_diag.message = health.last_error or 'Component failed'
            elif health.status == 'STALE':
                comp_diag.level = DiagnosticStatus.STALE
                comp_diag.message = 'Component data is stale'
            else:
                comp_diag.level = DiagnosticStatus.STALE
                comp_diag.message = 'Component status unknown'

            # Add component metrics
            comp_diag.values.append(KeyValue(
                key='uptime',
                value=f'{health.get_uptime():.1f}s'
            ))
            comp_diag.values.append(KeyValue(
                key='error_count',
                value=str(health.error_count)
            ))

            if health.last_update:
                time_since_update = (
                    datetime.now() - health.last_update
                ).total_seconds()
                comp_diag.values.append(KeyValue(
                    key='last_update',
                    value=f'{time_since_update:.1f}s ago'
                ))

            diag_array.status.append(comp_diag)

        self.diagnostics_pub.publish(diag_array)

def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()

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

## Monitoring Dashboard

Create a simple text-based dashboard:

### Code Example: Status Dashboard

```python
# status_dashboard.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
import json
import os

class StatusDashboard(Node):
    """Simple text-based status dashboard"""

    def __init__(self):
        super().__init__('status_dashboard')

        self.system_status = {}
        self.diagnostics = {}

        # Subscribers
        self.status_sub = self.create_subscription(
            String,
            '/system_status',
            self.status_callback,
            10
        )

        self.diag_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10
        )

        # Timer to refresh display
        self.display_timer = self.create_timer(1.0, self.update_display)

        self.get_logger().info('Status Dashboard started')

    def status_callback(self, msg):
        """Handle system status updates"""
        self.system_status = json.loads(msg.data)

    def diagnostics_callback(self, msg):
        """Handle diagnostics updates"""
        self.diagnostics = {}
        for status in msg.status:
            self.diagnostics[status.name] = {
                'level': status.level,
                'message': status.message,
                'values': {kv.key: kv.value for kv in status.values}
            }

    def update_display(self):
        """Update the dashboard display"""
        # Clear screen (Unix/Linux)
        os.system('clear' if os.name == 'posix' else 'cls')

        print("=" * 60)
        print("WORKFLOW ORCHESTRATION - SYSTEM STATUS DASHBOARD")
        print("=" * 60)
        print()

        # Overall system status
        if self.system_status:
            overall = self.system_status.get('overall', 'UNKNOWN')
            timestamp = self.system_status.get('timestamp', 'N/A')

            status_color = {
                'HEALTHY': '\033[92m',  # Green
                'WARNING': '\033[93m',  # Yellow
                'DEGRADED': '\033[91m',  # Red
                'UNKNOWN': '\033[90m'   # Gray
            }.get(overall, '')
            reset_color = '\033[0m'

            print(f"Overall Status: {status_color}{overall}{reset_color}")
            print(f"Last Update: {timestamp}")
            print()

        # Component status
        if self.system_status and 'components' in self.system_status:
            print("Component Status:")
            print("-" * 60)

            for name, status in self.system_status['components'].items():
                status_symbol = {
                    'OK': '✓',
                    'WARN': '⚠',
                    'ERROR': '✗',
                    'STALE': '?',
                    'UNKNOWN': '?'
                }.get(status, '?')

                print(f"  {status_symbol} {name:30s} {status}")

                # Show diagnostics if available
                diag_key = f'Component/{name}'
                if diag_key in self.diagnostics:
                    diag = self.diagnostics[diag_key]
                    if 'values' in diag:
                        for key, value in diag['values'].items():
                            print(f"      {key}: {value}")

            print()

        # System diagnostics
        if 'System' in self.diagnostics:
            sys_diag = self.diagnostics['System']
            print("System Metrics:")
            print("-" * 60)
            print(f"  Message: {sys_diag.get('message', 'N/A')}")
            if 'values' in sys_diag:
                for key, value in sys_diag['values'].items():
                    print(f"  {key}: {value}")
            print()

        print("=" * 60)
        print("Press Ctrl+C to exit")

def main(args=None):
    rclpy.init(args=args)
    node = StatusDashboard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nDashboard closed")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Complete Monitoring System

**Terminal 1: Supervisor**
```bash
ros2 run workflow_orchestration supervisor_node
```

**Terminal 2: Watchdog Monitor**
```bash
ros2 run workflow_orchestration watchdog_monitor
```

**Terminal 3: Recovery Manager**
```bash
ros2 run workflow_orchestration recovery_manager
```

**Terminal 4: Status Dashboard**
```bash
ros2 run workflow_orchestration status_dashboard
```

**Terminal 5: Your Workflow Nodes**
```bash
ros2 launch workflow_orchestration complete_workflow_launch.py
```

## Metrics Collection

Track performance metrics:

### Code Example: Metrics Collector

```python
# metrics_collector.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
import json
from datetime import datetime

class MetricsCollector(Node):
    """Collect and log system metrics"""

    def __init__(self):
        super().__init__('metrics_collector')

        self.metrics_log = []

        # Subscribers
        self.status_sub = self.create_subscription(
            String,
            '/system_status',
            self.status_callback,
            10
        )

        self.diag_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10
        )

        # Timer to save metrics
        self.save_timer = self.create_timer(60.0, self.save_metrics)

        self.get_logger().info('Metrics Collector started')

    def status_callback(self, msg):
        """Collect status metrics"""
        data = json.loads(msg.data)

        metric = {
            'timestamp': datetime.now().isoformat(),
            'type': 'status',
            'data': data
        }

        self.metrics_log.append(metric)

    def diagnostics_callback(self, msg):
        """Collect diagnostic metrics"""
        metrics = []

        for status in msg.status:
            metric = {
                'name': status.name,
                'level': status.level,
                'message': status.message,
                'values': {kv.key: kv.value for kv in status.values}
            }
            metrics.append(metric)

        log_entry = {
            'timestamp': datetime.now().isoformat(),
            'type': 'diagnostics',
            'data': metrics
        }

        self.metrics_log.append(log_entry)

    def save_metrics(self):
        """Save metrics to file"""
        if not self.metrics_log:
            return

        filename = f'metrics_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'

        try:
            with open(filename, 'w') as f:
                json.dump(self.metrics_log, f, indent=2)

            self.get_logger().info(
                f'Saved {len(self.metrics_log)} metrics to {filename}'
            )

            # Clear log after saving
            self.metrics_log = []

        except Exception as e:
            self.get_logger().error(f'Failed to save metrics: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Save metrics on exit
        node.save_metrics()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

### 1. Supervisor Design
- Single supervisor per system
- Monitor all critical components
- Provide unified status view
- Coordinate recovery actions

### 2. Health Reporting
- Use standard diagnostic messages
- Include relevant metrics
- Update at appropriate rate (1-10 Hz)
- Log all status changes

### 3. Failure Analysis
- Track failure patterns
- Identify root causes
- Correlate related failures
- Learn from history

### 4. Operator Interface
- Clear, actionable information
- Real-time updates
- Historical data access
- Alert prioritization

## Hardware Notes

> **Simulation vs. Real Hardware**: Supervisor nodes work identically in simulation and reality. However, real deployments benefit from persistent logging, remote monitoring capabilities, and integration with fleet management systems. Consider using time-series databases (InfluxDB, Prometheus) for production metrics.

## Summary

- Supervisor nodes provide centralized system monitoring
- Diagnostic messages standardize health reporting
- Dashboards give operators real-time visibility
- Metrics collection enables performance analysis
- Coordinated recovery improves system reliability

## Exercises

1. **Add Performance Metrics** (Advanced)
   - Track CPU and memory usage per component
   - Monitor message rates and latencies
   - Detect performance degradation
   - Acceptance Criteria: Dashboard shows resource usage

2. **Implement Alert Prioritization** (Expert)
   - Classify alerts by severity
   - Implement alert escalation
   - Reduce alert fatigue
   - Acceptance Criteria: Critical alerts highlighted

## Next Steps

Continue to [A3: Error Recovery and Fault Tolerance](./A3-error-recovery-fault-tolerance.md) to learn comprehensive error recovery strategies and production deployment.
