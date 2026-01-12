---
id: a3_error_recovery_fault_tolerance
title: Error Recovery and Fault Tolerance
tier: advanced
chapter: chapter_4_workflow
estimated_time: 60-90 minutes
prerequisites: ["a1_watchdogs_health_monitoring", "a2_supervisor_system_monitoring"]
---

# A3: Error Recovery and Fault Tolerance

## Learning Objectives

By the end of this lesson, you will be able to:
- Design comprehensive error recovery strategies
- Implement graceful degradation patterns
- Handle state persistence and recovery
- Build production-ready fault-tolerant systems
- Deploy workflows with confidence

## Introduction

Error recovery is the final piece of production-ready workflow orchestration. While watchdogs detect failures and supervisors coordinate responses, recovery strategies determine how the system returns to normal operation.

In this lesson, you'll implement comprehensive error recovery that handles everything from transient network glitches to catastrophic component failures, ensuring your robot can operate continuously with minimal human intervention.

## Recovery Strategy Hierarchy

Implement multiple levels of recovery:

```
Level 1: Retry (transient failures)
    ↓ (if fails)
Level 2: Fallback (use alternative approach)
    ↓ (if fails)
Level 3: Degraded Mode (reduced functionality)
    ↓ (if fails)
Level 4: Safe Stop (halt operations safely)
    ↓ (if fails)
Level 5: Emergency Stop (immediate halt)
```

### Code Example: Recovery Strategy Manager

```python
# recovery_strategy_manager.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
import time

class RecoveryLevel(Enum):
    """Recovery strategy levels"""
    RETRY = 1
    FALLBACK = 2
    DEGRADED = 3
    SAFE_STOP = 4
    EMERGENCY_STOP = 5

class RecoveryStrategy:
    """Defines recovery strategy for a failure type"""
    def __init__(self, name, max_retries=3, retry_delay=2.0):
        self.name = name
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        self.retry_count = 0
        self.last_attempt = None

    def can_retry(self):
        """Check if retry is allowed"""
        if self.retry_count >= self.max_retries:
            return False

        if self.last_attempt:
            time_since_attempt = time.time() - self.last_attempt
            if time_since_attempt < self.retry_delay:
                return False

        return True

    def attempt_retry(self):
        """Record retry attempt"""
        self.retry_count += 1
        self.last_attempt = time.time()

    def reset(self):
        """Reset retry counter"""
        self.retry_count = 0
        self.last_attempt = None

class RecoveryStrategyManager(Node):
    """Manages recovery strategies for different failure types"""

    def __init__(self):
        super().__init__('recovery_strategy_manager')

        # Define recovery strategies for different failures
        self.strategies = {
            'navigation_failed': RecoveryStrategy(
                'navigation_failed',
                max_retries=3,
                retry_delay=5.0
            ),
            'sensor_dropout': RecoveryStrategy(
                'sensor_dropout',
                max_retries=5,
                retry_delay=1.0
            ),
            'communication_loss': RecoveryStrategy(
                'communication_loss',
                max_retries=10,
                retry_delay=0.5
            ),
        }

        # Current recovery level
        self.recovery_level = RecoveryLevel.RETRY

        # Subscribers
        self.failure_sub = self.create_subscription(
            String,
            '/system_failures',
            self.failure_callback,
            10
        )

        # Publishers
        self.recovery_action_pub = self.create_publisher(
            String,
            '/recovery_actions',
            10
        )

        self.system_mode_pub = self.create_publisher(
            String,
            '/system_mode',
            10
        )

        self.get_logger().info('Recovery Strategy Manager initialized')

    def failure_callback(self, msg):
        """Handle system failure"""
        failure_type = msg.data

        self.get_logger().warn(f'Handling failure: {failure_type}')

        # Get strategy for this failure type
        strategy = self.strategies.get(
            failure_type,
            RecoveryStrategy('default', max_retries=1)
        )

        # Attempt recovery based on current level
        if self.recovery_level == RecoveryLevel.RETRY:
            self.attempt_retry_recovery(failure_type, strategy)
        elif self.recovery_level == RecoveryLevel.FALLBACK:
            self.attempt_fallback_recovery(failure_type)
        elif self.recovery_level == RecoveryLevel.DEGRADED:
            self.enter_degraded_mode(failure_type)
        elif self.recovery_level == RecoveryLevel.SAFE_STOP:
            self.execute_safe_stop()
        else:
            self.execute_emergency_stop()

    def attempt_retry_recovery(self, failure_type, strategy):
        """Attempt retry recovery"""
        if strategy.can_retry():
            strategy.attempt_retry()
            self.get_logger().info(
                f'Retry {strategy.retry_count}/{strategy.max_retries} '
                f'for {failure_type}'
            )

            # Publish retry action
            action = String()
            action.data = f'RETRY:{failure_type}'
            self.recovery_action_pub.publish(action)

        else:
            self.get_logger().warn(
                f'Max retries exceeded for {failure_type}, '
                'escalating to fallback'
            )
            self.recovery_level = RecoveryLevel.FALLBACK
            self.attempt_fallback_recovery(failure_type)

    def attempt_fallback_recovery(self, failure_type):
        """Attempt fallback recovery"""
        self.get_logger().info(f'Attempting fallback for {failure_type}')

        # Publish fallback action
        action = String()
        action.data = f'FALLBACK:{failure_type}'
        self.recovery_action_pub.publish(action)

        # If fallback fails, escalate to degraded mode
        # (In real implementation, you'd wait for fallback result)

    def enter_degraded_mode(self, failure_type):
        """Enter degraded operation mode"""
        self.get_logger().warn(
            f'Entering degraded mode due to {failure_type}'
        )

        # Publish system mode change
        mode = String()
        mode.data = 'DEGRADED'
        self.system_mode_pub.publish(mode)

        # Publish degraded mode action
        action = String()
        action.data = f'DEGRADED_MODE:{failure_type}'
        self.recovery_action_pub.publish(action)

    def execute_safe_stop(self):
        """Execute safe stop"""
        self.get_logger().error('Executing safe stop')

        # Publish safe stop command
        mode = String()
        mode.data = 'SAFE_STOP'
        self.system_mode_pub.publish(mode)

        action = String()
        action.data = 'SAFE_STOP'
        self.recovery_action_pub.publish(action)

    def execute_emergency_stop(self):
        """Execute emergency stop"""
        self.get_logger().error('EMERGENCY STOP')

        # Publish emergency stop
        mode = String()
        mode.data = 'EMERGENCY_STOP'
        self.system_mode_pub.publish(mode)

        action = String()
        action.data = 'EMERGENCY_STOP'
        self.recovery_action_pub.publish(action)

def main(args=None):
    rclpy.init(args=args)
    node = RecoveryStrategyManager()

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

## State Persistence

Preserve state across failures:

### Code Example: State Persistence

```python
# state_persistence.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import os
from datetime import datetime

class StatePersistence(Node):
    """Persist and restore workflow state"""

    def __init__(self):
        super().__init__('state_persistence')

        # Parameters
        self.declare_parameter('state_file', '/tmp/workflow_state.json')
        self.declare_parameter('save_interval', 5.0)  # seconds

        self.state_file = self.get_parameter('state_file').value
        self.save_interval = self.get_parameter('save_interval').value

        # Current state
        self.workflow_state = {
            'current_waypoint_index': 0,
            'completed_waypoints': [],
            'navigation_state': 'idle',
            'last_known_pose': None,
            'error_count': 0,
            'last_update': None
        }

        # Subscribers to track state
        self.waypoint_sub = self.create_subscription(
            String,
            '/waypoint_status',
            self.waypoint_callback,
            10
        )

        self.nav_state_sub = self.create_subscription(
            String,
            '/navigation_state',
            self.nav_state_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )

        # Timer to save state periodically
        self.save_timer = self.create_timer(
            self.save_interval,
            self.save_state
        )

        # Try to restore previous state
        self.restore_state()

        self.get_logger().info('State Persistence initialized')

    def waypoint_callback(self, msg):
        """Track waypoint progress"""
        # Parse waypoint status (e.g., "Waypoint 2/5")
        parts = msg.data.split()
        if len(parts) >= 2:
            current = parts[1].split('/')[0]
            self.workflow_state['current_waypoint_index'] = int(current) - 1

        self.workflow_state['last_update'] = datetime.now().isoformat()

    def nav_state_callback(self, msg):
        """Track navigation state"""
        self.workflow_state['navigation_state'] = msg.data

        if msg.data == 'goal_reached':
            waypoint_idx = self.workflow_state['current_waypoint_index']
            if waypoint_idx not in self.workflow_state['completed_waypoints']:
                self.workflow_state['completed_waypoints'].append(waypoint_idx)

        self.workflow_state['last_update'] = datetime.now().isoformat()

    def pose_callback(self, msg):
        """Track robot pose"""
        self.workflow_state['last_known_pose'] = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }

    def save_state(self):
        """Save current state to file"""
        try:
            with open(self.state_file, 'w') as f:
                json.dump(self.workflow_state, f, indent=2)

            self.get_logger().info(
                'State saved',
                throttle_duration_sec=30.0
            )

        except Exception as e:
            self.get_logger().error(f'Failed to save state: {e}')

    def restore_state(self):
        """Restore state from file"""
        if not os.path.exists(self.state_file):
            self.get_logger().info('No previous state to restore')
            return

        try:
            with open(self.state_file, 'r') as f:
                restored_state = json.load(f)

            self.workflow_state.update(restored_state)

            self.get_logger().info(
                f'State restored: waypoint {self.workflow_state["current_waypoint_index"]}, '
                f'{len(self.workflow_state["completed_waypoints"])} completed'
            )

            # Publish restored state
            # (In real implementation, you'd notify other nodes)

        except Exception as e:
            self.get_logger().error(f'Failed to restore state: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = StatePersistence()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Save state on exit
        node.save_state()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Graceful Degradation

Reduce functionality when components fail:

### Code Example: Degraded Mode Manager

```python
# degraded_mode_manager.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum

class OperationMode(Enum):
    """System operation modes"""
    NORMAL = "normal"
    DEGRADED = "degraded"
    MINIMAL = "minimal"
    SAFE_STOP = "safe_stop"

class DegradedModeManager(Node):
    """Manages graceful degradation of system functionality"""

    def __init__(self):
        super().__init__('degraded_mode_manager')

        self.current_mode = OperationMode.NORMAL
        self.failed_components = set()

        # Define critical components
        self.critical_components = {
            'navigation_controller',
            'waypoint_manager'
        }

        # Define optional components
        self.optional_components = {
            'status_monitor'
        }

        # Subscribers
        self.failure_sub = self.create_subscription(
            String,
            '/watchdog_alerts',
            self.failure_callback,
            10
        )

        # Publishers
        self.mode_pub = self.create_publisher(
            String,
            '/operation_mode',
            10
        )

        self.capability_pub = self.create_publisher(
            String,
            '/system_capabilities',
            10
        )

        self.get_logger().info('Degraded Mode Manager initialized')

    def failure_callback(self, msg):
        """Handle component failures"""
        alert_type, component = msg.data.split(': ')

        if alert_type == 'FAILED':
            self.failed_components.add(component)
            self.evaluate_operation_mode()
        elif alert_type == 'RECOVERED':
            self.failed_components.discard(component)
            self.evaluate_operation_mode()

    def evaluate_operation_mode(self):
        """Determine appropriate operation mode"""
        # Check for critical component failures
        critical_failures = self.failed_components & self.critical_components

        if critical_failures:
            # Critical component failed - safe stop
            new_mode = OperationMode.SAFE_STOP
            self.get_logger().error(
                f'Critical components failed: {critical_failures}'
            )

        elif len(self.failed_components) > 0:
            # Non-critical failures - degraded mode
            new_mode = OperationMode.DEGRADED
            self.get_logger().warn(
                f'Operating in degraded mode. Failed: {self.failed_components}'
            )

        else:
            # All components healthy
            new_mode = OperationMode.NORMAL

        # Update mode if changed
        if new_mode != self.current_mode:
            self.transition_to_mode(new_mode)

    def transition_to_mode(self, new_mode):
        """Transition to new operation mode"""
        old_mode = self.current_mode
        self.current_mode = new_mode

        self.get_logger().info(
            f'Mode transition: {old_mode.value} -> {new_mode.value}'
        )

        # Publish mode change
        mode_msg = String()
        mode_msg.data = new_mode.value
        self.mode_pub.publish(mode_msg)

        # Publish updated capabilities
        self.publish_capabilities()

    def publish_capabilities(self):
        """Publish current system capabilities"""
        capabilities = []

        if self.current_mode == OperationMode.NORMAL:
            capabilities = [
                'full_navigation',
                'waypoint_following',
                'status_monitoring',
                'error_recovery'
            ]
        elif self.current_mode == OperationMode.DEGRADED:
            capabilities = [
                'basic_navigation',
                'manual_waypoints',
                'limited_monitoring'
            ]
        elif self.current_mode == OperationMode.MINIMAL:
            capabilities = [
                'manual_control_only'
            ]
        else:  # SAFE_STOP
            capabilities = []

        cap_msg = String()
        cap_msg.data = ','.join(capabilities)
        self.capability_pub.publish(cap_msg)

        self.get_logger().info(f'Capabilities: {capabilities}')

def main(args=None):
    rclpy.init(args=args)
    node = DegradedModeManager()

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

## Production Deployment Checklist

Before deploying to production:

### 1. Testing
- [ ] All failure modes tested
- [ ] Recovery behaviors validated
- [ ] Stress testing completed
- [ ] Long-duration testing (24+ hours)
- [ ] Edge cases covered

### 2. Monitoring
- [ ] Logging configured
- [ ] Metrics collection enabled
- [ ] Dashboards operational
- [ ] Alerts configured
- [ ] Remote access enabled

### 3. Documentation
- [ ] Operator runbook created
- [ ] Troubleshooting guide written
- [ ] Recovery procedures documented
- [ ] Configuration documented
- [ ] Known issues listed

### 4. Safety
- [ ] Emergency stop tested
- [ ] Safe stop validated
- [ ] Operator training completed
- [ ] Safety protocols established
- [ ] Incident response plan ready

### 5. Performance
- [ ] Resource usage acceptable
- [ ] Latency within limits
- [ ] Throughput meets requirements
- [ ] Scalability validated
- [ ] Optimization completed

## Best Practices

### 1. Recovery Design
- Implement multiple recovery levels
- Start with least disruptive recovery
- Escalate only when necessary
- Always have a safe fallback

### 2. State Management
- Persist critical state regularly
- Validate restored state
- Handle corrupted state gracefully
- Clean up old state files

### 3. Degradation Strategy
- Define clear capability levels
- Communicate mode changes
- Maintain safety in all modes
- Provide manual override

### 4. Production Readiness
- Test exhaustively
- Monitor continuously
- Document thoroughly
- Train operators

## Hardware Notes

> **Simulation vs. Real Hardware**: Error recovery is critical on real hardware where failures have physical consequences. Always test recovery behaviors in simulation first, then in controlled real-world conditions before full deployment. Real hardware may have failure modes not present in simulation - plan for the unexpected.

## Summary

- Implement hierarchical recovery strategies
- Persist state to survive failures
- Gracefully degrade when components fail
- Follow production deployment checklist
- Test, monitor, and document thoroughly

## Exercises

1. **Implement Checkpoint System** (Advanced)
   - Save workflow checkpoints at key points
   - Resume from last checkpoint after failure
   - Validate checkpoint integrity
   - Acceptance Criteria: Can resume after any failure

2. **Build Failure Injection Tool** (Expert)
   - Create tool to inject failures for testing
   - Test all recovery paths
   - Measure recovery time
   - Acceptance Criteria: All recovery paths tested

## Congratulations!

You've completed the Advanced tier and mastered production-ready workflow orchestration! You can now:
- Implement watchdogs and health monitoring
- Build supervisor nodes for system oversight
- Design comprehensive error recovery strategies
- Deploy fault-tolerant robotic workflows

Your workflows are now ready for production deployment with confidence.

## Next Steps

1. **Complete the Advanced Exercises**: Build a complete production system
2. **Deploy to Real Hardware**: Test in real-world conditions
3. **Share Your Work**: Contribute patterns to the community
4. **Move to Chapter 5**: Learn adaptive robotics and learning systems
