---
id: advanced_exercises_workflow
title: "Advanced Tier Exercises"
sidebar_position: 63
tier: advanced
chapter: chapter_4_workflow
section: exercises
---

# Advanced Tier Exercises

This document consolidates all exercises from the Advanced tier lessons for building production-ready, fault-tolerant robotic workflows.

---

## Exercise 01: Watchdog System Implementation

**Objective**: Implement a comprehensive watchdog system for a multi-node workflow.

**Scenario**: You have a navigation workflow with multiple nodes (sensor, planner, controller). Build a watchdog that monitors all nodes and detects failures.

**Requirements**:

1. **Heartbeat Mechanism**: Each node publishes heartbeats
2. **Watchdog Node**: Monitors all heartbeats
3. **Timeout Detection**: Detects when nodes stop responding
4. **Health Status**: Publishes overall system health
5. **Alerts**: Logs warnings and errors

**Tasks**:

1. Modify existing nodes to publish heartbeats
2. Create a watchdog node that subscribes to all heartbeats
3. Implement timeout detection logic
4. Publish system health status
5. Test by intentionally killing nodes

**Acceptance Criteria**:
- [ ] All nodes publish heartbeats
- [ ] Watchdog detects node failures within 2 seconds
- [ ] System health status is accurate
- [ ] Alerts are logged appropriately
- [ ] Tested with multiple failure scenarios

**Starter Code**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time

class WatchdogNode(Node):
    def __init__(self):
        super().__init__('watchdog')

        # Dictionary to track last heartbeat time for each node
        self.last_heartbeat = {}
        self.timeout_threshold = 2.0  # seconds

        # TODO: Create subscribers for each node's heartbeat
        # TODO: Create publisher for system health
        # TODO: Create timer to check heartbeats

    def heartbeat_callback(self, msg, node_name):
        """Update last heartbeat time for a node"""
        self.last_heartbeat[node_name] = time.time()

    def check_health(self):
        """Check if all nodes are healthy"""
        current_time = time.time()
        all_healthy = True

        for node_name, last_time in self.last_heartbeat.items():
            if current_time - last_time > self.timeout_threshold:
                self.get_logger().error(f'Node {node_name} timeout!')
                all_healthy = False

        # TODO: Publish health status

def main():
    rclpy.init()
    node = WatchdogNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Exercise 02: Supervisor Node with Recovery

**Objective**: Build a supervisor node that can automatically recover from failures.

**Scenario**: Your navigation system occasionally fails. Build a supervisor that detects failures and implements recovery strategies.

**Recovery Strategies**:
1. **Restart**: Restart the failed node
2. **Fallback**: Switch to a simpler behavior
3. **Safe Mode**: Stop and wait for manual intervention

**Requirements**:

1. Monitor system health
2. Detect different types of failures
3. Implement appropriate recovery for each failure type
4. Log all recovery attempts
5. Escalate to safe mode if recovery fails

**Tasks**:

1. Create supervisor node
2. Implement failure detection
3. Implement recovery strategies
4. Add state persistence for recovery
5. Test with various failure scenarios

**Acceptance Criteria**:
- [ ] Supervisor detects failures correctly
- [ ] Recovery strategies are implemented
- [ ] System recovers automatically when possible
- [ ] Safe mode activates when recovery fails
- [ ] All actions are logged

---

## Exercise 03: Sensor Dropout Handling

**Objective**: Handle sensor dropouts gracefully without system failure.

**Scenario**: Your robot's LIDAR occasionally drops out for 1-2 seconds. Implement handling so the robot continues operating safely.

**Requirements**:

1. Detect sensor dropouts
2. Use last known good data temporarily
3. Reduce speed or stop if dropout is too long
4. Resume normal operation when sensor recovers
5. Log all dropout events

**Tasks**:

1. Implement sensor health monitoring
2. Create data buffering for last known good data
3. Implement degraded operation mode
4. Add automatic recovery when sensor returns
5. Test with simulated dropouts

**Acceptance Criteria**:
- [ ] Sensor dropouts detected within 500ms
- [ ] Robot operates safely during short dropouts
- [ ] Robot stops safely during long dropouts
- [ ] Normal operation resumes automatically
- [ ] All events logged

**Starter Code**:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time

class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')

        self.last_sensor_time = time.time()
        self.last_good_data = None
        self.dropout_threshold = 0.5  # seconds
        self.in_dropout = False

        # TODO: Create subscriber for sensor data
        # TODO: Create timer to check for dropouts
        # TODO: Create publisher for processed data

    def sensor_callback(self, msg):
        """Process incoming sensor data"""
        self.last_sensor_time = time.time()
        self.last_good_data = msg

        if self.in_dropout:
            self.get_logger().info('Sensor recovered!')
            self.in_dropout = False

        # TODO: Publish data

    def check_dropout(self):
        """Check if sensor has dropped out"""
        current_time = time.time()
        time_since_data = current_time - self.last_sensor_time

        if time_since_data > self.dropout_threshold and not self.in_dropout:
            self.get_logger().warn('Sensor dropout detected!')
            self.in_dropout = True
            # TODO: Implement degraded operation

def main():
    rclpy.init()
    node = SensorMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Exercise 04: Continuous Operation System

**Objective**: Deploy a workflow that can run continuously for 24+ hours.

**Requirements**:

1. **Resource Management**: No memory leaks, CPU usage stable
2. **Log Rotation**: Logs don't fill disk
3. **Performance Monitoring**: Track key metrics
4. **Graceful Restart**: Can restart without data loss
5. **Health Reporting**: Regular health reports

**Tasks**:

1. Profile existing workflow for resource usage
2. Fix any memory leaks or resource issues
3. Implement log rotation
4. Add performance monitoring
5. Run 24-hour stress test

**Acceptance Criteria**:
- [ ] System runs for 24+ hours without issues
- [ ] Memory usage is stable
- [ ] CPU usage is reasonable
- [ ] Logs are managed properly
- [ ] Performance metrics are tracked

---

## Capstone Project: Production-Ready Autonomous Robot

**Objective**: Build a complete, production-ready autonomous robot workflow with full fault tolerance.

**System Requirements**:

1. **Core Functionality**:
   - Autonomous navigation
   - Obstacle avoidance
   - Task execution (delivery, patrol, etc.)

2. **Fault Tolerance**:
   - Watchdog monitoring
   - Supervisor with recovery
   - Sensor dropout handling
   - Error state management

3. **Production Features**:
   - Continuous operation capability
   - Resource management
   - Comprehensive logging
   - Performance monitoring
   - Health reporting

4. **Testing**:
   - Unit tests for critical components
   - Integration tests for workflows
   - Stress tests for reliability
   - Failure injection tests

**Architecture**:

```
┌─────────────────────────────────────────────────┐
│              Supervisor Node                     │
│  (Monitors health, implements recovery)          │
└─────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────┐
│              Watchdog Node                       │
│  (Monitors heartbeats, detects failures)         │
└─────────────────────────────────────────────────┘
                      ↓
┌──────────────┬──────────────┬──────────────────┐
│ State Machine│ Sensor Nodes │ Navigation Stack │
│    Node      │  (monitored) │   (monitored)    │
└──────────────┴──────────────┴──────────────────┘
```

**Tasks**:

1. Design complete system architecture
2. Implement all core functionality
3. Add all fault tolerance features
4. Implement production features
5. Write comprehensive tests
6. Deploy and run 24-hour test
7. Document everything

**Deliverables**:
- Complete source code
- Launch files and configuration
- Test suite
- Deployment guide
- Operations manual
- Performance report from 24-hour test
- Video demonstration

**Acceptance Criteria**:
- [ ] All core functionality works
- [ ] All fault tolerance features implemented
- [ ] Passes all tests
- [ ] Runs for 24+ hours successfully
- [ ] Comprehensive documentation
- [ ] Production-ready quality

---

## Self-Assessment Checklist

After completing all advanced exercises, verify you can:

### Fault Tolerance
- [ ] Implement watchdog systems
- [ ] Build supervisor nodes with recovery
- [ ] Handle sensor dropouts gracefully
- [ ] Implement multiple recovery strategies

### Production Systems
- [ ] Design for continuous operation
- [ ] Manage resources effectively
- [ ] Implement comprehensive logging
- [ ] Monitor performance metrics

### Testing & Validation
- [ ] Write tests for fault tolerance
- [ ] Perform stress testing
- [ ] Inject failures for testing
- [ ] Validate recovery mechanisms

### Deployment
- [ ] Deploy production-ready systems
- [ ] Document operations procedures
- [ ] Create monitoring dashboards
- [ ] Plan maintenance strategies

### Ready for Real-World Deployment?
If you checked all boxes above, you have the skills to build and deploy production-ready robotic workflows!

---

## Next Steps

1. **Apply to Real Projects**: Use these patterns in your own robotic systems
2. **Continue Learning**: Explore Chapter 5 for advanced AI integration
3. **Contribute**: Share your implementations with the community
4. **Iterate**: Continuously improve based on real-world experience

---

**Congratulations!** You've completed the Advanced tier and can now build production-ready, fault-tolerant robotic workflows.
