# Research Document: Chapter 4 - Workflow Orchestration

**Branch**: `004-workflow-orchestration` | **Date**: 2025-12-30
**Status**: Complete

---

## Research Summary

This document consolidates research findings for implementing Chapter 4 on Workflow Orchestration. All "NEEDS CLARIFICATION" items from the Technical Context have been resolved.

---

## 1. Pure Python FSM Patterns for ROS 2

### Decision: Enum-based State Machine with Class Pattern

**Rationale**: Provides type safety via Python enums, thread-safe transitions with locks, and explicit transition rules that prevent invalid state changes. Easily extendable and learn-friendly for educational content.

**Alternatives Considered**:
- YASMIN (external library) - Rejected per FR-A01 requiring no external state machine libraries
- SMACH (ROS 1 legacy) - Rejected due to ROS 1 dependency
- py_trees (behavior trees) - Rejected as more complex than needed for this chapter

### Core Implementation Pattern

```python
from enum import Enum
from typing import Callable, Dict
import threading

class RobotState(Enum):
    """Define all possible states for the FSM."""
    IDLE = "idle"
    EXECUTING = "executing"
    PAUSED = "paused"
    ERROR = "error"
    SHUTDOWN = "shutdown"

class StateTransition:
    """Represents a valid state transition."""
    def __init__(self, from_state: RobotState, to_state: RobotState,
                 condition: Callable = None):
        self.from_state = from_state
        self.to_state = to_state
        self.condition = condition or (lambda: True)

class SimpleFSM:
    """Pure Python FSM implementation."""

    def __init__(self, initial_state: RobotState):
        self.current_state = initial_state
        self.transitions: Dict[RobotState, list] = {}
        self._lock = threading.Lock()

    def add_transition(self, from_state: RobotState, to_state: RobotState,
                       condition: Callable = None):
        """Register a valid state transition."""
        if from_state not in self.transitions:
            self.transitions[from_state] = []
        self.transitions[from_state].append(
            StateTransition(from_state, to_state, condition)
        )

    def try_transition(self, target_state: RobotState) -> bool:
        """Attempt to transition to target state."""
        with self._lock:
            if self.current_state not in self.transitions:
                return False

            for transition in self.transitions[self.current_state]:
                if (transition.to_state == target_state and
                    transition.condition()):
                    self.current_state = target_state
                    return True
            return False

    def is_in_state(self, state: RobotState) -> bool:
        """Check if FSM is in specific state."""
        with self._lock:
            return self.current_state == state
```

### ROS 2 Integration Pattern

```python
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class WorkflowNode(Node):
    """Node with integrated FSM for workflow orchestration."""

    def __init__(self):
        super().__init__('workflow_node')

        # FSM state
        self.state_fsm = SimpleFSM(RobotState.IDLE)
        self._setup_transitions()

        # Thread-safe state mutation
        self._state_lock = threading.Lock()

        # Callback groups for concurrent execution
        self.cb_group_main = ReentrantCallbackGroup()

        # State-driven timer
        self.state_timer = self.create_timer(
            0.1,  # 10 Hz
            self.update_state,
            callback_group=self.cb_group_main
        )
```

### Key Best Practices

1. **Always use locks** for thread-safe state mutation
2. **Use ReentrantCallbackGroup** for concurrent operations
3. **Check cancellation** before blocking operations in action servers
4. **Validate state in callbacks**, not just transitions
5. **Use enums over strings** for type safety
6. **Implement watchdog timers** for state validation

---

## 2. Watchdog Supervisor Patterns for ROS 2

### Decision: Timer-based Heartbeat Monitoring with Multi-tier Recovery

**Rationale**: Timer-based approach is simple, reliable, and integrates naturally with ROS 2 timer callbacks. Multi-tier recovery provides escalation path for production systems.

**Alternatives Considered**:
- Process monitoring via subprocess - More complex, OS-dependent
- ROS 2 lifecycle nodes only - Good for controlled startup but insufficient for crash recovery

### Core Heartbeat Monitor Pattern

```python
class HeartbeatMonitor(Node):
    """Monitor node health via message timestamps."""

    def __init__(self):
        super().__init__('heartbeat_monitor')

        # Parameters
        self.declare_parameter('timeout_sec', 5.0)
        self.timeout = self.get_parameter('timeout_sec').value

        # State
        self.last_heartbeat_time = time.time()
        self.is_alive = False

        # Watchdog timer
        self.watchdog_timer = self.create_timer(1.0, self.check_health)

    def heartbeat_callback(self, msg):
        """Update timestamp on heartbeat reception."""
        self.last_heartbeat_time = time.time()
        self.is_alive = True

    def check_health(self):
        """Periodic health check."""
        elapsed = time.time() - self.last_heartbeat_time

        if elapsed > self.timeout:
            if self.is_alive:  # State transition
                self.is_alive = False
                self.get_logger().error(
                    f'Node health: CRITICAL - No heartbeat for {elapsed:.1f}s'
                )
```

### Multi-tier Recovery Strategy

```python
def _attempt_recovery(self, node_name, info):
    """Execute recovery with backoff strategy."""

    # Backoff calculation (exponential)
    backoff = self.backoff * (2 ** info['recovery_attempts'])

    # Recovery actions (in order of escalation)
    if info['recovery_attempts'] == 0:
        self._restart_graceful(node_name)  # Tier 1: Soft reset
    elif info['recovery_attempts'] == 1:
        self._restart_force(node_name)     # Tier 2: Hard restart
    elif info['recovery_attempts'] == 2:
        self._notify_critical(node_name)   # Tier 3: Alert human

    info['recovery_attempts'] += 1
```

### Sensor Dropout Detection

```python
class SensorDropoutDetector(Node):
    """Detect sensor dropouts using frequency analysis."""

    def sensor_callback(self, msg, topic):
        now = time.time()
        interval = now - self.last_msg_time
        self.last_msg_time = now

        # Frequency-based dropout detection
        if interval > self.expected_interval * 1.5:  # 50% slower than expected
            self.consecutive_dropouts += 1

            if self.consecutive_dropouts >= self.alert_threshold:
                self._trigger_dropout_alert(topic)
        else:
            self.consecutive_dropouts = 0
```

### Key Implementation Patterns

| Pattern | Use Case | Implementation |
|---------|----------|-----------------|
| Heartbeat Monitor | Simple node health | Timer + timestamp comparison |
| Lifecycle Node | Controlled startup/shutdown | configure/activate/deactivate/cleanup |
| Latency Monitor | Detect delays & dropouts | Rolling window + statistics |
| Watchdog Supervisor | Multiple node supervision | Timeout detection + recovery escalation |

---

## 3. ROS 2 launch_testing Framework

### Decision: launch_testing with pytest Integration (Humble-compatible)

**Rationale**: Provides full pytest integration, proper domain ID isolation, and works with ROS 2 Humble. launch_pytest is newer but Rolling-focused.

**Alternatives Considered**:
- launch_pytest (modern) - Preferred for Rolling, but Humble compatibility uncertain
- ros2-easy-test (community) - Simpler but less feature-rich

### Basic Test Structure

```python
"""Test pipeline startup and message passing."""

import unittest
import launch
import launch_ros
import launch_testing
from launch_ros.actions import Node

def generate_test_description():
    """Generate the launch description for nodes under test."""
    return launch.LaunchDescription([
        Node(
            package='workflow_examples',
            executable='sensor_node',
            name='sensor',
        ),
        Node(
            package='workflow_examples',
            executable='processor_node',
            name='processor',
        ),
        launch_testing.actions.ReadyToTest(),
    ])

class TestStartupBehavior(unittest.TestCase):
    """Active tests running while nodes are alive."""

    def test_nodes_are_reachable(self):
        """Verify nodes launched successfully."""
        assert True

class TestProcessOutput(unittest.TestCase):
    """Post-shutdown tests examining process behavior."""

    @launch_testing.post_shutdown_test()
    def test_graceful_shutdown(self, proc_info):
        """Verify clean process termination."""
        for proc in proc_info.processes.values():
            self.assertEqual(proc.returncode, 0)
```

### Message Passing Test Pattern

```python
@pytest.mark.launch
def test_message_passing(launch_context):
    """Verify messages flow through pipeline."""
    rclpy.init()

    messages = []

    class TestListener(rclpy.node.Node):
        def __init__(self):
            super().__init__('test_listener')
            self.sub = self.create_subscription(
                String, '/output_topic', self.callback, 10
            )

        def callback(self, msg):
            messages.append(msg.data)

    listener = TestListener()
    executor = SingleThreadedExecutor()
    executor.add_node(listener)

    # Spin for up to 5 seconds waiting for messages
    start = time.time()
    while len(messages) < 3 and (time.time() - start) < 5:
        executor.spin_once(timeout_sec=0.1)

    assert len(messages) >= 3, f"Expected 3+ messages, got {len(messages)}"

    listener.destroy_node()
    rclpy.shutdown()
```

### CMakeLists.txt Registration

```cmake
find_package(launch_testing_ament_cmake REQUIRED)

# Register integration tests with isolated domain IDs
add_ros_isolated_launch_test(
    test/test_pipeline.py
    TIMEOUT 60
)
```

---

## 4. Constitution Check Resolution

### Items Requiring Phase 1 Resolution

#### Principle III: Agent-Human Partnership
- **Issue**: Chapter MUST include AI Agent Assisted Prompts for RAG usage
- **Resolution**: Add RAG prompt sections to each lesson with:
  - Debugging prompts (e.g., "My pipeline nodes start but don't communicate")
  - Concept explanation prompts (e.g., "Explain state machine transitions")
  - Code generation prompts (e.g., "Generate a watchdog timer node")

#### Principle V: AI-Native Content
- **Issue**: Each lesson MUST include 1-2 executable code blocks
- **Resolution**: Minimum code block requirements per lesson:
  - B1-B3: 1 conceptual code block + 1 executable demo
  - I1-I3: 2 executable code blocks (build on each other)
  - A1-A3: 2 executable code blocks + 1 extension exercise

#### Principle VII: Safety & Ethics First
- **Issue**: Safety warnings for code that could cause physical harm
- **Resolution**: Add safety callout boxes:
  - Motor control sections: "SAFETY: Test in simulation before hardware"
  - Recovery logic: "CAUTION: Ensure E-stop is operational during testing"
  - Watchdog configuration: "WARNING: Incorrect timeout values may cause unexpected robot behavior"

---

## 5. ROS 2 Launch File Patterns

### Decision: Python Launch Files with Parameterization

**Rationale**: Python launch files offer full programming flexibility, type checking, and conditional logic. Aligned with FR-I01 requirements.

### Pipeline Launch Pattern

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare configurable parameters
    use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation (true) or real hardware (false)'
    )

    return LaunchDescription([
        use_sim,

        # Sensor node
        Node(
            package='workflow_examples',
            executable='lidar_sensor',
            name='lidar',
            parameters=[{
                'use_sim': LaunchConfiguration('use_sim'),
                'scan_rate_hz': 10.0,
            }],
            remappings=[
                ('scan_out', '/robot/lidar/scan'),
            ],
        ),

        # Path planner (depends on sensor)
        Node(
            package='workflow_examples',
            executable='path_planner',
            name='planner',
            parameters=[{
                'lookahead_distance': 1.5,
            }],
            remappings=[
                ('scan_in', '/robot/lidar/scan'),
                ('path_out', '/robot/path'),
            ],
        ),

        # Motor controller (depends on planner)
        Node(
            package='workflow_examples',
            executable='motor_controller',
            name='motor',
            remappings=[
                ('path_in', '/robot/path'),
                ('cmd_vel', '/robot/cmd_vel'),
            ],
        ),
    ])
```

---

## 6. Inter-Node Data Passing Patterns

### Decision: Topics for Streaming, Services for Request-Response

**Rationale**: Follows ROS 2 conventions. Topics for continuous sensor data, services for discrete operations like mode changes.

### Pattern Matrix

| Data Pattern | ROS 2 Mechanism | Use Case |
|--------------|-----------------|----------|
| Continuous stream | Topic (pub/sub) | Sensor data, state updates |
| Request-response | Service | Mode changes, parameter queries |
| Long-running task | Action | Navigation goals, motion sequences |
| Shared state | Parameters | Configuration, runtime settings |

### Service-Based Branching (FR-I02)

```python
from std_srvs.srv import SetBool
from custom_srvs.srv import SelectPath  # Custom service

class ConditionalPipeline(Node):
    def __init__(self):
        super().__init__('conditional_pipeline')

        # Service to switch pipeline branches
        self.branch_srv = self.create_service(
            SelectPath,
            'select_path',
            self.branch_callback
        )

        self.active_branch = 'default'

    def branch_callback(self, request, response):
        """Switch active processing branch."""
        if request.path_name in ['default', 'fallback', 'emergency']:
            self.active_branch = request.path_name
            response.success = True
            response.message = f'Switched to {request.path_name}'
        else:
            response.success = False
            response.message = f'Unknown path: {request.path_name}'
        return response
```

---

## 7. Mock Node Patterns for Gazebo-Free Testing

### Decision: Separate ROS 2 Package for Mock Implementations

**Rationale**: Clean separation allows students without Gazebo to follow all exercises. Mocks publish realistic data patterns.

### Mock Lidar Pattern

```python
class MockLidar(Node):
    """Mock lidar sensor for testing without Gazebo."""

    def __init__(self):
        super().__init__('mock_lidar')

        self.declare_parameter('scan_rate_hz', 10.0)
        self.declare_parameter('num_readings', 360)
        self.declare_parameter('max_range', 10.0)

        rate = self.get_parameter('scan_rate_hz').value

        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_scan)

    def publish_scan(self):
        """Publish simulated laser scan."""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_frame'
        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = 2 * math.pi / self.num_readings
        msg.range_min = 0.1
        msg.range_max = self.max_range

        # Generate realistic-looking data with some obstacles
        msg.ranges = self._generate_scan_data()

        self.publisher.publish(msg)
```

---

## References

### Official Documentation
- [ROS 2 Execution and Callbacks](https://docs.ros2.org/latest/api/rclpy/api/execution_and_callbacks.html)
- [ROS 2 Using Callback Groups](https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html)
- [ROS 2 launch_testing](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Integration.html)

### Related Patterns in Codebase
- Bridge node pattern: `chapters/02-digital-twin/advanced/src/bridge_node.py`
- Action server pattern: `chapters/01-ros2-nervous-system/code/advanced/fibonacci_action_server.py`
- Latency monitoring: `chapters/02-digital-twin/advanced/src/latency_monitor.py`

---

*Research completed: 2025-12-30*
