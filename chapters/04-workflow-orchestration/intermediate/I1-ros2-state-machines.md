---
id: i1_ros2_state_machines
title: Implementing ROS 2 State Machines
tier: intermediate
chapter: chapter_4_workflow
estimated_time: 60-90 minutes
prerequisites: ["b1_pipelines_flows_triggers", "b2_state_machines_concepts", "b3_data_handoff"]
---

# I1: Implementing ROS 2 State Machines

## Learning Objectives

By the end of this lesson, you will be able to:
- Implement finite state machines in Python ROS 2 nodes
- Manage state transitions based on ROS 2 messages
- Publish state information for monitoring
- Integrate state machines with navigation systems
- Handle errors and implement recovery behaviors

## Introduction

State machines are essential for managing complex robot behaviors. In this lesson, you'll learn to implement production-ready state machines in ROS 2 that can control navigation workflows, handle failures, and coordinate multiple components.

We'll build a complete navigation state machine that manages waypoint following, obstacle avoidance, and error recovery - a pattern used in real warehouse robots and autonomous vehicles.

## Basic State Machine Pattern

Let's start with a simple state machine structure in ROS 2:

### Code Example: Basic FSM Node

```python
# basic_fsm_node.py
import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import String

class RobotState(Enum):
    """Enumeration of possible robot states"""
    IDLE = "idle"
    NAVIGATING = "navigating"
    REACHED_GOAL = "reached_goal"
    ERROR = "error"

class BasicFSM(Node):
    """Basic finite state machine node"""

    def __init__(self):
        super().__init__('basic_fsm')

        # Initialize state
        self.state = RobotState.IDLE

        # Publisher for state information
        self.state_pub = self.create_publisher(
            String,
            '/robot_state',
            10
        )

        # Timer for state machine updates
        self.timer = self.create_timer(0.5, self.state_machine_update)

        self.get_logger().info('Basic FSM initialized')

    def state_machine_update(self):
        """Main state machine loop"""
        # Publish current state
        state_msg = String()
        state_msg.data = self.state.value
        self.state_pub.publish(state_msg)

        # State-specific logic
        if self.state == RobotState.IDLE:
            self.handle_idle_state()
        elif self.state == RobotState.NAVIGATING:
            self.handle_navigating_state()
        elif self.state == RobotState.REACHED_GOAL:
            self.handle_reached_goal_state()
        elif self.state == RobotState.ERROR:
            self.handle_error_state()

    def handle_idle_state(self):
        """Handle IDLE state logic"""
        self.get_logger().info('State: IDLE - Waiting for goal',
                              throttle_duration_sec=2.0)

    def handle_navigating_state(self):
        """Handle NAVIGATING state logic"""
        self.get_logger().info('State: NAVIGATING - Moving to goal',
                              throttle_duration_sec=2.0)

    def handle_reached_goal_state(self):
        """Handle REACHED_GOAL state logic"""
        self.get_logger().info('State: REACHED_GOAL - Goal achieved',
                              throttle_duration_sec=2.0)

    def handle_error_state(self):
        """Handle ERROR state logic"""
        self.get_logger().error('State: ERROR - Recovery needed',
                               throttle_duration_sec=2.0)

    def transition_to(self, new_state):
        """Transition to a new state with logging"""
        old_state = self.state
        self.state = new_state
        self.get_logger().info(
            f'State transition: {old_state.value} -> {new_state.value}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = BasicFSM()

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

**To run this example:**
```bash
# In terminal 1: Run the FSM node
ros2 run workflow_orchestration basic_fsm_node

# In terminal 2: Monitor state
ros2 topic echo /robot_state
```

## Navigation State Machine

Now let's build a complete navigation state machine that integrates with Nav2:

### Code Example: Navigation FSM

```python
# navigation_fsm.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import time

class NavigationState(Enum):
    """Navigation workflow states"""
    IDLE = "idle"
    WAITING_FOR_GOAL = "waiting_for_goal"
    NAVIGATING = "navigating"
    GOAL_REACHED = "goal_reached"
    NAVIGATION_FAILED = "navigation_failed"
    RECOVERING = "recovering"

class NavigationFSM(Node):
    """Navigation state machine with Nav2 integration"""

    def __init__(self):
        super().__init__('navigation_fsm')

        # State management
        self.state = NavigationState.IDLE
        self.previous_state = None

        # Nav2 action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Goal management
        self.current_goal = None
        self.goal_handle = None
        self.retry_count = 0
        self.max_retries = 3

        # Publishers
        self.state_pub = self.create_publisher(
            String,
            '/navigation_state',
            10
        )

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Timer for state machine
        self.timer = self.create_timer(0.1, self.state_machine_update)

        self.get_logger().info('Navigation FSM initialized')
        self.transition_to(NavigationState.WAITING_FOR_GOAL)

    def state_machine_update(self):
        """Main state machine loop"""
        # Publish current state
        state_msg = String()
        state_msg.data = self.state.value
        self.state_pub.publish(state_msg)

        # State handlers
        if self.state == NavigationState.IDLE:
            pass  # Do nothing in idle

        elif self.state == NavigationState.WAITING_FOR_GOAL:
            self.get_logger().info(
                'Waiting for goal on /goal_pose',
                throttle_duration_sec=5.0
            )

        elif self.state == NavigationState.NAVIGATING:
            # Check navigation progress
            if self.goal_handle is not None:
                if not self.goal_handle.is_active:
                    # Navigation completed or failed
                    result = self.goal_handle.get_result()
                    if result is not None:
                        self.handle_navigation_result(result)

        elif self.state == NavigationState.GOAL_REACHED:
            self.get_logger().info('Goal reached successfully!')
            self.transition_to(NavigationState.WAITING_FOR_GOAL)

        elif self.state == NavigationState.NAVIGATION_FAILED:
            self.get_logger().warn(
                f'Navigation failed (attempt {self.retry_count}/{self.max_retries})'
            )
            if self.retry_count < self.max_retries:
                self.transition_to(NavigationState.RECOVERING)
            else:
                self.get_logger().error('Max retries exceeded, giving up')
                self.transition_to(NavigationState.WAITING_FOR_GOAL)
                self.retry_count = 0

        elif self.state == NavigationState.RECOVERING:
            self.get_logger().info('Attempting recovery...')
            time.sleep(2.0)  # Simple recovery delay
            self.retry_count += 1
            # Retry navigation
            if self.current_goal is not None:
                self.send_navigation_goal(self.current_goal)

    def goal_callback(self, msg):
        """Handle new goal received"""
        if self.state == NavigationState.WAITING_FOR_GOAL:
            self.get_logger().info(
                f'New goal received: ({msg.pose.position.x:.2f}, '
                f'{msg.pose.position.y:.2f})'
            )
            self.current_goal = msg
            self.retry_count = 0
            self.send_navigation_goal(msg)
        else:
            self.get_logger().warn(
                f'Goal received but robot is in {self.state.value} state'
            )

    def send_navigation_goal(self, goal_pose):
        """Send navigation goal to Nav2"""
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            self.transition_to(NavigationState.NAVIGATION_FAILED)
            return

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Send goal
        self.get_logger().info('Sending navigation goal to Nav2')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            self.transition_to(NavigationState.NAVIGATION_FAILED)
            return

        self.get_logger().info('Goal accepted, navigating...')
        self.transition_to(NavigationState.NAVIGATING)

        # Get result
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        self.handle_navigation_result(result)

    def handle_navigation_result(self, result):
        """Process navigation result"""
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded!')
            self.transition_to(NavigationState.GOAL_REACHED)
        else:
            self.get_logger().warn(f'Navigation failed with status: {result.status}')
            self.transition_to(NavigationState.NAVIGATION_FAILED)

    def transition_to(self, new_state):
        """Transition to new state with logging"""
        if self.state != new_state:
            self.previous_state = self.state
            self.state = new_state
            self.get_logger().info(
                f'State transition: {self.previous_state.value} -> {new_state.value}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = NavigationFSM()

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

**To run this example:**
```bash
# Terminal 1: Launch Nav2 (from Chapter 3)
ros2 launch nav2_bringup tb3_simulation_launch.py

# Terminal 2: Run navigation FSM
ros2 run workflow_orchestration navigation_fsm

# Terminal 3: Send a goal
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}"

# Terminal 4: Monitor state
ros2 topic echo /navigation_state
```

## State Visualization

Create a simple visualization tool to monitor state transitions:

### Code Example: State Monitor

```python
# state_monitor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class StateMonitor(Node):
    """Monitor and log state transitions"""

    def __init__(self):
        super().__init__('state_monitor')

        self.current_state = None
        self.state_history = []

        self.subscription = self.create_subscription(
            String,
            '/navigation_state',
            self.state_callback,
            10
        )

        self.get_logger().info('State monitor started')

    def state_callback(self, msg):
        """Handle state updates"""
        new_state = msg.data

        if new_state != self.current_state:
            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

            # Log transition
            if self.current_state is not None:
                self.get_logger().info(
                    f'[{timestamp}] {self.current_state} -> {new_state}'
                )
            else:
                self.get_logger().info(f'[{timestamp}] Initial state: {new_state}')

            # Update history
            self.state_history.append({
                'timestamp': timestamp,
                'from': self.current_state,
                'to': new_state
            })

            self.current_state = new_state

            # Print statistics
            if len(self.state_history) % 10 == 0:
                self.print_statistics()

    def print_statistics(self):
        """Print state transition statistics"""
        self.get_logger().info(f'Total transitions: {len(self.state_history)}')

        # Count states
        state_counts = {}
        for entry in self.state_history:
            state = entry['to']
            state_counts[state] = state_counts.get(state, 0) + 1

        self.get_logger().info('State distribution:')
        for state, count in state_counts.items():
            self.get_logger().info(f'  {state}: {count} times')

def main(args=None):
    rclpy.init(args=args)
    node = StateMonitor()

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

## Best Practices

### 1. State Design
- Keep states simple and focused
- Avoid too many states (5-10 is typical)
- Make transitions explicit and well-defined
- Document state meanings and transitions

### 2. Error Handling
- Always have an ERROR state
- Implement recovery behaviors
- Log all errors with context
- Set maximum retry limits

### 3. Monitoring
- Publish state for external monitoring
- Log all state transitions
- Track state duration
- Collect statistics

### 4. Testing
- Test each state independently
- Test all transitions
- Test error conditions
- Test recovery behaviors

## Hardware Notes

> **Simulation vs. Real Hardware**: State machines work the same in simulation and reality, but timing may differ. Real hardware has sensor delays, actuator lag, and network latency. Always test state transitions with realistic timing and add timeout handling for production systems.

## Summary

- State machines manage complex robot behaviors in ROS 2
- Use enums for type-safe state definitions
- Publish state for monitoring and debugging
- Integrate with Nav2 using action clients
- Implement error handling and recovery
- Test thoroughly before deployment

## Exercises

1. **Add Battery Monitoring** (Intermediate)
   - Add a LOW_BATTERY state
   - Transition to charging when battery is low
   - Resume navigation after charging
   - Acceptance Criteria: Robot returns to dock when battery < 20%

2. **Implement Timeout Handling** (Advanced)
   - Add timeout for navigation
   - Transition to NAVIGATION_FAILED if timeout exceeded
   - Make timeout configurable via parameter
   - Acceptance Criteria: Navigation fails after 60 seconds

## Next Steps

Continue to [I2: Multi-Node Workflow Coordination](./I2-multi-node-coordination.md) to learn how to coordinate multiple nodes in a workflow.
