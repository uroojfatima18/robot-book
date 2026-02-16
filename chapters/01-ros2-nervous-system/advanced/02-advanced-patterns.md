---
id: a_lesson2_advanced_patterns
title: "Advanced ROS 2 Patterns & AI Integration"
tier: advanced
chapter: chapter_1_ros2
estimated_time: "2-3 hours"
prerequisites: ["a_lesson1_urdf_humanoid"]
---

# Advanced ROS 2 Patterns & AI Integration

## Learning Objectives

By the end of this lesson, you will be able to:

- **Implement** action servers for long-running tasks
- **Create** action clients that send goals and process feedback
- **Handle** goal cancellation and completion states
- **Understand** how AI agents integrate with ROS 2 systems
- **Apply** async patterns for responsive robot applications

## Introduction

Topics and services handle most robot communication needs, but some tasks require more: long-running operations that provide progress updates and can be cancelled. Think of a humanoid robot walking to the kitchen—you want to know it's making progress, and you might need to stop it if something changes.

**Actions** are ROS 2's solution for these scenarios. They combine the goal submission of services with the streaming updates of topics, plus built-in cancellation support.

In this lesson, you'll implement a complete action server and client, understand feedback mechanisms, and learn how AI agents might use these patterns to control robots.

---

## Action Server Implementation

### Theory

An **action server** handles long-running tasks with three phases:

1. **Goal Acceptance**: Decide whether to accept the incoming request
2. **Execution**: Perform the task, publishing feedback during progress
3. **Completion**: Return the final result (success, failure, or cancelled)

```
┌─────────────────────────────────────────────────────────────┐
│                    Action Server Lifecycle                   │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│   Client                                  Server             │
│     │                                       │                │
│     │────── send_goal(target=10) ──────────▶│                │
│     │                                       │                │
│     │◀───── goal_accepted ─────────────────│                │
│     │                                       │ [executing]    │
│     │◀───── feedback(current=2) ───────────│                │
│     │◀───── feedback(current=5) ───────────│                │
│     │◀───── feedback(current=8) ───────────│                │
│     │                                       │                │
│     │◀───── result(success=true) ──────────│                │
│     │                                       │                │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Code Example: Fibonacci Action Server

This classic example computes Fibonacci numbers while providing feedback:

```python
#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Physical AI & Humanoid Robotics Textbook
#
# Chapter 1: The Robotic Nervous System (ROS 2)
# Lesson: A2 - Advanced ROS 2 Patterns & AI Integration
# Example: Fibonacci Action Server

"""
Fibonacci Action Server

This action server computes Fibonacci sequences up to a requested order.
It demonstrates:
- Goal handling and acceptance
- Feedback publishing during execution
- Result generation upon completion
- Cancellation handling
"""

import time
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):
    """Action server that computes Fibonacci sequences."""

    def __init__(self):
        super().__init__('fibonacci_action_server')

        # Create callback group for concurrent handling
        self.callback_group = ReentrantCallbackGroup()

        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Fibonacci action server ready')

    def goal_callback(self, goal_request):
        """Accept or reject incoming goals."""
        self.get_logger().info(f'Received goal request: order={goal_request.order}')

        # Validate the request
        if goal_request.order < 0:
            self.get_logger().warn('Rejecting goal: order must be non-negative')
            return GoalResponse.REJECT

        if goal_request.order > 100:
            self.get_logger().warn('Rejecting goal: order too large (max 100)')
            return GoalResponse.REJECT

        self.get_logger().info('Goal accepted')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancellation requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the Fibonacci computation."""
        self.get_logger().info('Executing goal...')

        # Initialize Fibonacci sequence
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Compute Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result = Fibonacci.Result()
                result.sequence = feedback_msg.partial_sequence
                return result

            # Compute next number
            next_num = feedback_msg.partial_sequence[-1] + feedback_msg.partial_sequence[-2]
            feedback_msg.partial_sequence.append(next_num)

            # Publish feedback
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)

            # Simulate computation time
            time.sleep(0.5)

        # Mark goal as succeeded
        goal_handle.succeed()

        # Return result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info(f'Goal succeeded: {result.sequence}')

        return result


def main(args=None):
    rclpy.init(args=args)

    node = FibonacciActionServer()

    # Use MultiThreadedExecutor for concurrent goal handling
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Action Client Implementation

### Theory

An **action client** sends goals to an action server and processes responses:

1. **Send Goal**: Submit a goal with optional feedback callback
2. **Monitor Feedback**: Receive progress updates during execution
3. **Handle Result**: Process the final outcome
4. **Cancel if Needed**: Request cancellation if circumstances change

### Code Example: Fibonacci Action Client

```python
#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Physical AI & Humanoid Robotics Textbook
#
# Chapter 1: The Robotic Nervous System (ROS 2)
# Lesson: A2 - Advanced ROS 2 Patterns & AI Integration
# Example: Fibonacci Action Client

"""
Fibonacci Action Client

This client sends goals to the Fibonacci action server and
handles feedback and results.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):
    """Action client for Fibonacci computation."""

    def __init__(self):
        super().__init__('fibonacci_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

        self.get_logger().info('Fibonacci action client ready')

    def send_goal(self, order: int):
        """Send a goal and wait for result."""
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Sending goal: order={order}')

        # Wait for server
        self._action_client.wait_for_server()

        # Send goal with feedback callback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Add callback for when goal is accepted/rejected
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Get the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the final result."""
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

        # Shutdown after receiving result
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """Handle feedback during execution."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback received: {feedback.partial_sequence}'
        )


def main(args=None):
    rclpy.init(args=args)

    client = FibonacciActionClient()

    # Send goal with order 10
    client.send_goal(10)

    rclpy.spin(client)


if __name__ == '__main__':
    main()
```

### Hands-on Exercise: Run Action Server and Client

```bash
# Terminal 1: Start the action server
python3 fibonacci_action_server.py

# Terminal 2: Start the action client
python3 fibonacci_action_client.py

# Terminal 3: Observe action topics
ros2 action list
ros2 action info /fibonacci
```

---

## Feedback Mechanisms

### Theory

**Feedback** is what makes actions powerful. It provides real-time progress updates:

- **Progress percentage**: "50% complete"
- **Current state**: "Moving to waypoint 3 of 5"
- **Sensor readings**: "Distance to goal: 2.5m"
- **Estimated time**: "ETA: 30 seconds"

### Code Example: Rich Feedback

```python
# Action definition with detailed feedback
# (This would be in a .action file)

# --- Goal ---
geometry_msgs/PoseStamped target_pose
float32 max_velocity

# --- Result ---
bool success
string message
float32 total_time

# --- Feedback ---
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
float32 percent_complete
string status_message
float32 estimated_time_remaining
```

Using rich feedback in server:

```python
def execute_navigation(self, goal_handle):
    """Execute navigation with detailed feedback."""
    feedback = NavigateToPose.Feedback()

    while not self.at_goal():
        feedback.current_pose = self.get_current_pose()
        feedback.distance_remaining = self.distance_to_goal()
        feedback.percent_complete = self.calculate_progress()
        feedback.status_message = f'Moving toward target'
        feedback.estimated_time_remaining = self.estimate_eta()

        goal_handle.publish_feedback(feedback)

        # Move toward goal
        self.move_step()

    goal_handle.succeed()
    return NavigateToPose.Result(success=True)
```

---

## Goal Handling and Cancellation

### Theory

Proper goal handling includes:

1. **Acceptance criteria**: Should we even attempt this goal?
2. **Preemption policy**: What happens if a new goal arrives?
3. **Cancellation handling**: How do we stop cleanly?

### Code Example: Advanced Goal Handling

```python
def goal_callback(self, goal_request):
    """Sophisticated goal acceptance logic."""

    # Check safety constraints
    if not self.is_safe_to_execute(goal_request):
        self.get_logger().error('Goal rejected: safety constraint violated')
        return GoalResponse.REJECT

    # Check resource availability
    if self.robot_is_charging():
        self.get_logger().warn('Goal rejected: robot is charging')
        return GoalResponse.REJECT

    # Check goal validity
    if not self.is_reachable(goal_request.target):
        self.get_logger().warn('Goal rejected: target unreachable')
        return GoalResponse.REJECT

    return GoalResponse.ACCEPT


def cancel_callback(self, goal_handle):
    """Handle cancellation with cleanup."""
    self.get_logger().info('Cancel requested')

    # Always accept cancellation for safety
    # But perform graceful stop
    self.initiate_graceful_stop()

    return CancelResponse.ACCEPT


async def execute_callback(self, goal_handle):
    """Execute with proper cancellation handling."""

    try:
        while not self.goal_reached():
            # Check for cancellation FIRST
            if goal_handle.is_cancel_requested:
                # Perform cleanup
                self.stop_motors()
                self.save_state()

                goal_handle.canceled()
                return self.create_canceled_result()

            # Continue execution
            await self.move_step_async()
            goal_handle.publish_feedback(self.get_feedback())

        goal_handle.succeed()
        return self.create_success_result()

    except Exception as e:
        self.get_logger().error(f'Execution failed: {e}')
        goal_handle.abort()
        return self.create_failure_result(str(e))
```

---

## AI Agent Integration Concepts

### Theory

AI agents (like LLM-based systems) can interact with ROS 2 robots through actions. The pattern:

1. **AI perceives** environment via topic subscriptions (camera, sensors)
2. **AI decides** on high-level action (e.g., "pick up the cup")
3. **AI sends** action goal to robot
4. **AI monitors** feedback for success/failure
5. **AI adjusts** plan based on outcome

```
┌─────────────────────────────────────────────────────────────┐
│                   AI Agent + ROS 2 Pattern                   │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐      Perception       ┌──────────────┐   │
│  │              │◀─────────────────────│   Robot      │   │
│  │   AI Agent   │   (topics: camera,   │   Hardware   │   │
│  │   (LLM +     │    sensors, etc.)    │              │   │
│  │    Planner)  │                      │              │   │
│  │              │───────────────────────▶│              │   │
│  └──────────────┘      Actions          └──────────────┘   │
│                    (goals + feedback)                       │
│                                                              │
│  Example Flow:                                              │
│  1. AI receives: "Go to kitchen and get water"              │
│  2. AI plans: [navigate_to(kitchen), find(cup), grasp()]    │
│  3. AI sends: NavigateToPose action goal                    │
│  4. AI monitors: Feedback shows 50% progress                │
│  5. AI receives: Result = success                           │
│  6. AI continues: Next action in plan                       │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Conceptual Code: AI-Robot Interface

```python
class AIRobotInterface:
    """
    Conceptual interface for AI agent to control robot.

    Note: This is a simplified example showing the pattern.
    Real AI integration would involve LLM APIs, planning systems,
    and more sophisticated error handling.
    """

    def __init__(self, node):
        self.node = node

        # Action clients for robot capabilities
        self.navigate_client = ActionClient(node, NavigateToPose, 'navigate')
        self.manipulate_client = ActionClient(node, PickPlace, 'manipulate')

        # Subscribers for perception
        self.camera_sub = node.create_subscription(
            Image, '/camera/image', self.on_image, 10
        )

    async def execute_high_level_command(self, command: str):
        """
        Execute a high-level command from AI.

        Example: "Pick up the red cup from the table"
        """
        # 1. Parse command into actions (simplified)
        actions = self.parse_command(command)

        # 2. Execute each action sequentially
        for action in actions:
            success = await self.execute_action(action)

            if not success:
                return False, f"Failed at: {action}"

        return True, "Command completed"

    async def execute_action(self, action):
        """Execute a single action and wait for result."""
        goal = self.create_goal(action)
        result = await self.send_goal_and_wait(goal)
        return result.success

    def parse_command(self, command: str):
        """
        Parse natural language into robot actions.
        In practice, this would use an LLM or semantic parser.
        """
        # Simplified example
        if "pick up" in command.lower():
            return [
                ('perceive', {'target': 'cup'}),
                ('navigate', {'target': 'detected_object'}),
                ('grasp', {'target': 'detected_object'})
            ]
        return []
```

**Key Considerations for AI Integration:**

1. **Safety**: AI should never bypass safety checks
2. **Feedback Loop**: AI needs to handle failures gracefully
3. **State Awareness**: AI must track what the robot is doing
4. **Cancellation**: Human should always be able to interrupt

---

## Async Patterns in ROS 2

### Theory

ROS 2 supports async/await for non-blocking operations:

- **Async service calls**: Don't block while waiting for response
- **Async action goals**: Continue processing while action executes
- **Concurrent execution**: Handle multiple requests simultaneously

### Code Example: Async Action Client

```python
#!/usr/bin/env python3
"""Async action client pattern."""

import asyncio
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from action_tutorials_interfaces.action import Fibonacci


class AsyncActionClient(Node):
    """Action client using async/await."""

    def __init__(self):
        super().__init__('async_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    async def send_goal_async(self, order: int):
        """Send goal and await result."""
        self.get_logger().info(f'Sending goal: order={order}')

        # Wait for server
        self._action_client.wait_for_server()

        # Create goal
        goal = Fibonacci.Goal()
        goal.order = order

        # Send goal
        goal_handle = await self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return None

        self.get_logger().info('Goal accepted')

        # Await result
        result = await goal_handle.get_result_async()
        return result.result.sequence

    def _feedback_callback(self, feedback_msg):
        """Handle feedback."""
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.partial_sequence}')


async def main_async():
    """Async main function."""
    rclpy.init()

    client = AsyncActionClient()

    # Run multiple goals concurrently
    results = await asyncio.gather(
        client.send_goal_async(5),
        client.send_goal_async(8),
    )

    for i, result in enumerate(results):
        print(f'Result {i}: {result}')

    client.destroy_node()
    rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == '__main__':
    main()
```

---

## Diagrams

![Action Server Lifecycle](../diagrams/action-lifecycle.svg)
*Figure 1: The action server lifecycle showing state transitions from goal receipt through execution to completion.*

---

## Hardware Notes

> **Simulation vs. Real Hardware**
>
> | Aspect | Development | Production |
> |--------|-------------|------------|
> | Timeouts | Generous (testing) | Tuned to hardware capability |
> | Cancellation | Always test | Critical for safety |
> | Feedback Rate | Fast (debugging) | Balanced (bandwidth) |
> | Error Handling | Log and continue | Fail-safe defaults |
>
> **Real robot tips:**
> - Always implement cancellation—humans need emergency stop
> - Test feedback at realistic rates (not just localhost)
> - Handle network failures gracefully in action clients
> - Log all goal states for debugging hardware issues

---

## Summary

In this lesson, you learned:

- **Action servers** handle long-running tasks with goals, feedback, and results
- **Action clients** send goals, monitor progress, and handle outcomes
- **Feedback mechanisms** provide real-time progress to clients
- **Goal handling** includes acceptance, preemption, and cancellation
- **AI integration** uses actions for high-level robot control
- **Async patterns** enable concurrent, non-blocking operations

---

## AI-Assisted Learning

<details>
<summary>Ask the AI Assistant</summary>

### Conceptual Questions
- "When should I use an action instead of a service? What are the tradeoffs?"
- "How do I handle multiple concurrent action goals in my robot system?"

### Debugging Help
- "My action server accepts goals but never completes. What should I check?"
- "The action client times out waiting for the server. How do I debug this?"

### Extension Ideas
- "How would I implement a preemptive action server that cancels the current goal when a new one arrives?"
- "Can I chain multiple actions together to create a behavior tree?"

</details>

---

## Exercises

### Exercise 1: Custom Action Server (Medium)

**Description**: Create an action server for a countdown timer.

**Tasks**:
1. Define the action: Goal=target count, Feedback=current count, Result=final message
2. Implement server that counts down from target to 0
3. Publish feedback every second
4. Handle cancellation

**Acceptance Criteria**:
- [ ] Goal of 10 counts down from 10 to 0
- [ ] Feedback shows current count each second
- [ ] Cancellation stops countdown and returns partial result
- [ ] Result includes completion status

### Exercise 2: Concurrent Goals (Hard)

**Description**: Handle multiple action goals simultaneously.

**Tasks**:
1. Modify the Fibonacci server to accept multiple concurrent goals
2. Use proper callback groups for concurrency
3. Test with multiple clients sending goals at the same time

**Acceptance Criteria**:
- [ ] Server handles 3+ simultaneous goals
- [ ] Each goal receives its own feedback stream
- [ ] Cancelling one goal doesn't affect others
- [ ] Results are correct for each goal

### Exercise 3: Action Client Library (Challenge)

**Description**: Create a reusable action client wrapper.

**Tasks**:
1. Create a class that wraps common action client patterns
2. Support blocking and non-blocking goal submission
3. Include timeout handling and automatic retry
4. Handle common error cases gracefully

**Acceptance Criteria**:
- [ ] Blocking: `result = client.call(goal, timeout=10.0)`
- [ ] Non-blocking: `future = client.call_async(goal, on_feedback=cb)`
- [ ] Automatic retry on transient failures
- [ ] Clear error messages for permanent failures

---

## Navigation

| Previous | Up | Next |
|----------|-----|------|
| [A1: URDF & Humanoid Robot Description](./01-urdf-humanoid.md) | [Chapter 1 Home](../README.md) | [Chapter Summary](../README.md) |

---

## Congratulations!

You've completed the Advanced tier of Chapter 1. You now have a solid foundation in ROS 2:

- **Beginner**: Concepts, installation, first demos
- **Intermediate**: Nodes, topics, services, launch files
- **Advanced**: URDF, actions, AI integration patterns

Continue exploring by building your own robot projects or move on to Chapter 2 to learn about simulation with Gazebo and Unity!
