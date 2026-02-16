---
id: b2_state_machines_concepts
title: State Machines (Conceptual Introduction)
tier: beginner
chapter: chapter_4_workflow
estimated_time: 1 hour
prerequisites: ["b1_pipelines_flows_triggers"]
---

# B2: State Machines (Conceptual Introduction)

## Learning Objectives

By the end of this lesson, you will be able to:
- Define a state machine in the context of robotics
- Understand the components of a state machine: states, transitions, and events
- Recognize common state machine patterns in robotics
- Identify when to use state machines vs. simple sequential logic

## Introduction

A state machine is a computational model that represents a system with a finite number of states and transitions between those states. In robotics, state machines are essential for managing complex behaviors and ensuring that robots respond appropriately to different situations. They provide a structured way to handle various operational modes and transitions between them.

State machines are particularly useful when a robot needs to behave differently based on its current situation or the environment. For example, a delivery robot might have different behaviors when it's charging, navigating, or delivering a package.

## What is a State Machine?

A state machine consists of:

- **States**: The different conditions or modes the system can be in
- **Transitions**: The rules for moving from one state to another
- **Events**: The triggers that cause transitions to occur
- **Actions**: What the system does when entering, exiting, or while in a state

Think of a traffic light as a simple state machine:
- States: Red, Yellow, Green
- Transitions: Red → Green → Yellow → Red (in a cycle)
- Events: Timer expiration
- Actions: Display the appropriate light color

### Code Example: Simple State Machine

```python
# simple_state_machine.py
from enum import Enum

class TrafficLightState(Enum):
    RED = "red"
    YELLOW = "yellow"
    GREEN = "green"

class TrafficLight:
    def __init__(self):
        self.state = TrafficLightState.RED
        self.time_in_state = 0

    def update(self, dt):
        """Update the traffic light based on time passed"""
        self.time_in_state += dt

        # Transitions based on time in state
        if self.state == TrafficLightState.RED and self.time_in_state >= 60:
            self.state = TrafficLightState.GREEN
            self.time_in_state = 0
        elif self.state == TrafficLightState.GREEN and self.time_in_state >= 45:
            self.state = TrafficLightState.YELLOW
            self.time_in_state = 0
        elif self.state == TrafficLightState.YELLOW and self.time_in_state >= 5:
            self.state = TrafficLightState.RED
            self.time_in_state = 0

    def get_light_color(self):
        return self.state.value

# Example usage
light = TrafficLight()
print(f"Initial state: {light.get_light_color()}")

# Simulate time passing
light.update(65)  # Should transition to green
print(f"After 65 seconds: {light.get_light_color()}")

light.update(50)  # Should transition to yellow
print(f"After 50 more seconds: {light.get_light_color()}")
```

**Expected Output**:
```
Initial state: red
After 65 seconds: green
After 50 more seconds: yellow
```

## State Machines in Robotics

Robotic systems often use state machines to manage different operational modes. Common examples include:

- **Navigation Robot**: IDLE → NAVIGATING → AVOIDING_OBSTACLES → REACHED_GOAL → IDLE
- **Delivery Robot**: CHARGING → NAVIGATING → DELIVERING → RETURNING → CHARGING
- **Assembly Robot**: IDLE → PICKING → PLACING → IDLE

### Code Example: Robot Navigation State Machine

```python
# robot_state_machine.py
from enum import Enum

class RobotState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    AVOIDING_OBSTACLES = "avoiding_obstacles"
    REACHED_GOAL = "reached_goal"
    ERROR = "error"

class RobotStateMachine:
    def __init__(self):
        self.state = RobotState.IDLE
        self.destination = None
        self.obstacle_detected = False
        self.goal_reached = False

    def set_destination(self, destination):
        """Set a new destination and start navigating"""
        self.destination = destination
        if self.state == RobotState.IDLE:
            self.state = RobotState.NAVIGATING

    def detect_obstacle(self, detected):
        """Update obstacle detection status"""
        self.obstacle_detected = detected

        # Handle transitions based on obstacle detection
        if detected and self.state == RobotState.NAVIGATING:
            self.state = RobotState.AVOIDING_OBSTACLES
        elif not detected and self.state == RobotState.AVOIDING_OBSTACLES and not self.goal_reached:
            self.state = RobotState.NAVIGATING

    def reach_goal(self):
        """Mark goal as reached"""
        self.goal_reached = True
        self.state = RobotState.REACHED_GOAL

    def return_to_idle(self):
        """Return to idle state"""
        self.state = RobotState.IDLE
        self.destination = None
        self.obstacle_detected = False
        self.goal_reached = False

# Example usage
robot = RobotStateMachine()
print(f"Initial state: {robot.state.value}")

# Set destination and start navigating
robot.set_destination("kitchen")
print(f"After setting destination: {robot.state.value}")

# Detect obstacle
robot.detect_obstacle(True)
print(f"After detecting obstacle: {robot.state.value}")

# Clear obstacle
robot.detect_obstacle(False)
print(f"After clearing obstacle: {robot.state.value}")

# Reach goal
robot.reach_goal()
print(f"After reaching goal: {robot.state.value}")
```

**Expected Output**:
```
Initial state: idle
After setting destination: navigating
After detecting obstacle: avoiding_obstacles
After clearing obstacle: navigating
After reaching goal: reached_goal
```

## Types of State Machines

### Finite State Machine (FSM)
The most common type, with a finite number of states and well-defined transitions between them. Good for simple, predictable behaviors.

### Hierarchical State Machine (HSM)
States can contain sub-states, creating a hierarchy. Useful for complex behaviors that have multiple levels of detail.

### Statechart
Extends FSM with additional features like parallel states, history states, and timeouts.

## When to Use State Machines

State machines are ideal when:
- The system has a finite number of distinct operational modes
- Transitions between modes are well-defined
- The system needs to respond differently based on its current state
- You want to make the system's behavior explicit and easy to understand

State machines may not be the best choice when:
- The system's behavior is continuous rather than discrete
- There are too many possible states to enumerate
- The system needs complex decision-making logic

## Diagrams

![State Machine](../diagrams/state-machine.svg)
*State machine diagram showing states, transitions, and events*

## Hardware Notes

> **Simulation vs. Real Hardware**: In simulation, state transitions can be instantaneous and predictable. In real hardware, you must account for sensor delays, processing time, and the physical limitations of actuators. Real robots may be in a transition state for some time, and you need to handle cases where a transition fails to complete.

## Summary

- State machines represent systems with finite states and defined transitions
- They're essential for managing complex robotic behaviors
- Common patterns include navigation, delivery, and assembly workflows
- Choose state machines when you have distinct operational modes with well-defined transitions

## AI-Assisted Learning

<details>
<summary>Ask the AI Assistant</summary>

### Conceptual Questions
- What are the differences between FSM, HSM, and Statechart approaches?
- How do you handle state transitions that might fail?

### Debugging Help
- How do you debug a state machine that gets stuck in an unexpected state?
- What happens if multiple events trigger transitions simultaneously?

### Extension Ideas
- How would you implement a state machine with timeouts for each state?
- What would a hierarchical state machine look like for a complex robot?

</details>

## Exercises

1. **State Machine Design** (Beginner)
   - Design a state machine for a robot vacuum cleaner
   - Define at least 5 states and the transitions between them
   - Acceptance Criteria: Each state has clear entry/exit conditions

2. **Transition Analysis** (Intermediate)
   - Identify potential issues with a robot state machine that has no error state
   - Describe how you would modify it to handle unexpected situations
   - Acceptance Criteria: Analysis includes at least 3 potential failure scenarios

## Next Steps

Continue to [B3: Data Handoff Between Components](./B3-data-handoff.md)