---
id: beginner_exercises_workflow
title: "Beginner Tier Exercises"
sidebar_position: 53
tier: beginner
chapter: chapter_4_workflow
section: exercises
---

# Beginner Tier Exercises

This document consolidates all exercises from the Beginner tier lessons for easy reference and self-assessment.

---

## From Lesson 01: Pipelines, Flows, and Triggers

### Exercise 01.1: Pipeline Design (Beginner)

**Objective**: Design a pipeline for a robot that needs to pick up an object.

**Tasks**:

1. Identify at least 4 components needed for the task
2. Define the input and output for each component
3. Draw or describe the data flow between components
4. Identify potential failure points

**Example Components to Consider**:
- Vision system (object detection)
- Path planner (approach trajectory)
- Arm controller (grasping motion)
- Gripper controller (grip force)

**Acceptance Criteria**:
- [ ] Pipeline includes at least 4 components
- [ ] Each component has clear input/output relationships
- [ ] Data flow is logical and sequential
- [ ] At least 2 failure points identified

<details>
<summary>Solution Example</summary>

**Pipeline Design: Object Pickup**

1. **Vision System**
   - Input: RGB-D camera feed
   - Output: Object pose (position + orientation)
   - Failure: Object not detected or occluded

2. **Path Planner**
   - Input: Object pose, current robot pose
   - Output: Trajectory waypoints
   - Failure: No collision-free path found

3. **Arm Controller**
   - Input: Trajectory waypoints
   - Output: Joint commands
   - Failure: Joint limits exceeded

4. **Gripper Controller**
   - Input: Grasp command, force feedback
   - Output: Gripper position/force
   - Failure: Object slips or grip too tight

**Data Flow**: Camera → Vision → Path Planner → Arm Controller → Gripper Controller

</details>

---

### Exercise 01.2: Trigger Analysis (Intermediate)

**Objective**: Identify 3 different types of triggers that might be used in a home robot.

**Tasks**:

1. Describe a **time-based trigger** and its action
2. Describe an **event-based trigger** and its action
3. Describe a **condition-based trigger** and its action

**Example Scenario**: A home cleaning robot

**Acceptance Criteria**:
- [ ] Each trigger has a clear condition
- [ ] Each trigger has a resulting action
- [ ] Triggers are appropriate for the scenario

<details>
<summary>Solution Example</summary>

**Home Cleaning Robot Triggers**

1. **Time-Based Trigger**
   - Condition: Every day at 2:00 PM
   - Action: Start cleaning routine
   - Rationale: Scheduled cleaning when house is typically empty

2. **Event-Based Trigger**
   - Condition: User presses "clean now" button
   - Action: Immediately start cleaning current room
   - Rationale: On-demand cleaning for spills or messes

3. **Condition-Based Trigger**
   - Condition: Battery level drops below 20%
   - Action: Return to charging dock
   - Rationale: Prevent battery depletion and ensure availability

</details>

---

### Exercise 01.3: Flow Pattern Identification (Challenge)

**Objective**: Identify the appropriate flow pattern for different robotic scenarios.

**Tasks**:

For each scenario, identify whether it requires **sequential**, **parallel**, or **conditional** flow:

| Scenario | Flow Type | Justification |
|----------|-----------|---------------|
| 1. Robot must check battery, then navigate, then deliver package | | |
| 2. Robot processes camera and LIDAR data simultaneously | | |
| 3. Robot chooses path based on obstacle detection | | |
| 4. Robot must grasp object, then lift, then place | | |
| 5. Robot runs multiple sensor filters at the same time | | |

**Acceptance Criteria**:
- [ ] Correct flow type identified for each scenario
- [ ] Justification explains why that flow type is appropriate

<details>
<summary>Solution</summary>

1. **Sequential** - Tasks must happen in order; can't deliver before navigating
2. **Parallel** - Both sensors provide independent data that can be processed simultaneously
3. **Conditional** - Path selection depends on sensor readings
4. **Sequential** - Physical constraints require ordered execution
5. **Parallel** - Independent processing can happen concurrently for efficiency

</details>

---

## From Lesson 02: State Machines (Conceptual Introduction)

### Exercise 02.1: State Machine Design (Beginner)

**Objective**: Design a state machine for a robot vacuum cleaner.

**Tasks**:

1. Define at least 5 states
2. Define transitions between states
3. Identify events that trigger transitions
4. Include an error/recovery state

**States to Consider**:
- IDLE, CLEANING, CHARGING, RETURNING_TO_DOCK, ERROR, etc.

**Acceptance Criteria**:
- [ ] At least 5 states defined
- [ ] Each state has clear entry/exit conditions
- [ ] Transitions are logical and complete
- [ ] Error handling is included

<details>
<summary>Solution Example</summary>

**Robot Vacuum State Machine**

**States**:
1. IDLE - Waiting for command
2. CLEANING - Actively cleaning
3. AVOIDING_OBSTACLE - Navigating around obstacle
4. RETURNING_TO_DOCK - Going back to charge
5. CHARGING - At dock, charging battery
6. ERROR - Fault detected

**Transitions**:
- IDLE → CLEANING: User starts cleaning OR scheduled time
- CLEANING → AVOIDING_OBSTACLE: Obstacle detected
- AVOIDING_OBSTACLE → CLEANING: Obstacle cleared
- CLEANING → RETURNING_TO_DOCK: Battery low OR cleaning complete
- RETURNING_TO_DOCK → CHARGING: Dock reached
- CHARGING → IDLE: Battery full
- ANY → ERROR: Critical fault (stuck, sensor failure, etc.)
- ERROR → IDLE: User resets OR auto-recovery successful

</details>

---

### Exercise 02.2: Transition Analysis (Intermediate)

**Objective**: Identify potential issues with a robot state machine that has no error state.

**Tasks**:

1. Consider a delivery robot with states: IDLE, NAVIGATING, DELIVERING, RETURNING
2. Identify at least 3 potential failure scenarios
3. Describe how you would modify the state machine to handle each failure

**Acceptance Criteria**:
- [ ] At least 3 failure scenarios identified
- [ ] Each scenario has a proposed solution
- [ ] Solutions include new states or transitions as needed

<details>
<summary>Solution Example</summary>

**Failure Scenarios & Solutions**

1. **Scenario**: Robot gets stuck while navigating
   - **Problem**: No way to handle being stuck
   - **Solution**: Add STUCK state with transitions to RECOVERING or ERROR

2. **Scenario**: Package drops during delivery
   - **Problem**: Robot continues as if delivery succeeded
   - **Solution**: Add DELIVERY_FAILED state with transition back to NAVIGATING or RETURNING

3. **Scenario**: Communication loss with base station
   - **Problem**: Robot can't receive new commands
   - **Solution**: Add COMMUNICATION_LOST state with timeout and retry logic

**Modified State Machine**:
- Add ERROR state as catch-all
- Add RECOVERING state for automatic recovery attempts
- Add timeout transitions from any state to ERROR
- Add health monitoring that can trigger ERROR from any state

</details>

---

### Exercise 02.3: State Machine Implementation (Challenge)

**Objective**: Implement a simple traffic light state machine in Python.

**Tasks**:

1. Create a TrafficLight class with states: RED, YELLOW, GREEN
2. Implement time-based transitions
3. Add a method to get the current light color
4. Test the state machine with a simulation loop

**Acceptance Criteria**:
- [ ] State machine correctly cycles through states
- [ ] Timing is configurable
- [ ] Code is clean and well-commented

<details>
<summary>Solution Example</summary>

```python
from enum import Enum
import time

class LightState(Enum):
    RED = "red"
    YELLOW = "yellow"
    GREEN = "green"

class TrafficLight:
    def __init__(self, red_duration=60, yellow_duration=5, green_duration=45):
        self.state = LightState.RED
        self.time_in_state = 0
        self.durations = {
            LightState.RED: red_duration,
            LightState.YELLOW: yellow_duration,
            LightState.GREEN: green_duration
        }

    def update(self, dt):
        """Update state based on elapsed time"""
        self.time_in_state += dt

        # Check for state transitions
        if self.time_in_state >= self.durations[self.state]:
            self._transition_to_next_state()
            self.time_in_state = 0

    def _transition_to_next_state(self):
        """Transition to the next state in the cycle"""
        if self.state == LightState.RED:
            self.state = LightState.GREEN
        elif self.state == LightState.GREEN:
            self.state = LightState.YELLOW
        elif self.state == LightState.YELLOW:
            self.state = LightState.RED

    def get_color(self):
        """Get current light color"""
        return self.state.value

# Test the state machine
if __name__ == "__main__":
    light = TrafficLight(red_duration=3, yellow_duration=1, green_duration=2)

    print("Traffic Light Simulation (6 seconds)")
    for i in range(60):  # 60 iterations, 0.1s each = 6 seconds
        light.update(0.1)
        print(f"Time {i*0.1:.1f}s: {light.get_color()}")
        time.sleep(0.1)
```

</details>

---

## Self-Assessment Checklist

After completing all beginner exercises, verify you can:

### Pipeline Concepts
- [ ] Design a multi-component pipeline
- [ ] Identify appropriate flow patterns (sequential, parallel, conditional)
- [ ] Recognize different trigger types
- [ ] Identify failure points in a pipeline

### State Machine Concepts
- [ ] Design a state machine with multiple states
- [ ] Define transitions and events
- [ ] Include error handling in state machines
- [ ] Implement a simple state machine in Python

### Ready for Intermediate?
If you checked all boxes above, you're ready to proceed to the Intermediate tier where you'll implement workflows in ROS 2!

---

## Next Steps

Continue to [Intermediate Tier: ROS 2 Workflow Implementation](../../intermediate/README.md)
