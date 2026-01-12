---
id: b1_pipelines_flows_triggers
title: Pipelines, Flows, and Triggers
tier: beginner
chapter: chapter_4_workflow
estimated_time: 1-2 hours
prerequisites: []
---

# B1: Pipelines, Flows, and Triggers

## Learning Objectives

By the end of this lesson, you will be able to:
- Define a "process pipeline" in robotics
- Understand task sequencing and dependency ordering
- Identify components required for an end-to-end task
- Recognize different types of triggers in robotic workflows

## Introduction

In robotics, a process pipeline is a sequence of interconnected operations where the output of one stage becomes the input to the next. Think of it like an assembly line where each station performs a specific task before passing the product to the next station. Understanding pipelines is crucial for building complex robotic systems that can handle sophisticated tasks.

In this lesson, we'll explore the fundamental concepts of robotic workflows, including how tasks are sequenced, how data flows between components, and how triggers initiate different actions in the system.

## What is a Robotic Pipeline?

A robotic pipeline is a series of connected processes where data flows from one component to another in a predetermined sequence. Each component performs a specific function and passes its results to the next component in the chain.

For example, in a mobile robot navigation system:
1. **Sensor Input**: LIDAR captures distance measurements
2. **Processing**: Path planner calculates a safe route
3. **Actuation**: Motor controller executes movement commands

This creates a continuous flow where each stage depends on the output of the previous stage.

### Code Example: Simple Pipeline Simulation

```python
# pipeline_simulation.py
def lidar_sensor():
    """Simulate LIDAR sensor reading obstacles"""
    return [1.5, 2.0, 0.8, 3.2, 1.1]  # Distance measurements in meters

def path_planner(obstacles):
    """Simple path planner that checks for obstacles"""
    safe_distance = 1.0
    if min(obstacles) < safe_distance:
        return "OBSTACLE_DETECTED"
    else:
        return "CLEAR_PATH"

def motor_controller(instruction):
    """Simulate motor controller based on path planner output"""
    if instruction == "OBSTACLE_DETECTED":
        return "STOPPING_MOTORS"
    else:
        return "MOVING_FORWARD"

# Execute the pipeline
sensor_data = lidar_sensor()
planning_result = path_planner(sensor_data)
motor_command = motor_controller(planning_result)

print(f"Pipeline result: {motor_command}")
```

**Expected Output**:
```
Pipeline result: STOPPING_MOTORS
```

## Types of Data Flow

Robotic pipelines can have different types of data flow patterns:

### Sequential Flow
The most common pattern where data flows from one component to the next in a linear fashion. Each component waits for the previous component to complete before processing.

### Parallel Flow
Multiple components process data simultaneously. This is useful when you need to perform several operations on the same input data.

### Conditional Flow
The pipeline path changes based on certain conditions. For example, if an obstacle is detected, the robot might switch from "move forward" to "avoid obstacle" behavior.

### Code Example: Conditional Pipeline

```python
# conditional_pipeline.py
def lidar_sensor():
    """Simulate LIDAR sensor reading obstacles"""
    return [2.5, 3.0, 1.8, 4.2, 2.1]

def path_planner(obstacles):
    """Path planner with conditional logic"""
    safe_distance = 1.0
    if min(obstacles) < safe_distance:
        return "OBSTACLE_DETECTED"
    elif min(obstacles) < 2.0:
        return "CAUTION_ZONE"
    else:
        return "CLEAR_PATH"

def motor_controller(instruction):
    """Motor controller with multiple behaviors"""
    behavior_map = {
        "OBSTACLE_DETECTED": "EMERGENCY_STOP",
        "CAUTION_ZONE": "SLOW_DOWN",
        "CLEAR_PATH": "NORMAL_SPEED"
    }
    return behavior_map.get(instruction, "UNKNOWN")

# Execute the conditional pipeline
sensor_data = lidar_sensor()
planning_result = path_planner(sensor_data)
motor_command = motor_controller(planning_result)

print(f"Conditional pipeline result: {motor_command}")
```

**Expected Output**:
```
Conditional pipeline result: SLOW_DOWN
```

## Triggers in Robotic Workflows

Triggers are events or conditions that initiate specific actions in a robotic workflow. They can be:

- **Time-based**: Actions triggered at specific intervals or times
- **Event-based**: Actions triggered by sensor readings or external events
- **Condition-based**: Actions triggered when certain thresholds are crossed
- **Manual**: Actions triggered by user input

### Code Example: Trigger-Based Pipeline

```python
# trigger_pipeline.py
import time

def check_battery_level():
    """Simulate battery level check"""
    return 0.3  # 30% battery remaining

def check_proximity_sensor():
    """Simulate proximity sensor reading"""
    return 0.5  # 0.5 meters from obstacle

def battery_monitor():
    """Monitor battery level and trigger action if low"""
    battery_level = check_battery_level()
    if battery_level < 0.2:
        return "RETURN_TO_CHARGER"
    return None

def obstacle_detector():
    """Detect obstacles and trigger avoidance"""
    proximity = check_proximity_sensor()
    if proximity < 0.8:
        return "AVOID_OBSTACLE"
    return None

def execute_trigger_actions():
    """Execute actions based on triggers"""
    battery_action = battery_monitor()
    obstacle_action = obstacle_detector()

    if battery_action:
        return battery_action
    elif obstacle_action:
        return obstacle_action
    else:
        return "CONTINUE_NORMAL_OPERATION"

# Check for triggers
action = execute_trigger_actions()
print(f"Triggered action: {action}")
```

**Expected Output**:
```
Triggered action: AVOID_OBSTACLE
```

## Diagrams

![Pipeline Flow](../diagrams/pipeline-flow.svg)
*Basic pipeline flow showing data movement from sensors to actuators*

## Hardware Notes

> **Simulation vs. Real Hardware**: In simulation, we can easily model sensor readings and robot responses. In real hardware, you must account for sensor noise, processing delays, and the physical limitations of actuators. Real sensors may return inconsistent readings, and motors may not respond instantaneously to commands.

## Summary

- A robotic pipeline is a sequence of interconnected operations where output feeds into the next stage
- Data flows through pipelines in sequential, parallel, or conditional patterns
- Triggers initiate actions based on time, events, conditions, or manual input
- Understanding pipeline flow is essential for building complex robotic systems

## AI-Assisted Learning

<details>
<summary>Ask the AI Assistant</summary>

### Conceptual Questions
- What are the advantages and disadvantages of different pipeline flow patterns?
- How do you determine the optimal sequence for a robotic pipeline?

### Debugging Help
- How do you identify bottlenecks in a robotic pipeline?
- What happens when one component in a pipeline fails?

### Extension Ideas
- How could you implement a pipeline with feedback loops?
- What would a pipeline look like for a more complex robot with multiple sensors?

</details>

## Exercises

1. **Pipeline Design** (Beginner)
   - Design a pipeline for a robot that needs to pick up an object
   - Identify the components and their data flow
   - Acceptance Criteria: Pipeline includes at least 4 components with clear input/output relationships

2. **Trigger Analysis** (Intermediate)
   - Identify 3 different types of triggers that might be used in a home robot
   - Describe what actions each trigger would initiate
   - Acceptance Criteria: Each trigger has a clear condition and resulting action

## Next Steps

Continue to [B2: State Machines (Conceptual Introduction)](./B2-state-machines-concepts.md)