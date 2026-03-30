---
id: i1_state_machines_ros2
title: "I1: State Machines in ROS 2"
sidebar_position: 55
tier: intermediate
chapter: chapter_4_workflow
estimated_time: "2-3 hours"
prerequisites: ["beginner_tier_complete"]
---

# I1: State Machines in ROS 2

> **Status**: Content in development. See [Intermediate Exercises](./exercises/intermediate-exercises.md) for hands-on practice with this topic.

## Learning Objectives

By the end of this lesson, you will be able to:
- Implement finite state machines in ROS 2 Python nodes
- Publish state information for monitoring
- Handle state transitions based on events and timers
- Integrate state machines with ROS 2 communication patterns
- Debug and test state machine implementations

## Introduction

In the beginner tier, you learned about state machines conceptually. Now, you'll implement them in ROS 2 nodes, making them practical tools for managing robot behaviors. This lesson covers the patterns and best practices for building robust state machines that integrate seamlessly with ROS 2's communication infrastructure.

## Coming Soon

This lesson is currently under development. In the meantime:

1. **Review the Beginner Content**: Ensure you understand state machine concepts from [Beginner Lesson 02](../beginner/02-state-machines-concepts.md)

2. **Practice with Exercises**: Complete [Exercise 01: Delivery Robot State Machine](./exercises/intermediate-exercises.md#exercise-01-delivery-robot-state-machine) which provides detailed implementation guidance

3. **Explore AI Prompts**: Use [Intermediate AI Prompts](../ai-prompts/intermediate-prompts.md) for implementation help

4. **Study Example Code**: The exercises include starter code and complete solutions

## Planned Topics

This lesson will cover:

### 1. Basic State Machine Implementation
- Creating state enums in Python
- Managing state in ROS 2 node classes
- State transition logic
- Publishing state for monitoring

### 2. Event-Driven Transitions
- Subscriber-based state transitions
- Timer-based state transitions
- Service-based state changes
- Action-based state management

### 3. State Machine Patterns
- Simple FSM pattern
- State with entry/exit actions
- Hierarchical state machines
- State persistence and recovery

### 4. Integration with ROS 2
- Publishing state on topics
- Logging state changes
- Parameter-based configuration
- Lifecycle node integration

### 5. Testing and Debugging
- Unit testing state machines
- Integration testing with other nodes
- Debugging state transition issues
- Visualization tools

## Resources

While this lesson is in development, use these resources:

- **ROS 2 Node Tutorial**: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
- **Python Enum Documentation**: https://docs.python.org/3/library/enum.html
- **State Machine Libraries**: https://github.com/amaury-d/smach (for reference)

## Next Steps

Continue to [I2: Multi-Node Pipelines](./02-multi-node-pipelines.md) or practice with [Intermediate Exercises](./exercises/intermediate-exercises.md)
