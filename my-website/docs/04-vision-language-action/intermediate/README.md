---
id: chapter_4_intermediate_tier
title: "Intermediate Tier: ROS 2 Workflow Implementation"
sidebar_position: 54
tier: intermediate
chapter: chapter_4_workflow
estimated_time: "4-6 hours"
---

# Intermediate Tier: ROS 2 Workflow Implementation

## Welcome to the Intermediate Tier

This tier takes the conceptual understanding from the Beginner tier and puts it into practice with ROS 2. You'll implement multi-node workflows, create launch files for orchestration, and build systems that coordinate multiple components.

---

## Tier Overview

```
🟡 INTERMEDIATE TIER - Implementation & Practice
═══════════════════════════════════════════════════

What You'll Learn:
• Implementing state machines in ROS 2 nodes
• Multi-node pipeline orchestration with launch files
• Inter-node data passing patterns (topics, services, actions)
• Launch file configuration and parameter management
• Error handling and basic recovery mechanisms

What You'll Build:
• Multi-node ROS 2 workflows
• State machine-based robot behaviors
• Launch files for system orchestration
• Working navigation pipeline example
```

---

## Learning Objectives

By the end of the Intermediate tier, you will be able to:

1. **Implement** state machines in ROS 2 Python nodes
2. **Create** multi-node workflows using ROS 2 communication patterns
3. **Write** launch files to orchestrate multiple nodes
4. **Configure** nodes with parameters and remappings
5. **Handle** basic errors and implement fallback behaviors
6. **Test** and debug multi-node ROS 2 systems
7. **Build** a complete navigation workflow from scratch

---

## Prerequisites

Before starting this tier, you should have:

- **Completed Beginner Tier** of this chapter
- **ROS 2 Fundamentals**: Nodes, topics, services, actions (Chapter 1)
- **Python Programming**: Classes, inheritance, async patterns
- **Development Environment**: ROS 2 Humble with colcon build tools
- **Simulation Setup**: Gazebo or equivalent (Chapter 2-3)

**Knowledge Assumptions**: You understand workflow concepts and can write basic ROS 2 nodes.

---

## Lessons in This Tier

### Lesson 01: State Machines in ROS 2
**Duration**: 2-3 hours

How do you implement state machines in ROS 2 nodes? What patterns work best?

**Key Topics**:
- Implementing FSM in Python with ROS 2
- State management in node classes
- Publishing state information
- Transition logic and event handling
- Timer-based and callback-based transitions
- Example: Navigation state machine

**Outcomes**:
- ✅ Working state machine ROS 2 node
- ✅ State publishing for monitoring
- ✅ Clean transition logic

**File**: [01: State Machines in ROS 2](./01-state-machines-ros2.md) (Content in development - see exercises)

---

### Lesson 02: Multi-Node Pipelines with Launch Files
**Duration**: 2-3 hours

How do you orchestrate multiple nodes into a cohesive workflow? How do launch files help?

**Key Topics**:
- Launch file structure and syntax
- Starting multiple nodes with one command
- Parameter configuration in launch files
- Topic remapping for flexible connections
- Node composition and lifecycle management
- Example: Sensor → Processing → Control pipeline

**Outcomes**:
- ✅ Multi-node launch files
- ✅ Parameterized node configuration
- ✅ Working pipeline example

**File**: [02: Multi-Node Pipelines](./02-multi-node-pipelines.md) (Content in development - see exercises)

---

### Lesson 03: Inter-Node Communication Patterns
**Duration**: 1-2 hours

What are the best practices for nodes to communicate in a workflow?

**Key Topics**:
- When to use topics vs. services vs. actions
- Data passing patterns and message design
- Synchronization and timing considerations
- QoS profiles for workflow reliability
- Example: Coordinated multi-robot task

**Outcomes**:
- ✅ Understanding of communication trade-offs
- ✅ Appropriate pattern selection
- ✅ Reliable data passing

**File**: [03: Inter-Node Communication](./03-inter-node-communication.md) (Content in development - see exercises)

---

## Progression & Scaffolding

The Intermediate tier builds practical skills progressively:

```
Lesson 01                    Lesson 02                    Lesson 03
└─ State Machines in ROS 2   └─ Multi-Node Pipelines     └─ Communication Patterns
   ├─ FSM implementation        ├─ Launch files              ├─ Topics vs Services
   ├─ State publishing          ├─ Parameters                ├─ Message design
   ├─ Transitions               ├─ Remapping                 ├─ QoS profiles
   └─ Example node              └─ Example pipeline          └─ Best practices
                ↓
        Ready for Advanced Tier
     (fault tolerance & production)
```

---

## Estimated Timeline

| Lesson | Duration | Cumulative | Notes |
|--------|----------|-----------|-------|
| 01: State Machines in ROS 2 | 2-3 hours | 2-3 hours | Hands-on implementation |
| 02: Multi-Node Pipelines | 2-3 hours | 4-6 hours | Launch files and orchestration |
| 03: Inter-Node Communication | 1-2 hours | 5-8 hours | Patterns and best practices |
| **Intermediate Total** | **5-8 hours** | **5-8 hours** | Practical implementation |

---

## Hands-On Exercises

At the end of this tier, you'll complete:

- **Exercise 01**: Implement a delivery robot state machine
- **Exercise 02**: Create a multi-node sensor processing pipeline
- **Exercise 03**: Build a launch file for a complete workflow
- **Exercise 04**: Debug and fix a broken multi-node system
- **Capstone Project**: Complete navigation workflow with state machine

All exercises are in [Intermediate Exercises](./exercises/intermediate-exercises.md).

---

## AI-Assisted Learning

Stuck? Use these AI prompts to get help:

- **Implementation**: "How do I implement a state machine in a ROS 2 Python node?"
- **Debugging**: "My launch file isn't starting all nodes. How do I debug this?"
- **Design**: "Should I use a topic or service for this workflow communication?"
- **Optimization**: "How can I improve the reliability of my multi-node pipeline?"

See [Intermediate AI Prompts](../ai-prompts/intermediate-prompts.md) for a full library.

---

## What's Next?

After completing this tier:

1. **Review** your implementations and ensure they work correctly
2. **Complete** all exercises and the capstone project
3. **Test** your workflows in simulation
4. **Move Forward** to the **Advanced Tier** for fault tolerance and production-ready systems

The Advanced tier assumes you can build working multi-node workflows and focuses on making them robust and production-ready.

---

## Resources

- **ROS 2 Launch System**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html
- **ROS 2 Actions Tutorial**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
- **QoS Profiles**: https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html
- **State Machine Libraries**: https://github.com/amaury-d/smach

---

## Ready to Start?

Begin with **[Lesson 01: State Machines in ROS 2](./01-state-machines-ros2.md)**.

---

*"Theory becomes practice. Let's build real workflows."*
