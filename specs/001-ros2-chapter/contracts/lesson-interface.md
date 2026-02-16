# Lesson Interface Contract: Chapter 1

**Branch**: `001-ros2-chapter` | **Date**: 2025-12-20
**Purpose**: Define the standard interface for all lessons in Chapter 1

## Lesson Markdown Structure

Every lesson file MUST follow this structure:

```markdown
---
id: {lesson_id}
title: {lesson_title}
tier: {beginner|intermediate|advanced}
chapter: chapter_1_ros2
estimated_time: {time_string}
prerequisites: [{list_of_lesson_ids}]
---

# {Lesson Title}

## Learning Objectives

By the end of this lesson, you will be able to:
- {Objective 1 - action verb}
- {Objective 2 - action verb}
- {Objective 3 - action verb}

## Introduction

{2-3 paragraphs introducing the topic}

## {Section 1 Title}

{Content with embedded code examples}

### Code Example: {Example Title}

```python
# {example_id}
{code}
```

**Expected Output**:
```
{output}
```

## {Section 2 Title}

{Content continues...}

## Diagrams

![{alt_text}](../diagrams/{diagram_file}.svg)
*{Caption}*

## Hardware Notes

> **Simulation vs. Real Hardware**: {Notes on deployment differences}

## Summary

{Key takeaways in 3-5 bullet points}

## AI-Assisted Learning

<details>
<summary>Ask the AI Assistant</summary>

### Conceptual Questions
- {Prompt 1}
- {Prompt 2}

### Debugging Help
- {Prompt 3}

### Extension Ideas
- {Prompt 4}

</details>

## Exercises

1. **{Exercise Title}** ({difficulty})
   - {Description}
   - Acceptance Criteria: {criteria}

## Next Steps

Continue to [{Next Lesson Title}](./{next_lesson_file}.md)
```

---

## Beginner Tier Lessons

### B1: Introduction to ROS 2

| Field | Value |
|-------|-------|
| File | `beginner/01-intro-to-ros2.md` |
| ID | `b_lesson1_intro` |
| Time | 1-2 hours |
| Prerequisites | None |

**Required Sections**:
- What is ROS 2?
- Why "Nervous System"?
- ROS 2 vs ROS 1
- Core Concepts (Nodes, Topics, Services, Actions)
- Installation on Ubuntu 22.04
- First Demo: Talker/Listener

**Code Examples Required**:
1. `ros2 --version` verification
2. `ros2 run demo_nodes_cpp talker`
3. `ros2 run demo_nodes_cpp listener`
4. `ros2 topic list` / `ros2 node list`

**Diagrams Required**:
1. ROS 2 Architecture Overview
2. Node-Topic-Service Communication

---

### B2: Basic Sensors Overview

| Field | Value |
|-------|-------|
| File | `beginner/02-sensors-overview.md` |
| ID | `b_lesson2_sensors` |
| Time | 1 hour |
| Prerequisites | `b_lesson1_intro` |

**Required Sections**:
- Sensor Types for Humanoid Robots
- IMU: Orientation and Motion
- LIDAR: Distance and Mapping
- Cameras: RGB and Depth
- Force Sensors: Touch and Pressure
- ROS 2 Sensor Message Types

**Code Examples Required**:
1. Sensor message structure (std_msgs, sensor_msgs)
2. Simulated sensor data echo

**Diagrams Required**:
1. Humanoid Sensor Placement

---

## Intermediate Tier Lessons

### I1: Nodes, Topics, Services, and Actions

| Field | Value |
|-------|-------|
| File | `intermediate/01-nodes-topics.md` |
| ID | `i_lesson1_nodes_topics` |
| Time | 2 hours |
| Prerequisites | `b_lesson2_sensors` |

**Required Sections**:
- Creating Python Nodes with rclpy
- Publishers and Subscribers
- Topic QoS (Quality of Service) Basics
- Services: Request/Response Pattern
- Actions: Long-Running Tasks with Feedback

**Code Examples Required**:
1. Minimal Publisher Node (complete, runnable)
2. Minimal Subscriber Node (complete, runnable)
3. Simple Service Server and Client
4. Basic Action overview (client-side)

**Diagrams Required**:
1. Pub/Sub Data Flow
2. Service Request/Response Pattern
3. Action Goal/Feedback/Result Pattern

---

### I2: Python ROS Bridge (rclpy)

| Field | Value |
|-------|-------|
| File | `intermediate/02-python-ros-bridge.md` |
| ID | `i_lesson2_python_integration` |
| Time | 1-2 hours |
| Prerequisites | `i_lesson1_nodes_topics` |

**Required Sections**:
- rclpy Library Overview
- Node Lifecycle
- Parameters: Declaration and Usage
- Launch Files: Multi-Node Orchestration
- Executors and Callback Groups

**Code Examples Required**:
1. Parameter declaration and retrieval
2. Launch file (Python style)
3. Multi-node launch example

**Diagrams Required**:
1. rclpy Node Lifecycle

---

## Advanced Tier Lessons

### A1: URDF & Humanoid Robot Description

| Field | Value |
|-------|-------|
| File | `advanced/01-urdf-humanoid.md` |
| ID | `a_lesson1_urdf_humanoid` |
| Time | 2 hours |
| Prerequisites | `i_lesson2_python_integration` |

**Required Sections**:
- URDF File Structure
- Links: Bodies and Geometry
- Joints: Connections and Constraints
- Visual and Collision Elements
- Building a Humanoid Skeleton
- RViz2 Visualization

**Code Examples Required**:
1. Basic URDF structure (XML)
2. Humanoid base-torso-head chain
3. Joint types: fixed, revolute, continuous
4. RViz2 launch command

**Diagrams Required**:
1. URDF Tree Structure
2. Joint Types Visualization

---

### A2: Advanced ROS 2 Patterns & AI Integration

| Field | Value |
|-------|-------|
| File | `advanced/02-advanced-patterns.md` |
| ID | `a_lesson2_advanced_patterns` |
| Time | 1-2 hours |
| Prerequisites | `a_lesson1_urdf_humanoid` |

**Required Sections**:
- Action Server Implementation
- Feedback Mechanisms
- Goal Handling and Cancellation
- AI Agent Integration Concepts
- Async Patterns in ROS 2

**Code Examples Required**:
1. Fibonacci Action Server (complete)
2. Fibonacci Action Client (complete)
3. Custom action definition (conceptual)

**Diagrams Required**:
1. Action Server State Machine

---

## Code Example Contract

All code examples MUST:

```yaml
code_example:
  # Required fields
  id: string           # Unique identifier
  language: enum       # python | bash | xml | yaml
  code: string         # Syntactically valid code

  # Conditional fields
  expected_output: string   # Required for runnable examples
  file_path: string         # Required for standalone files

  # Optional fields
  dependencies: string[]    # e.g., ["ros-humble-demo-nodes-cpp"]
  ros2_commands: string[]   # e.g., ["ros2 run {package} {node}"]
```

**Python Code Standards**:
```python
#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Chapter 1: {Lesson Title}
# Example: {Example Name}

import rclpy
from rclpy.node import Node
# ... imports ...

class ExampleNode(Node):
    """Brief description of what this node does."""

    def __init__(self):
        super().__init__('example_node')
        # ... initialization ...

    # ... methods ...

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
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

---

## AI Prompt Contract

All AI prompts MUST:

```yaml
ai_prompt:
  id: string           # Unique identifier
  category: enum       # conceptual | debugging | extension | real-world
  prompt: string       # The user's question
  context: string      # Background for the AI
  expected_topics:     # Topics response should cover
    - string
    - string
  tags:                # For RAG indexing
    - string           # Must include tier: #beginner, #intermediate, #advanced
    - string           # Must include topic: #ros2, #urdf, #actions, etc.
```

**Example**:
```yaml
ai_prompt:
  id: what_is_ros2
  category: conceptual
  prompt: "What is ROS 2 and why is it called a 'robot operating system' even though it's not really an OS?"
  context: "Reader is a complete beginner learning ROS 2 for the first time"
  expected_topics:
    - ROS 2 as middleware framework
    - Difference from traditional operating systems
    - Role in robot software stack
  tags:
    - "#beginner"
    - "#ros2"
    - "#fundamentals"
```

---

## Validation Checklist

Before a lesson is considered complete:

- [ ] Follows Markdown structure template
- [ ] Has YAML frontmatter with all required fields
- [ ] Learning objectives use action verbs
- [ ] All code examples are syntactically valid
- [ ] All code examples include expected output
- [ ] Python code follows standard template
- [ ] At least 2 AI prompts included
- [ ] Hardware notes section present
- [ ] Diagrams referenced exist in diagrams/ folder
- [ ] Next Steps links to correct next lesson
- [ ] Exercises have clear acceptance criteria
