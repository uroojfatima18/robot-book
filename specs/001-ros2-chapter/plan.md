# Implementation Plan: Chapter 1 - The Robotic Nervous System (ROS 2)

**Branch**: `001-ros2-chapter` | **Date**: 2025-12-20 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-ros2-chapter/spec.md`

## Summary

This plan defines the implementation approach for Chapter 1 of the Physical AI & Humanoid Robotics textbook. The chapter introduces ROS 2 as the "nervous system" of robots, progressing from beginner concepts through advanced URDF modeling and AI agent integration. Content will be structured as Markdown files with embedded code examples, diagrams, and AI-assisted prompts following the constitution's AI-native requirements.

## Technical Context

**Content Format**: Markdown with embedded Python code blocks
**Primary Stack**: ROS 2 Humble/Iron, Python 3.10+, rclpy
**Simulation**: Gazebo Classic/Fortress, RViz2
**Storage**: Git repository with Markdown files
**Testing**: Code examples validated on Ubuntu 22.04 with ROS 2 installed
**Target Platform**: Ubuntu 22.04 LTS (primary), WSL2 for Windows, Docker for macOS
**Project Type**: Educational content (textbook chapter)
**Performance Goals**: All code examples execute within 30 seconds, lessons completable in 2-4 hours per tier
**Constraints**: No unsafe robot commands, simulation-first approach per constitution
**Scale/Scope**: 6 lessons across 3 tiers, ~50 pages of content, 15+ code examples

## Content Standards

*Added based on spec clarifications (2025-12-23)*

### Lesson Structure

Each lesson MUST follow this standardized format:
1. **Theory** - Conceptual explanation with diagrams
2. **Code Example** - Working, executable code demonstrating the concept
3. **Hands-on Exercise** - Reader practice activity
4. **Summary** - Key takeaways and next steps

### Accessibility Requirements

- All diagrams MUST include descriptive alt-text for screen readers
- Diagrams MUST use color-blind safe palettes (avoid red-green only distinctions)
- Content structure MUST use proper heading hierarchy for screen reader navigation
- Code examples MUST not rely solely on color for syntax highlighting

### Glossary

A consolidated glossary of ROS 2 terms (nodes, topics, services, actions, URDF, etc.) MUST appear at the chapter start before any lessons.


## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Requirement | Status | Notes |
|-----------|-------------|--------|-------|
| I. Embodied Learning | Concepts translate to simulation actions | PASS | All code runs in Gazebo/RViz |
| II. Simulation-First | Start in Gazebo, include real-world notes | PASS | Hardware deployment notes in each lesson |
| III. Agent-Human Partnership | AI Agent Assisted Prompts included | PASS | RAG prompts per lesson planned |
| IV. Progressive Mastery | Beginner → Intermediate → Advanced | PASS | 3-tier structure defined |
| V. AI-Native Content | Machine-readable, RAG-compatible | PASS | Markdown with semantic headings |
| VI. ROS 2 + Python | Follow ROS 2/Python conventions | PASS | rclpy standard patterns used |
| VII. Safety First | No unsafe commands, simulation first | PASS | All examples validated in sim |

**Gate Status**: PASSED - No violations requiring justification

## Project Structure

### Documentation (this feature)

```text
specs/001-ros2-chapter/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output (content structure)
├── quickstart.md        # Phase 1 output (reader guide)
├── contracts/           # Phase 1 output (lesson interfaces)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
chapters/
└── 01-ros2-nervous-system/
    ├── README.md                    # Chapter landing page
    ├── glossary.md                  # ROS 2 terminology glossary (consolidated, at chapter start)
    ├── introduction.md              # Chapter introduction
    │
    ├── beginner/
    │   ├── 01-intro-to-ros2.md      # B1: Introduction to ROS 2
    │   ├── 02-sensors-overview.md   # B2: Basic Sensors Overview
    │   └── exercises/
    │       └── beginner-exercises.md
    │
    ├── intermediate/
    │   ├── 01-nodes-topics.md       # I1: Nodes, Topics, Services, Actions
    │   ├── 02-python-ros-bridge.md  # I2: Python ROS Bridge (rclpy)
    │   └── exercises/
    │       └── intermediate-exercises.md
    │
    ├── advanced/
    │   ├── 01-urdf-humanoid.md      # A1: URDF & Humanoid Robot Description
    │   ├── 02-advanced-patterns.md  # A2: Advanced ROS 2 Patterns & AI Integration (conceptual)
    │   └── exercises/
    │       └── advanced-exercises.md
    │
    ├── code/
    │   ├── beginner/
    │   │   └── demo_commands.sh
    │   ├── intermediate/
    │   │   ├── minimal_publisher.py
    │   │   ├── minimal_subscriber.py
    │   │   ├── simple_service.py
    │   │   └── launch/
    │   │       └── talker_listener_launch.py
    │   └── advanced/
    │       ├── fibonacci_action_server.py
    │       ├── fibonacci_action_client.py
    │       └── urdf/
    │           └── humanoid_basic.urdf
    │
    ├── diagrams/
    │   ├── ros2-architecture.svg
    │   ├── node-topic-service.svg
    │   ├── humanoid-sensor-placement.svg
    │   └── urdf-structure.svg
    │
    └── ai-prompts/
        ├── beginner-prompts.md
        ├── intermediate-prompts.md
        └── advanced-prompts.md
```

**Structure Decision**: Educational content structure with tiered lesson folders, shared code directory, and AI prompts directory per constitution requirements.

## Complexity Tracking

> No violations requiring justification - structure is minimal and appropriate for educational content.

## Implementation Phases

### Phase 0: Research & Preparation

**Objective**: Resolve all technical unknowns and establish best practices

| Research Topic | Decision | Rationale |
|----------------|----------|-----------|
| ROS 2 Distribution | Humble LTS (primary), Iron (secondary) | Humble is LTS until 2027, widest adoption |
| Installation Method | Binary apt packages | Simplest for beginners, consistent across readers |
| URDF vs XACRO | URDF (chapter 1), XACRO mentioned | URDF simpler for learning, XACRO for later chapters |
| Gazebo Version | Gazebo Fortress (recommended), Classic (supported) | Fortress is current, Classic for legacy compatibility |
| Code Organization | ROS 2 package structure not required | Chapter focuses on concepts, not package creation |
| AI Integration Pattern | Conceptual overview only | Working AI code deferred to future chapters |

### Phase 1: Content Design

**Lesson Flow Architecture**:

```
Chapter 1: The Robotic Nervous System (ROS 2)
│
├── Introduction (15 min read)
│   ├── What is ROS 2?
│   ├── Why "Nervous System"?
│   └── Learning Outcomes
│
├── BEGINNER TIER (2-4 hours)
│   ├── B1: Introduction to ROS 2 (1-2 hours)
│   │   ├── ROS 2 Ecosystem Overview
│   │   ├── Architecture: Nodes, Topics, Services, Actions
│   │   ├── Installation on Ubuntu 22.04
│   │   └── First Demo: Talker/Listener
│   │
│   └── B2: Basic Sensors Overview (1 hour)
│       ├── IMU: Orientation & Motion
│       ├── LIDAR: Distance & Mapping
│       ├── Cameras: RGB & Depth
│       └── Force Sensors: Touch & Pressure
│
├── INTERMEDIATE TIER (2-4 hours)
│   ├── I1: Nodes, Topics, Services, Actions (2 hours)
│   │   ├── Creating Python Nodes
│   │   ├── Publishing & Subscribing
│   │   ├── Services: Request/Response
│   │   └── Actions: Long-Running Tasks
│   │
│   └── I2: Python ROS Bridge (1-2 hours)
│       ├── rclpy Library Deep Dive
│       ├── Parameters & Configuration
│       └── Launch Files
│
└── ADVANCED TIER (2-4 hours)
    ├── A1: URDF & Humanoid Description (2 hours)
    │   ├── URDF File Structure
    │   ├── Links & Joints
    │   ├── Humanoid Skeleton Design
    │   └── RViz2 Visualization
    │
    └── A2: Advanced Patterns & AI (1-2 hours)
        ├── Action Server Implementation
        ├── Feedback Mechanisms
        └── AI Agent Integration Concepts
```

### Phase 2: Implementation Tasks

**Deferred to `/sp.tasks` command** - This plan establishes structure; tasks.md will break down implementation steps.

## Deliverables Checklist

| Artifact | Status | Path |
|----------|--------|------|
| plan.md | Complete | specs/001-ros2-chapter/plan.md |
| research.md | Complete | specs/001-ros2-chapter/research.md |
| data-model.md | Complete | specs/001-ros2-chapter/data-model.md |
| quickstart.md | Complete | specs/001-ros2-chapter/quickstart.md |
| contracts/ | Complete | specs/001-ros2-chapter/contracts/ |
| tasks.md | Deferred | specs/001-ros2-chapter/tasks.md |

## Risk Analysis

| Risk | Mitigation |
|------|------------|
| ROS 2 version differences | Document version-specific notes, test on Humble & Iron |
| Installation failures on reader systems | Include troubleshooting section, VM fallback guide |
| Code examples becoming outdated | Pin ROS 2 distribution, include deprecation warnings |
| Non-Ubuntu readers excluded | Provide Docker/WSL2/VM setup alternatives |
| Gazebo version fragmentation | Support both Classic and Fortress with notes |
| Accessibility gaps | Follow NFR-001 to NFR-004 for all content |

## Next Steps

Phase 0-1 artifacts complete. Ready for Phase 2:

1. Run `/sp.tasks` to generate implementation tasks
2. Begin content authoring following contracts and Content Standards
3. Create diagrams per data-model specifications (with alt-text, color-blind safe)
4. Implement AI prompts per lesson requirements
