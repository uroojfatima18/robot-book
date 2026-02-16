# Implementation Plan: Chapter 4 - Workflow Orchestration

**Branch**: `004-workflow-orchestration` | **Date**: 2025-12-30 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-workflow-orchestration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Chapter 4 teaches multi-component robotic workflow orchestration using ROS 2, covering state machines, pipeline coordination, and failure recovery. The chapter uses a mobile robot navigation domain (waypoint following, obstacle avoidance) with pure Python FSM patterns, structured as ROS 2 packages with colcon build workflow. Content progresses from conceptual understanding (beginner) through hands-on implementation (intermediate) to production-ready fault tolerance (advanced).

## Technical Context

**Language/Version**: Python 3.10+ (ROS 2 compatible)
**Primary Dependencies**: ROS 2 Humble/Iron, rclpy, std_msgs, std_srvs, sensor_msgs, geometry_msgs, nav_msgs, Gazebo Sim
**Storage**: YAML configuration files for ROS 2 parameters and launch configs
**Testing**: pytest for unit tests, launch_testing for ROS 2 integration tests, automated assessment tests per tier
**Target Platform**: Ubuntu 22.04 (ROS 2 Humble/Iron), Gazebo Sim for simulation, mock nodes for fallback
**Project Type**: ROS 2 workspace with multiple packages (educational content structure)
**Performance Goals**: Watchdog detection <5 seconds, pipeline timing variance <100ms, 3 retry attempts for transient failures
**Constraints**: Must work without Gazebo (mock fallbacks), all code examples buildable with colcon, no external state machine libraries
**Scale/Scope**: 3 learning tiers, 9 sub-lessons (B1-B3, I1-I3, A1-A3), 4 user stories, minimum 9 hands-on exercises

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Embodied Learning
- [x] **PASS**: All lessons demonstrate theory applied to physical systems (mobile robot navigation)
- [x] **PASS**: Pipeline examples use lidar_sensor → path_planner → motor_controller (physical interaction)
- [x] **PASS**: Every sub-lesson includes hands-on exercises with simulated robot behavior

### Principle II: Simulation-First, Reality-Ready
- [x] **PASS**: Gazebo Sim is primary simulation environment (spec.md: line 258)
- [x] **PASS**: Mock node fallbacks provided for environments without Gazebo (FR-X05)
- [x] **PASS**: Real-world deployment considerations addressed in advanced tier (watchdogs, recovery)

### Principle III: Agent-Human Partnership
- [ ] **NEEDS WORK**: Chapter MUST include AI Agent Assisted Prompts for RAG usage (not yet specified)
- Action: Add RAG prompts section during Phase 1 design

### Principle IV: Progressive Mastery
- [x] **PASS**: Beginner → Intermediate → Advanced structure defined with 3 sub-lessons each
- [x] **PASS**: No prerequisite knowledge assumed beyond Chapters 1-3
- [x] **PASS**: Each tier builds on previous (FR-X02)

### Principle V: AI-Native Content
- [ ] **NEEDS WORK**: All examples MUST be machine-readable and RAG-compatible (format TBD)
- [ ] **NEEDS WORK**: Each lesson MUST include 1-2 executable code blocks (count TBD)
- Action: Define explicit code block requirements per lesson in Phase 1

### Principle VI: ROS 2 + Python Conventions
- [x] **PASS**: Python 3.10+ with ROS 2 Humble/Iron specified
- [x] **PASS**: Gazebo Sim for simulation
- [x] **PASS**: Code examples delivered as ROS 2 packages with colcon build (FR-X06)

### Principle VII: Safety & Ethics First
- [x] **PASS**: Simulation-first validation (Principle II alignment)
- [ ] **NEEDS WORK**: Safety warnings for code that could cause physical harm (not explicitly addressed)
- Action: Add safety callouts for motor control and recovery code in Phase 1

**Gate Status**: CONDITIONAL PASS (3 items need resolution in Phase 1 design)

## Project Structure

### Documentation (this feature)

```text
specs/004-workflow-orchestration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
chapters/04-workflow-orchestration/
├── README.md                    # Chapter overview and navigation
├── content/
│   ├── beginner/
│   │   ├── B1-pipelines-flows-triggers.md
│   │   ├── B2-state-machines-concepts.md
│   │   └── B3-data-handoff.md
│   ├── intermediate/
│   │   ├── I1-launch-files.md
│   │   ├── I2-inter-node-data-passing.md
│   │   └── I3-fallback-paths.md
│   └── advanced/
│       ├── A1-watchdogs-supervisors.md
│       ├── A2-sensor-dropout-handling.md
│       └── A3-value-based-routing.md
├── code/
│   └── ros2_ws/
│       └── src/
│           ├── workflow_examples/       # ROS 2 package: example nodes
│           │   ├── package.xml
│           │   ├── setup.py
│           │   ├── workflow_examples/
│           │   │   ├── __init__.py
│           │   │   ├── simple_pipeline.py
│           │   │   ├── state_machine.py
│           │   │   ├── watchdog_supervisor.py
│           │   │   └── recovery_handler.py
│           │   ├── launch/
│           │   │   ├── pipeline_demo.launch.py
│           │   │   └── watchdog_demo.launch.py
│           │   └── config/
│           │       └── pipeline_params.yaml
│           ├── workflow_mocks/          # ROS 2 package: mock nodes (no Gazebo)
│           │   ├── package.xml
│           │   ├── setup.py
│           │   └── workflow_mocks/
│           │       ├── __init__.py
│           │       ├── mock_lidar.py
│           │       ├── mock_path_planner.py
│           │       └── mock_motor_controller.py
│           └── workflow_tests/          # ROS 2 package: automated assessments
│               ├── package.xml
│               ├── setup.py
│               ├── test/
│               │   ├── test_beginner_tier.py
│               │   ├── test_intermediate_tier.py
│               │   └── test_advanced_tier.py
│               └── workflow_tests/
│                   └── __init__.py
├── exercises/
│   ├── beginner/
│   ├── intermediate/
│   └── advanced/
├── diagrams/
│   ├── pipeline-flow.mmd
│   ├── state-machine.mmd
│   └── watchdog-architecture.mmd
└── assessments/
    ├── tier-beginner.md
    ├── tier-intermediate.md
    └── tier-advanced.md
```

**Structure Decision**: ROS 2 workspace structure under `chapters/04-workflow-orchestration/code/ros2_ws/` with three packages (examples, mocks, tests). Content follows book chapter structure with tier-based organization. This aligns with FR-X06 requiring proper ROS 2 package structure buildable with colcon.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| 3 ROS 2 packages | Separation of concerns: examples, mocks, tests | Single package would mix production examples with test infrastructure, confusing students |
| Mock nodes package | FR-X05 requires Gazebo-free fallbacks | Inline mocks in examples would bloat code and reduce clarity |
