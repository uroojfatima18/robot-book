# Implementation Plan: Chapter 2 - Digital Twin & Simulation

**Branch**: `002-digital-twin-simulation` | **Date**: 2025-12-25 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-digital-twin-simulation/spec.md`

## Summary

Create Chapter 2 of the Physical AI & Humanoid Robotics textbook focusing on Digital Twin concepts and simulation. The chapter teaches readers to create virtual robot replicas for safe training and testing using Gazebo Classic with ROS 2 integration. Content follows a tiered approach (Beginner → Intermediate → Advanced) with hands-on code examples in Python and visual diagrams showing data flow architecture.

## Technical Context

**Language/Version**: Python 3.10+ with ROS 2 Humble (rclpy)
**Primary Dependencies**: Gazebo Classic (gazebo11), ros2_control, gazebo_ros_pkgs, sensor_msgs, geometry_msgs
**Storage**: N/A (educational content - file-based simulation assets)
**Testing**: Manual execution verification, RTF monitoring (>= 0.8), latency checks (<= 50ms)
**Target Platform**: Ubuntu 22.04, software rendering acceptable
**Project Type**: Educational textbook chapter (markdown + code + assets)
**Performance Goals**: RTF >= 0.8 (80% real-time), Bridge latency <= 50ms, Code executes on first run
**Constraints**: 2-4 hours per tier, no GPU required, Chapter 1 URDF dependency
**Scale/Scope**: 3 tiers (B/I/A), ~6 lessons, 12+ code examples, 4+ diagrams

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| **I. Embodied Learning** | PASS | All lessons demonstrate theory through Gazebo simulation with physical interaction |
| **II. Simulation-First, Reality-Ready** | PASS | Gazebo Classic primary, real-world deployment notes included in Advanced tier |
| **III. Agent-Human Partnership** | PASS | AI Agent Assisted Prompts planned for each lesson, RAG-compatible structure |
| **IV. Progressive Mastery** | PASS | B1/B2 → I1/I2 → A1/A2 tiered structure with zero assumed prerequisites at chapter start |
| **V. AI-Native Content** | PASS | Machine-readable code blocks, queryable text, 1-2 executable examples per lesson |
| **VI. ROS 2 + Python Conventions** | PASS | All code uses rclpy, follows ROS 2 conventions, Gazebo for simulation |
| **VII. Safety & Ethics First** | PASS | All code validated in simulation, safety warnings for edge cases |

**Chapter Requirements Check**:
- [x] Code blocks for immediate experimentation (FR-208)
- [x] Diagrams/Visuals for clarity (FR-212)
- [x] Mini-projects & Exercises (each tier has hands-on tasks)
- [x] AI Agent Assisted Prompts for RAG usage (planned per lesson)
- [x] Beginner → Advanced Sub-Lessons (3-tier structure)

**Gate Status**: PASSED - No violations detected.

## Project Structure

### Documentation (this feature)

```text
specs/002-digital-twin-simulation/
├── plan.md              # This file
├── research.md          # Phase 0 output - Gazebo/ROS 2 best practices
├── data-model.md        # Phase 1 output - Entity definitions
├── quickstart.md        # Phase 1 output - Reader setup guide
├── contracts/           # Phase 1 output - Message type definitions
└── tasks.md             # Phase 2 output (via /sp.tasks)
```

### Source Code (chapter content)

```text
chapters/02-digital-twin/
├── README.md                    # Chapter overview and navigation
├── beginner/
│   ├── B1-digital-twin-concepts.md    # Lesson: What is a Digital Twin
│   ├── B2-first-simulation.md         # Lesson: Running your first simulation
│   └── assets/
│       ├── humanoid_lab.world         # Pre-built world file
│       └── diagrams/
│           └── digital-twin-concept.png
├── intermediate/
│   ├── I1-building-worlds.md          # Lesson: Creating .world files
│   ├── I2-spawning-models.md          # Lesson: URDF spawning and control
│   └── assets/
│       ├── simple_lab.world           # Template world file
│       └── launch/
│           └── spawn_humanoid.launch.py
├── advanced/
│   ├── A1-data-synchronization.md     # Lesson: Bridge architecture
│   ├── A2-building-bridge.md          # Lesson: Implementing the bridge
│   └── src/
│       ├── bridge_node.py             # Digital twin bridge implementation
│       ├── latency_monitor.py         # Performance monitoring
│       └── sensor_streamer.py         # AI training data streamer
└── exercises/
    ├── exercise-01-launch-world.md
    ├── exercise-02-create-world.md
    └── exercise-03-build-bridge.md
```

**Structure Decision**: Educational textbook chapter structure with tiered subdirectories (beginner/intermediate/advanced) following constitution requirement for progressive mastery. Each tier contains lesson markdown files, assets, and source code.

## Complexity Tracking

> No violations - table not required.

## Lesson Outline

### Beginner Tier (2-4 hours)

| Lesson | Title | Learning Outcome | Code Examples |
|--------|-------|------------------|---------------|
| B1 | What is a Digital Twin? | Define digital twin, identify components | Conceptual (diagram-based) |
| B2 | Running Your First Simulation | Launch Gazebo with humanoid world | `gazebo --verbose humanoid_lab.world` |

### Intermediate Tier (2-4 hours)

| Lesson | Title | Learning Outcome | Code Examples |
|--------|-------|------------------|---------------|
| I1 | Building Simulation Worlds | Create .world file with physics/lighting | XML world file, Python launcher |
| I2 | Spawning and Controlling Models | Spawn URDF, publish joint commands | `spawn_entity.py`, joint commander |

### Advanced Tier (2-4 hours)

| Lesson | Title | Learning Outcome | Code Examples |
|--------|-------|------------------|---------------|
| A1 | Digital Twin Architecture | Design data sync patterns | Architecture diagram, topic mapping |
| A2 | Building the Bridge Node | Implement bidirectional sync | `bridge_node.py`, latency monitor |

## Key Technical Decisions

1. **Gazebo Classic over Ignition**: Mature documentation, extensive ROS 2 examples, lower barrier for beginners
2. **ODE Physics Engine**: Default in Gazebo Classic, reliable for humanoid simulation
3. **ros2_control Integration**: Standard approach for joint control, portable to real hardware
4. **50ms Latency Threshold**: Balances 20Hz control loops with network tolerance
5. **0.8 RTF Threshold**: Industry standard for physics reliability

## Dependencies

### From Chapter 1
- Valid URDF humanoid model
- ROS 2 Humble installation knowledge
- Basic Python proficiency

### External Packages
- `gazebo_ros_pkgs` - Gazebo-ROS 2 integration
- `ros2_control` - Joint control framework
- `xacro` - URDF macro processing
- `rviz2` - Visualization (optional)

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Gazebo installation issues | Provide Docker alternative in quickstart |
| URDF inertia errors | Include URDF validator script and fix guide |
| RTF degradation | Document optimization steps, headless mode option |
| ROS 2 version mismatch | Pin to Humble, document version requirements |
