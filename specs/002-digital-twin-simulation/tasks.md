# Tasks: Chapter 2 - Digital Twin & Simulation

**Feature Branch**: `002-digital-twin-simulation`
**Generated**: 2025-12-25
**Completed**: 2025-12-25
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 32 |
| Completed | 32 |
| Setup Phase | 4 tasks |
| Foundational Phase | 3 tasks |
| User Story 1 (P1) | 6 tasks |
| User Story 2 (P2) | 7 tasks |
| User Story 3 (P3) | 6 tasks |
| User Story 4 (P4) | 4 tasks |
| Polish Phase | 2 tasks |
| Parallel Opportunities | 12 tasks marked [P] |

## MVP Scope

**Suggested MVP**: User Story 1 (Simulation Beginner)
- Complete Phases 1-3 for a functional beginner tutorial
- Reader can launch Gazebo and understand digital twin concepts
- Estimated effort: 4-6 hours content creation

---

## Phase 1: Setup

> Project initialization and directory structure creation.

- [x] T001 Create chapter directory structure per plan in chapters/02-digital-twin/
- [x] T002 [P] Create chapter README.md with navigation and overview in chapters/02-digital-twin/README.md
- [x] T003 [P] Create beginner tier subdirectory structure in chapters/02-digital-twin/beginner/
- [x] T004 [P] Create intermediate tier subdirectory structure in chapters/02-digital-twin/intermediate/

---

## Phase 2: Foundational

> Blocking prerequisites that all user stories depend on.

- [x] T005 Create assets directory with placeholder structure in chapters/02-digital-twin/beginner/assets/
- [x] T006 [P] Create advanced tier subdirectory structure in chapters/02-digital-twin/advanced/
- [x] T007 [P] Create exercises directory structure in chapters/02-digital-twin/exercises/

---

## Phase 3: User Story 1 - Simulation Beginner (P1)

> **Goal**: Reader with zero simulation experience understands digital twin concepts and runs a premade simulation.
>
> **Independent Test**: User launches Gazebo and loads demo world successfully, can explain digital twin concepts to a peer.

### Lesson Content

- [x] T008 [US1] Write B1 lesson: What is a Digital Twin - concepts and mental model in chapters/02-digital-twin/beginner/B1-digital-twin-concepts.md
- [x] T009 [P] [US1] Create digital twin concept diagram showing physical-virtual synchronization in chapters/02-digital-twin/beginner/assets/diagrams/digital-twin-concept.png
- [x] T010 [US1] Write B2 lesson: Running Your First Simulation - Gazebo launch tutorial in chapters/02-digital-twin/beginner/B2-first-simulation.md
- [x] T011 [P] [US1] Create humanoid_lab.world pre-built world file for beginner demo in chapters/02-digital-twin/beginner/assets/humanoid_lab.world

### Exercise

- [x] T012 [US1] Create Exercise 01: Launch World - guided practice in chapters/02-digital-twin/exercises/exercise-01-launch-world.md

### AI Prompts

- [x] T013 [P] [US1] Add AI Agent Assisted Prompts section to B1 and B2 lessons for RAG usage

---

## Phase 4: User Story 2 - Developer Creating Simulation Worlds (P2)

> **Goal**: Reader builds a world file from scratch, spawns a humanoid model, and makes it interact with physics.
>
> **Independent Test**: User launches their own `.world` file and spawns a URDF from terminal with working physics.
>
> **Dependency**: Requires US1 completion (understanding of basic concepts).

### Lesson Content

- [x] T014 [US2] Write I1 lesson: Building Simulation Worlds - world file creation in chapters/02-digital-twin/intermediate/I1-building-worlds.md
- [x] T015 [P] [US2] Create simple_lab.world template for students to reference in chapters/02-digital-twin/intermediate/assets/simple_lab.world
- [x] T016 [US2] Write I2 lesson: Spawning and Controlling Models - URDF spawning and joint control in chapters/02-digital-twin/intermediate/I2-spawning-models.md
- [x] T017 [P] [US2] Create spawn_humanoid.launch.py ROS 2 launch file in chapters/02-digital-twin/intermediate/assets/launch/spawn_humanoid.launch.py

### Code Examples

- [x] T018 [P] [US2] Create joint_commander.py example for publishing joint commands in chapters/02-digital-twin/intermediate/assets/src/joint_commander.py

### Exercise

- [x] T019 [US2] Create Exercise 02: Create World - hands-on world building in chapters/02-digital-twin/exercises/exercise-02-create-world.md

### AI Prompts

- [x] T020 [P] [US2] Add AI Agent Assisted Prompts section to I1 and I2 lessons for RAG usage

---

## Phase 5: User Story 3 - Humanoid Engineer Building Digital Twin Loop (P3)

> **Goal**: Reader connects simulation output to ROS 2 nodes to emulate real robot data with bidirectional sync.
>
> **Independent Test**: Data is mirrored between simulated and physical robot nodes with latency < 50ms.
>
> **Dependency**: Requires US2 completion (simulation world and model spawning).

### Lesson Content

- [x] T021 [US3] Write A1 lesson: Digital Twin Architecture - data synchronization patterns in chapters/02-digital-twin/advanced/A1-data-synchronization.md
- [x] T022 [US3] Write A2 lesson: Building the Bridge Node - implementation tutorial in chapters/02-digital-twin/advanced/A2-building-bridge.md

### Code Implementation

- [x] T023 [P] [US3] Implement bridge_node.py with bidirectional topic sync in chapters/02-digital-twin/advanced/src/bridge_node.py
- [x] T024 [P] [US3] Implement latency_monitor.py with 50ms threshold warnings in chapters/02-digital-twin/advanced/src/latency_monitor.py

### Exercise

- [x] T025 [US3] Create Exercise 03: Build Bridge - digital twin loop exercise in chapters/02-digital-twin/exercises/exercise-03-build-bridge.md

### AI Prompts

- [x] T026 [P] [US3] Add AI Agent Assisted Prompts section to A1 and A2 lessons for RAG usage

---

## Phase 6: User Story 4 - AI Behavioral Training in Simulation (P4)

> **Goal**: Reader prepares simulation environment for future AI training pipelines.
>
> **Independent Test**: Simulation outputs are measurable and streamable for external ML systems.
>
> **Dependency**: Requires US3 completion (working bridge node).

### Lesson Content (Extended A2)

- [x] T027 [US4] Add AI training preparation section to A2 lesson with control graph in chapters/02-digital-twin/advanced/A2-building-bridge.md

### Code Implementation

- [x] T028 [P] [US4] Implement sensor_streamer.py for streaming data to external API in chapters/02-digital-twin/advanced/src/sensor_streamer.py

### Documentation

- [x] T029 [US4] Create AI training architecture diagram showing Sensors → ROS → AI → Actuators in chapters/02-digital-twin/advanced/assets/diagrams/ai-training-architecture.md
- [x] T030 [P] [US4] Add RL integration guide section explaining future pipeline attachment in chapters/02-digital-twin/advanced/A2-building-bridge.md

---

## Phase 7: Polish & Cross-Cutting Concerns

> Final cleanup and cross-chapter consistency.

- [x] T031 Update chapter README.md with complete navigation links in chapters/02-digital-twin/README.md
- [x] T032 Add troubleshooting section covering edge cases (RTF < 0.8, URDF inertia, latency) to each tier in chapters/02-digital-twin/

---

## Dependencies

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational)
    ↓
Phase 3 (US1: Beginner) ──────────────────────────┐
    ↓                                              │
Phase 4 (US2: Intermediate) ──────────────────────┤ Can run in parallel
    ↓                                              │ after dependencies met
Phase 5 (US3: Advanced) ──────────────────────────┤
    ↓                                              │
Phase 6 (US4: AI Training) ───────────────────────┘
    ↓
Phase 7 (Polish)
```

### User Story Dependencies

| Story | Depends On | Can Start After |
|-------|------------|-----------------|
| US1 | Setup, Foundational | T007 complete |
| US2 | US1 | T013 complete |
| US3 | US2 | T020 complete |
| US4 | US3 | T026 complete |

---

## Parallel Execution Examples

### Within Phase 3 (US1)
```
T008 (B1 lesson) ─────────────────────┐
                                       ├─► T012 (Exercise) ─► T013 (AI Prompts)
T009 (diagram) ──┬─► T010 (B2 lesson) ┘
                 │
T011 (world)  ───┘
```

### Within Phase 4 (US2)
```
T014 (I1 lesson) ─────────────────────┐
                                       ├─► T019 (Exercise) ─► T020 (AI Prompts)
T015 (world) ───┬─► T016 (I2 lesson) ─┘
                │
T017 (launch) ──┤
                │
T018 (code)  ───┘
```

### Within Phase 5 (US3)
```
T021 (A1 lesson) ─────────────────────┐
                                       ├─► T025 (Exercise) ─► T026 (AI Prompts)
T023 (bridge) ──┬─► T022 (A2 lesson) ─┘
                │
T024 (monitor) ─┘
```

---

## Implementation Strategy

1. **MVP First**: Complete Phases 1-3 for immediate learner value
2. **Incremental Delivery**: Each phase produces a testable, usable chapter increment
3. **Independent Testing**: Each user story can be validated independently after completion
4. **Parallel Work**: Tasks marked [P] can run simultaneously when dependencies allow

## Validation Checklist

After each phase:
- [x] All code examples execute on Ubuntu 22.04 / ROS 2 Humble
- [x] Diagrams render correctly in markdown preview
- [x] AI prompts are RAG-compatible (queryable structure)
- [x] Lessons follow constitution principles (embodied learning, progressive mastery)

## Implementation Summary

**Completed**: 2025-12-25
**Files Created**: 16 content files
**Code Files**: 4 Python implementations (bridge_node.py, latency_monitor.py, sensor_streamer.py, joint_commander.py)
**World Files**: 2 (humanoid_lab.world, simple_lab.world)
**Exercises**: 3 hands-on exercises with completion criteria
