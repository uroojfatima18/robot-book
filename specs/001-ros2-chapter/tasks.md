# Tasks: Chapter 1 - The Robotic Nervous System (ROS 2)

**Input**: Design documents from `/specs/001-ros2-chapter/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Not explicitly requested in specification. Code examples will be validated manually per research.md.

**Organization**: Tasks are grouped by user story (P1-P4) to enable independent implementation and testing of each tier.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Beginner, US2=Intermediate, US3=Advanced URDF, US4=Advanced Actions)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md structure:
```
chapters/01-ros2-nervous-system/
├── README.md
├── glossary.md
├── introduction.md
├── beginner/
├── intermediate/
├── advanced/
├── code/
├── diagrams/
└── ai-prompts/
```

---

## Phase 1: Setup (Project Structure)

**Purpose**: Create chapter directory structure and foundational files

- [x] T001 Create chapter root directory at chapters/01-ros2-nervous-system/
- [x] T002 [P] Create tier subdirectories: beginner/, intermediate/, advanced/
- [x] T003 [P] Create code subdirectories: code/beginner/, code/intermediate/, code/advanced/
- [x] T004 [P] Create code/advanced/urdf/ and code/intermediate/launch/ directories
- [x] T005 [P] Create diagrams/ directory at chapters/01-ros2-nervous-system/diagrams/
- [x] T006 [P] Create ai-prompts/ directory at chapters/01-ros2-nervous-system/ai-prompts/
- [x] T007 [P] Create exercises directories: beginner/exercises/, intermediate/exercises/, advanced/exercises/

**Checkpoint**: Directory structure ready for content authoring

---

## Phase 2: Foundational (Chapter-Level Content)

**Purpose**: Create chapter landing page and introduction that ALL tiers depend on

**CRITICAL**: No tier content can begin until chapter introduction is complete

- [x] T008 Create chapter README.md at chapters/01-ros2-nervous-system/README.md with table of contents and learning path
- [x] T009 Create chapter introduction.md at chapters/01-ros2-nervous-system/introduction.md per lesson-interface contract
- [x] T009a Create glossary.md at chapters/01-ros2-nervous-system/glossary.md with ROS 2 terminology (nodes, topics, services, actions, URDF, etc.)
- [x] T010 [P] Create ROS 2 architecture diagram at chapters/01-ros2-nervous-system/diagrams/ros2-architecture.svg
- [x] T011 [P] Create node-topic-service diagram at chapters/01-ros2-nervous-system/diagrams/node-topic-service.svg

**Checkpoint**: Foundation ready - tier implementation can now begin

---

## Phase 3: User Story 1 - Beginner Tier (Priority: P1) MVP

**Goal**: Complete beginner learns ROS 2 fundamentals, installs ROS 2, runs demo nodes

**Independent Test**: Reader with no ROS experience completes tier and successfully runs `ros2 --version` and talker/listener demo

### Lesson B1: Introduction to ROS 2

- [x] T012 [US1] Create lesson file at chapters/01-ros2-nervous-system/beginner/01-intro-to-ros2.md with YAML frontmatter
- [x] T013 [US1] Write "What is ROS 2?" section explaining nervous system metaphor in 01-intro-to-ros2.md
- [x] T014 [US1] Write "ROS 2 vs ROS 1" comparison section in 01-intro-to-ros2.md
- [x] T015 [US1] Write "Core Concepts" section (nodes, topics, services, actions) in 01-intro-to-ros2.md
- [x] T016 [US1] Write "Installation on Ubuntu 22.04" section with step-by-step commands in 01-intro-to-ros2.md
- [x] T016a [US1] Write "Alternative Setup: WSL2 and Docker" section for Windows/macOS users in 01-intro-to-ros2.md
- [x] T017 [US1] Write "First Demo: Talker/Listener" walkthrough in 01-intro-to-ros2.md
- [x] T018 [P] [US1] Create demo_commands.sh script at chapters/01-ros2-nervous-system/code/beginner/demo_commands.sh
- [x] T019 [US1] Add Hardware Notes and Summary sections to 01-intro-to-ros2.md

### Lesson B2: Basic Sensors Overview

- [x] T020 [US1] Create lesson file at chapters/01-ros2-nervous-system/beginner/02-sensors-overview.md with YAML frontmatter
- [x] T021 [US1] Write "Sensor Types for Humanoid Robots" introduction in 02-sensors-overview.md
- [x] T022 [US1] Write IMU section (orientation, motion) in 02-sensors-overview.md
- [x] T023 [US1] Write LIDAR section (distance, mapping) in 02-sensors-overview.md
- [x] T024 [US1] Write Camera section (RGB, depth) in 02-sensors-overview.md
- [x] T025 [US1] Write Force Sensors section (touch, pressure) in 02-sensors-overview.md
- [x] T026 [US1] Write ROS 2 Sensor Message Types section with code examples in 02-sensors-overview.md
- [x] T027 [P] [US1] Create humanoid sensor placement diagram at chapters/01-ros2-nervous-system/diagrams/humanoid-sensor-placement.svg

### Beginner Tier Completion

- [x] T028 [US1] Create beginner exercises at chapters/01-ros2-nervous-system/beginner/exercises/beginner-exercises.md
- [x] T029 [US1] Create beginner AI prompts at chapters/01-ros2-nervous-system/ai-prompts/beginner-prompts.md
- [x] T030 [US1] Add navigation links between B1 and B2 lessons
- [x] T031 [US1] Validate all code examples in beginner tier execute correctly

**Checkpoint**: Beginner tier complete. Reader can install ROS 2 and understand fundamentals.

---

## Phase 4: User Story 2 - Intermediate Tier (Priority: P2)

**Goal**: Developer with beginner knowledge creates Python ROS 2 nodes, works with topics/services/actions

**Independent Test**: Reader creates minimal publisher/subscriber pair that successfully exchanges messages

### Lesson I1: Nodes, Topics, Services, and Actions

- [ ] T032 [US2] Create lesson file at chapters/01-ros2-nervous-system/intermediate/01-nodes-topics.md with YAML frontmatter
- [ ] T033 [US2] Write "Creating Python Nodes with rclpy" section in 01-nodes-topics.md
- [ ] T034 [US2] Write "Publishers and Subscribers" section with detailed explanation in 01-nodes-topics.md
- [ ] T035 [P] [US2] Create minimal_publisher.py at chapters/01-ros2-nervous-system/code/intermediate/minimal_publisher.py
- [ ] T036 [P] [US2] Create minimal_subscriber.py at chapters/01-ros2-nervous-system/code/intermediate/minimal_subscriber.py
- [ ] T037 [US2] Write "Topic QoS Basics" section in 01-nodes-topics.md
- [ ] T038 [US2] Write "Services: Request/Response Pattern" section in 01-nodes-topics.md
- [ ] T039 [P] [US2] Create simple_service.py at chapters/01-ros2-nervous-system/code/intermediate/simple_service.py
- [ ] T040 [US2] Write "Actions: Long-Running Tasks with Feedback" section in 01-nodes-topics.md
- [ ] T041 [P] [US2] Create pub-sub-flow diagram at chapters/01-ros2-nervous-system/diagrams/pub-sub-flow.svg
- [ ] T042 [P] [US2] Create service-pattern diagram at chapters/01-ros2-nervous-system/diagrams/service-pattern.svg
- [ ] T043 [P] [US2] Create action-pattern diagram at chapters/01-ros2-nervous-system/diagrams/action-pattern.svg

### Lesson I2: Python ROS Bridge (rclpy)

- [ ] T044 [US2] Create lesson file at chapters/01-ros2-nervous-system/intermediate/02-python-ros-bridge.md with YAML frontmatter
- [ ] T045 [US2] Write "rclpy Library Overview" section in 02-python-ros-bridge.md
- [ ] T046 [US2] Write "Node Lifecycle" section in 02-python-ros-bridge.md
- [ ] T047 [US2] Write "Parameters: Declaration and Usage" section with code examples in 02-python-ros-bridge.md
- [ ] T048 [US2] Write "Launch Files: Multi-Node Orchestration" section in 02-python-ros-bridge.md
- [ ] T049 [P] [US2] Create talker_listener_launch.py at chapters/01-ros2-nervous-system/code/intermediate/launch/talker_listener_launch.py
- [ ] T050 [US2] Write "Executors and Callback Groups" section in 02-python-ros-bridge.md
- [ ] T051 [P] [US2] Create rclpy-architecture diagram at chapters/01-ros2-nervous-system/diagrams/rclpy-architecture.svg

### Intermediate Tier Completion

- [ ] T052 [US2] Create intermediate exercises at chapters/01-ros2-nervous-system/intermediate/exercises/intermediate-exercises.md
- [ ] T053 [US2] Create intermediate AI prompts at chapters/01-ros2-nervous-system/ai-prompts/intermediate-prompts.md
- [ ] T054 [US2] Add navigation links between I1, I2, and connection to B2
- [ ] T055 [US2] Validate all Python code examples execute on ROS 2 Humble

**Checkpoint**: Intermediate tier complete. Reader can create functional ROS 2 Python nodes.

---

## Phase 5: User Story 3 - Advanced Tier Part 1: URDF (Priority: P3)

**Goal**: Engineer creates URDF models for humanoid robots and visualizes in RViz2

**Independent Test**: Reader creates basic humanoid URDF with torso and base link, loads successfully in RViz2

### Lesson A1: URDF & Humanoid Robot Description

- [ ] T056 [US3] Create lesson file at chapters/01-ros2-nervous-system/advanced/01-urdf-humanoid.md with YAML frontmatter
- [ ] T057 [US3] Write "URDF File Structure" section with XML explanation in 01-urdf-humanoid.md
- [ ] T058 [US3] Write "Links: Bodies and Geometry" section in 01-urdf-humanoid.md
- [ ] T059 [US3] Write "Joints: Connections and Constraints" section in 01-urdf-humanoid.md
- [ ] T060 [US3] Write "Visual and Collision Elements" section in 01-urdf-humanoid.md
- [ ] T061 [US3] Write "Building a Humanoid Skeleton" tutorial section in 01-urdf-humanoid.md
- [ ] T062 [P] [US3] Create humanoid_basic.urdf at chapters/01-ros2-nervous-system/code/advanced/urdf/humanoid_basic.urdf
- [ ] T063 [US3] Write "RViz2 Visualization" section with launch instructions in 01-urdf-humanoid.md
- [ ] T064 [P] [US3] Create urdf-structure diagram at chapters/01-ros2-nervous-system/diagrams/urdf-structure.svg
- [ ] T065 [P] [US3] Create joint-types diagram at chapters/01-ros2-nervous-system/diagrams/joint-types.svg
- [ ] T066 [US3] Add Hardware Notes section for real robot deployment considerations

**Checkpoint**: User Story 3 complete. Reader can create and visualize humanoid URDF models.

---

## Phase 6: User Story 4 - Advanced Tier Part 2: Actions & AI (Priority: P4)

**Goal**: Advanced developer implements action servers and understands AI agent integration

**Independent Test**: Reader implements Fibonacci action server that provides feedback and returns results

### Lesson A2: Advanced ROS 2 Patterns & AI Integration

- [ ] T067 [US4] Create lesson file at chapters/01-ros2-nervous-system/advanced/02-advanced-patterns.md with YAML frontmatter
- [ ] T068 [US4] Write "Action Server Implementation" section in 02-advanced-patterns.md
- [ ] T069 [P] [US4] Create fibonacci_action_server.py at chapters/01-ros2-nervous-system/code/advanced/fibonacci_action_server.py
- [ ] T070 [P] [US4] Create fibonacci_action_client.py at chapters/01-ros2-nervous-system/code/advanced/fibonacci_action_client.py
- [ ] T071 [US4] Write "Feedback Mechanisms" section in 02-advanced-patterns.md
- [ ] T072 [US4] Write "Goal Handling and Cancellation" section in 02-advanced-patterns.md
- [ ] T073 [US4] Write "AI Agent Integration Concepts" section (conceptual overview only, no working AI code) in 02-advanced-patterns.md
- [ ] T074 [US4] Write "Async Patterns in ROS 2" section in 02-advanced-patterns.md
- [ ] T075 [P] [US4] Create action-lifecycle diagram at chapters/01-ros2-nervous-system/diagrams/action-lifecycle.svg

### Advanced Tier Completion

- [ ] T076 [US4] Create advanced exercises at chapters/01-ros2-nervous-system/advanced/exercises/advanced-exercises.md
- [ ] T077 [US4] Create advanced AI prompts at chapters/01-ros2-nervous-system/ai-prompts/advanced-prompts.md
- [ ] T078 [US4] Add navigation links between A1, A2, and connection to I2
- [ ] T079 [US4] Validate action server/client examples execute correctly

**Checkpoint**: Advanced tier complete. Reader has mastered ROS 2 concepts from beginner to advanced.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final validation and refinements across all tiers

- [ ] T080 [P] Update chapter README.md with complete table of contents and verified links
- [ ] T081 [P] Verify all diagrams exist, are referenced correctly, and have descriptive alt-text
- [ ] T081a [P] Validate all diagrams use color-blind safe palettes (no red-green only distinctions)
- [ ] T081b [P] Verify content heading hierarchy supports screen reader navigation
- [ ] T082 [P] Validate all code examples follow code-standards.md conventions
- [ ] T083 Run full validation: all 6 lessons have YAML frontmatter per lesson-interface contract
- [ ] T083a Verify all lessons follow Theory -> Code Example -> Hands-on Exercise -> Summary structure
- [ ] T084 Verify each lesson has minimum 2 AI prompts as per data-model.md
- [ ] T085 Verify each tier has at least 1 exercise as per data-model.md
- [ ] T086 [P] Add troubleshooting section to beginner/01-intro-to-ros2.md for common installation issues
- [ ] T087 Cross-reference all Next Steps links work correctly between lessons
- [ ] T088 Final review: constitution compliance check (all 7 principles)

**Checkpoint**: Chapter 1 complete and ready for publication

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Phase 1 - BLOCKS all user stories
- **User Story 1/Beginner (Phase 3)**: Depends on Phase 2 completion
- **User Story 2/Intermediate (Phase 4)**: Depends on Phase 2 completion (can parallel with US1)
- **User Story 3/URDF (Phase 5)**: Depends on Phase 2 completion (can parallel with US1/US2)
- **User Story 4/Actions (Phase 6)**: Depends on Phase 2 completion (can parallel with US1/US2/US3)
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1 Beginner)**: Foundation only - entry point for all readers
- **User Story 2 (P2 Intermediate)**: Conceptually depends on US1 content, but can be authored in parallel
- **User Story 3 (P3 URDF)**: Conceptually depends on US2 content, but can be authored in parallel
- **User Story 4 (P4 Actions)**: Conceptually depends on US2/US3 content, but can be authored in parallel

### Within Each User Story

- Lesson files before code examples
- Code examples before diagrams (for reference accuracy)
- Content sections before exercises
- All tier content before AI prompts

### Parallel Opportunities

- T002-T007 (directory creation) can all run in parallel
- T010-T011 (foundational diagrams) can run in parallel
- Within each tier: diagram tasks marked [P] can run in parallel
- Within each tier: code file tasks marked [P] can run in parallel
- Different user stories can be worked on in parallel by different authors

---

## Parallel Example: User Story 1 (Beginner Tier)

```bash
# After T012 (lesson file created), launch code and diagram tasks together:
Task: "T018 [P] [US1] Create demo_commands.sh"
Task: "T027 [P] [US1] Create humanoid sensor placement diagram"

# These have no dependencies on each other
```

## Parallel Example: User Story 2 (Intermediate Tier)

```bash
# After T032-T034 (lesson structure), launch code files together:
Task: "T035 [P] [US2] Create minimal_publisher.py"
Task: "T036 [P] [US2] Create minimal_subscriber.py"
Task: "T039 [P] [US2] Create simple_service.py"

# Launch diagrams together:
Task: "T041 [P] [US2] Create pub-sub-flow diagram"
Task: "T042 [P] [US2] Create service-pattern diagram"
Task: "T043 [P] [US2] Create action-pattern diagram"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T007)
2. Complete Phase 2: Foundational (T008-T011)
3. Complete Phase 3: User Story 1 - Beginner Tier (T012-T031)
4. **STOP and VALIDATE**: Test with novice reader
5. Deploy/demo MVP chapter with Beginner content

### Incremental Delivery

1. Setup + Foundational → Structure ready
2. Add Beginner Tier (US1) → Test independently → Publish beginner content
3. Add Intermediate Tier (US2) → Test independently → Publish intermediate content
4. Add Advanced URDF (US3) → Test independently → Publish advanced part 1
5. Add Advanced Actions (US4) → Test independently → Publish complete chapter
6. Each tier adds value without breaking previous tiers

### Parallel Team Strategy

With multiple authors:
1. All authors complete Setup + Foundational together
2. Once Foundational is done:
   - Author A: User Story 1 (Beginner)
   - Author B: User Story 2 (Intermediate)
   - Author C: User Story 3 + 4 (Advanced)
3. Stories complete and integrate independently

---

## Summary

| Phase | User Story | Tasks | Parallel Opportunities |
|-------|------------|-------|------------------------|
| Phase 1 | Setup | T001-T007 (7) | 6 parallel directory tasks |
| Phase 2 | Foundational | T008-T011 + T009a (5) | 2 parallel diagram tasks |
| Phase 3 | US1 Beginner | T012-T031 + T016a (21) | 2 parallel code/diagram tasks |
| Phase 4 | US2 Intermediate | T032-T055 (24) | 8 parallel code/diagram tasks |
| Phase 5 | US3 URDF | T056-T066 (11) | 3 parallel diagram tasks |
| Phase 6 | US4 Actions | T067-T079 (13) | 3 parallel code/diagram tasks |
| Phase 7 | Polish | T080-T088 + accessibility (13) | 6 parallel validation tasks |

**Total Tasks**: 94
**MVP Scope**: Phases 1-3 (33 tasks for complete Beginner tier including glossary and WSL2/Docker)
**Per-Story Testability**: Each tier independently validates against spec success criteria

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story (US1-US4)
- Each tier is independently completable and testable per spec.md acceptance criteria
- Commit after each task or logical group
- Stop at any checkpoint to validate tier independently
- All code must follow contracts/code-standards.md
- All lessons must follow contracts/lesson-interface.md
- All lessons must follow Content Standards (Theory -> Example -> Exercise -> Summary)
- All diagrams must include alt-text and use color-blind safe palettes (NFR-001 to NFR-004)
- Glossary must be created before tier content begins
