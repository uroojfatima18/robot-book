# Tasks: Chapter 3 - AI-Robot Brain (NVIDIA Isaac)

**Input**: Design documents from `/specs/003-isaac-ai-brain/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/

**Tests**: No automated tests specified. Validation is manual (reader comprehension, code execution).

**Organization**: Tasks are grouped by user story (US1-US4) to enable independent implementation and testing of each tier.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Beginner, US2=Intermediate, US3=Advanced Navigation, US4=Advanced RL)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md structure:
- **Content**: `chapters/003-ai-robot-brain/`
- **Beginner tier**: `chapters/003-ai-robot-brain/beginner/`
- **Intermediate tier**: `chapters/003-ai-robot-brain/intermediate/`
- **Advanced tier**: `chapters/003-ai-robot-brain/advanced/`
- **Exercises**: `chapters/003-ai-robot-brain/exercises/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create chapter directory structure and README

- [x] T001 Create chapter directory structure: `chapters/003-ai-robot-brain/{beginner,intermediate,advanced}/{diagrams,code}` and `exercises/`
- [x] T002 Create chapter README with overview and navigation in `chapters/003-ai-robot-brain/README.md`
- [x] T003 [P] Create beginner tier diagrams folder at `chapters/003-ai-robot-brain/beginner/diagrams/`
- [x] T004 [P] Create intermediate tier code and launch folders at `chapters/003-ai-robot-brain/intermediate/code/` and `launch/`
- [x] T005 [P] Create advanced tier code and pretrained folders at `chapters/003-ai-robot-brain/advanced/code/` and `pretrained/`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Establish chapter-level learning objectives and cross-cutting content

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T006 Define chapter learning objectives per FR-001 in `chapters/003-ai-robot-brain/README.md`
- [x] T007 Create color-blind safe diagram palette reference per NFR-002 in `chapters/003-ai-robot-brain/beginner/diagrams/palette.md`
- [x] T008 [P] Create Chapter 1 (ROS 2) inline refresher template per NFR-006
- [x] T009 [P] Create Chapter 2 (Digital Twin) inline refresher template per NFR-006

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Beginner Understanding AI Perception (Priority: P1) MVP

**Goal**: Complete Beginner tier - readers understand perception, sensors, SLAM, and navigation concepts

**Independent Test**: Reader can explain perception pipeline, identify sensors, and describe SLAM/navigation

### Lesson B1: Introduction to Robotic Perception

- [x] T010 [US1] Create lesson B1 structure following NFR-005 in `chapters/003-ai-robot-brain/beginner/B1-introduction-perception.md`
- [x] T011 [P] [US1] Write theory section: how sensors capture data and create understanding per FR-002
- [x] T012 [P] [US1] Create perception pipeline diagram D1 (SVG) with alt-text per NFR-001 in `chapters/003-ai-robot-brain/beginner/diagrams/perception-pipeline.svg`
- [x] T013 [US1] Add demonstration commands for RViz2 visualization per FR-007 in B1
- [x] T014 [US1] Add AI Agent Assisted Prompts (2-3) for RAG per Constitution III in B1
- [x] T015 [US1] Write hands-on exercise and summary sections in B1

### Lesson B2: Understanding Sensor Types

- [x] T016 [US1] Create lesson B2 structure in `chapters/003-ai-robot-brain/beginner/B2-sensor-types.md`
- [x] T017 [P] [US1] Write theory section: RGB cameras, depth cameras (stereo, ToF, structured light), LIDAR per FR-003
- [x] T018 [P] [US1] Create sensor comparison diagram D2 (SVG) with alt-text in `chapters/003-ai-robot-brain/beginner/diagrams/sensor-comparison.svg`
- [x] T019 [US1] Add inline Chapter 1/2 refresher per NFR-006 in B2
- [x] T020 [US1] Add AI Agent Assisted Prompts (2-3) for RAG in B2
- [x] T021 [US1] Write hands-on exercise and summary sections in B2

### Lesson B3: SLAM and Navigation Concepts

- [x] T022 [US1] Create lesson B3 structure in `chapters/003-ai-robot-brain/beginner/B3-slam-navigation-intro.md`
- [x] T023 [P] [US1] Write theory section: SLAM concepts per FR-004
- [x] T024 [P] [US1] Write theory section: navigation components (global, local, execution) per FR-005
- [x] T025 [P] [US1] Create navigation architecture diagram D3 (SVG) with alt-text per FR-006 in `chapters/003-ai-robot-brain/beginner/diagrams/navigation-architecture.svg`
- [x] T026 [US1] Add demonstration commands for map visualization per FR-007 in B3
- [x] T027 [US1] Add AI Agent Assisted Prompts (2-3) for RAG in B3
- [x] T028 [US1] Write hands-on exercise and summary sections in B3

### Beginner Exercises

- [x] T029 [US1] Create beginner exercises file in `chapters/003-ai-robot-brain/exercises/beginner-exercises.md`
- [x] T030 [US1] Add 3-5 concept check exercises covering perception, sensors, SLAM, navigation

**MVP Checkpoint**: Beginner tier complete. Test: Reader can explain perception pipeline, identify sensors, describe SLAM/navigation. Validates SC-001, SC-002.

---

## Phase 4: User Story 2 - Developer Implementing Perception Pipelines (Priority: P2)

**Goal**: Complete Intermediate tier - readers implement perception nodes, TF2, SLAM, and basic Nav2

**Independent Test**: Reader can create sensor processing node, run SLAM, send navigation goals

### Lesson I1: Camera and Depth Data Processing

- [x] T031 [US2] Create lesson I1 structure in `chapters/003-ai-robot-brain/intermediate/I1-camera-depth-processing.md`
- [x] T032 [P] [US2] Write theory section: cv_bridge and ROS 2 image processing per FR-008, FR-009
- [x] T033 [P] [US2] Create camera_subscriber.py code example per FR-013 in `chapters/003-ai-robot-brain/intermediate/code/camera_subscriber.py`
- [x] T034 [P] [US2] Create depth_processor.py code example in `chapters/003-ai-robot-brain/intermediate/code/depth_processor.py`
- [x] T035 [US2] Add expected outputs and ROS 2 message annotations per code-example-standards.md
- [x] T036 [US2] Add inline Chapter 1/2 refresher (ROS 2 nodes, topics) per NFR-006
- [x] T037 [US2] Add AI Agent Assisted Prompts (2-3) for RAG in I1
- [x] T038 [US2] Write hands-on exercise and summary sections in I1

### Lesson I2: TF2 Coordinate Frames

- [x] T039 [US2] Create lesson I2 structure in `chapters/003-ai-robot-brain/intermediate/I2-tf2-coordinate-frames.md`
- [x] T040 [P] [US2] Write theory section: TF2 coordinate frame management per FR-010
- [x] T041 [P] [US2] Create TF tree example diagram D4 (SVG) with alt-text in `chapters/003-ai-robot-brain/intermediate/diagrams/tf-tree-example.svg`
- [x] T042 [P] [US2] Create tf2_broadcaster.py code example in `chapters/003-ai-robot-brain/intermediate/code/tf2_broadcaster.py`
- [x] T043 [US2] Add inline Chapter 1 refresher (URDF) per NFR-006
- [x] T044 [US2] Add AI Agent Assisted Prompts (2-3) for RAG in I2
- [x] T045 [US2] Write hands-on exercise and summary sections in I2

### Lesson I3: SLAM Toolbox Configuration

- [x] T046 [US2] Create lesson I3 structure in `chapters/003-ai-robot-brain/intermediate/I3-slam-toolbox.md`
- [x] T047 [P] [US2] Write theory section: SLAM Toolbox configuration and map generation per FR-011
- [x] T048 [P] [US2] Create SLAM process flow diagram D5 (SVG) with alt-text in `chapters/003-ai-robot-brain/intermediate/diagrams/slam-process.svg`
- [x] T049 [P] [US2] Create slam_launch.py launch file per FR-014 in `chapters/003-ai-robot-brain/intermediate/launch/slam_launch.py`
- [x] T050 [US2] Add inline Chapter 2 refresher (Gazebo) per NFR-006
- [x] T051 [US2] Add AI Agent Assisted Prompts (2-3) for RAG in I3
- [x] T052 [US2] Write hands-on exercise and summary sections in I3

### Lesson I4: Nav2 Basics

- [x] T053 [US2] Create lesson I4 structure in `chapters/003-ai-robot-brain/intermediate/I4-nav2-basics.md`
- [x] T054 [P] [US2] Write theory section: Nav2 lifecycle nodes, costmaps, navigation per FR-012
- [x] T055 [P] [US2] Create nav2_goal_sender.py code example per FR-015 in `chapters/003-ai-robot-brain/intermediate/code/nav2_goal_sender.py`
- [x] T056 [P] [US2] Create navigation_launch.py launch file per FR-014 in `chapters/003-ai-robot-brain/intermediate/launch/navigation_launch.py`
- [x] T057 [US2] Add AI Agent Assisted Prompts (2-3) for RAG in I4
- [x] T058 [US2] Write hands-on exercise and summary sections in I4

### Intermediate Exercises

- [x] T059 [US2] Create intermediate exercises file in `chapters/003-ai-robot-brain/exercises/intermediate-exercises.md`
- [x] T060 [US2] Add 3-5 coding exercises: sensor node, TF2, SLAM, Nav2

**Tier Checkpoint**: Intermediate tier complete. Test: Reader can create sensor node (SC-004), generate map (SC-005), launch Nav2 (SC-006).

---

## Phase 5: User Story 3 - Engineer Creating Autonomous Navigation (Priority: P3)

**Goal**: Complete Advanced Navigation - readers configure costmaps, planners, behavior trees

**Independent Test**: Reader can configure Nav2 costmaps/planners and create custom behavior trees

### Lesson A1: Costmap Configuration

- [x] T061 [US3] Create lesson A1 structure in `chapters/003-ai-robot-brain/advanced/A1-costmap-configuration.md`
- [x] T062 [P] [US3] Write theory section: costmap layers, inflation, update frequencies per FR-016
- [x] T063 [P] [US3] Create costmap layers diagram D6 (SVG) with alt-text in `chapters/003-ai-robot-brain/advanced/diagrams/costmap-layers.svg`
- [x] T064 [P] [US3] Create costmap_config.yaml example in `chapters/003-ai-robot-brain/advanced/code/costmap_config.yaml`
- [x] T065 [US3] Add AI Agent Assisted Prompts (2-3) for RAG in A1
- [x] T066 [US3] Write hands-on exercise and summary sections in A1

### Lesson A2: Planners and Behavior Trees

- [x] T067 [US3] Create lesson A2 structure in `chapters/003-ai-robot-brain/advanced/A2-planners-behavior-trees.md`
- [x] T068 [P] [US3] Write theory section: global planners (NavFn, Smac), local planners (DWB, MPPI) per FR-017
- [x] T069 [P] [US3] Write theory section: behavior trees structure, nodes, custom behaviors per FR-018
- [x] T070 [P] [US3] Create behavior_tree_example.xml code example in `chapters/003-ai-robot-brain/advanced/code/behavior_tree_example.xml`
- [x] T071 [US3] Add AI Agent Assisted Prompts (2-3) for RAG in A2
- [x] T072 [US3] Write hands-on exercise and summary sections in A2

**Tier Checkpoint (Navigation)**: Advanced navigation complete. Test: Reader can configure costmaps (SC-007).

---

## Phase 6: User Story 4 - Advanced Developer Implementing Learning Systems (Priority: P4)

**Goal**: Complete Advanced RL/Sim-to-Real - readers understand RL fundamentals and sim-to-real transfer

**Independent Test**: Reader can load pre-trained policy, explain RL loop, describe sim-to-real gap

### Lesson A3: Reinforcement Learning Fundamentals

- [x] T073 [US4] Create lesson A3 structure in `chapters/003-ai-robot-brain/advanced/A3-reinforcement-learning.md`
- [x] T074 [P] [US4] Write theory section: MDP, policy, value functions, PPO, SAC per FR-019
- [x] T075 [P] [US4] Write theory section: NVIDIA Isaac Gym overview per FR-020
- [x] T076 [P] [US4] Create RL training loop diagram D7 (SVG) with alt-text in `chapters/003-ai-robot-brain/advanced/diagrams/rl-loop.svg`
- [x] T077 [US4] Add AI Agent Assisted Prompts (2-3) for RAG in A3
- [x] T078 [US4] Write hands-on exercise and summary sections in A3

### Lesson A4: Sim-to-Real Transfer

- [x] T079 [US4] Create lesson A4 structure in `chapters/003-ai-robot-brain/advanced/A4-sim-to-real.md`
- [x] T080 [P] [US4] Write theory section: domain gap, domain randomization, system ID per FR-021
- [x] T081 [P] [US4] Write theory section: safety considerations per FR-023 with Constitution VII warnings
- [x] T082 [P] [US4] Create sim-to-real gap diagram D8 (SVG) with alt-text in `chapters/003-ai-robot-brain/advanced/diagrams/sim-to-real-gap.svg`
- [x] T083 [P] [US4] Create policy_loader.py code example per FR-022 in `chapters/003-ai-robot-brain/advanced/code/policy_loader.py`
- [x] T084 [US4] Add pre-trained locomotion policy placeholder in `chapters/003-ai-robot-brain/advanced/pretrained/locomotion_policy.onnx`
- [x] T085 [US4] Add AI Agent Assisted Prompts (2-3) for RAG in A4
- [x] T086 [US4] Write hands-on exercise and summary sections in A4

### Advanced Exercises

- [x] T087 [US4] Create advanced exercises file in `chapters/003-ai-robot-brain/exercises/advanced-exercises.md`
- [x] T088 [US4] Add 3-5 advanced exercises: costmaps, behavior trees, RL concepts, sim-to-real

**Chapter Checkpoint**: Advanced tier complete. Test: Reader can explain RL (SC-008), sim-to-real (SC-009).

---

## Phase 7: Polish and Cross-Cutting Concerns

**Purpose**: Validation and final improvements

- [x] T089 [P] Validate all Python code syntax: `python3 -m py_compile chapters/003-ai-robot-brain/*/code/*.py`
- [x] T090 [P] Validate YAML syntax for costmap_config.yaml
- [x] T091 [P] Verify all diagrams have alt-text per NFR-001
- [x] T092 [P] Verify color-blind safe palette usage per NFR-002
- [x] T093 Check heading hierarchy for screen readers per NFR-003
- [x] T094 Verify all lessons follow Theory then Code then Exercise then Summary per NFR-005
- [x] T095 Run quickstart.md validation commands
- [x] T096 Final review against constitution principles I-VII

---

## Dependencies and Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **US1 Beginner (Phase 3)**: Depends on Foundational - Can run independently
- **US2 Intermediate (Phase 4)**: Depends on Foundational - Can run parallel to US1
- **US3 Adv Navigation (Phase 5)**: Depends on Foundational - Can run parallel to US1/US2
- **US4 Adv RL (Phase 6)**: Depends on Foundational - Can run parallel to US1/US2/US3
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **US1 (P1 - Beginner)**: No dependencies on other stories - MVP deliverable
- **US2 (P2 - Intermediate)**: Can reference US1 concepts but testable independently
- **US3 (P3 - Adv Nav)**: Can reference US1/US2 concepts but testable independently
- **US4 (P4 - Adv RL)**: Can reference US1/US2/US3 concepts but testable independently

### Within Each User Story

- Lesson structure first
- Theory sections and diagrams in parallel
- Code examples in parallel
- AI prompts and exercises after theory
- Summary last

### Parallel Opportunities

**Phase 3 (US1 - Beginner)**: T011, T012 can run parallel; T017, T018 can run parallel; T023, T024, T025 can run parallel

**Phase 4 (US2 - Intermediate)**: T032, T033, T034 can run parallel; T040, T041, T042 can run parallel; T047, T048, T049 can run parallel; T054, T055, T056 can run parallel

**Phase 5 (US3 - Adv Nav)**: T062, T063, T064 can run parallel; T068, T069, T070 can run parallel

**Phase 6 (US4 - Adv RL)**: T074, T075, T076 can run parallel; T080, T081, T082, T083 can run parallel

---

## Implementation Strategy

### MVP First (User Story 1 = Beginner Tier)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T009)
3. Complete Phase 3: US1 Beginner (T010-T030)
4. **STOP and VALIDATE**: Reader can explain perception, sensors, SLAM
5. Deploy/demo Beginner tier if ready

### Incremental Delivery

1. Setup + Foundational then Foundation ready
2. Add Beginner tier (US1) then Test then Deploy (MVP!)
3. Add Intermediate tier (US2) then Test then Deploy
4. Add Advanced Navigation (US3) then Test then Deploy
5. Add Advanced RL (US4) then Test then Deploy
6. Each tier adds value without breaking previous tiers

### Parallel Team Strategy

With multiple content authors:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Author A: Beginner tier (US1)
   - Author B: Intermediate tier (US2)
   - Author C: Advanced Navigation (US3)
   - Author D: Advanced RL (US4)
3. Tiers complete and validate independently

---

## Summary

| Metric | Count |
|--------|-------|
| **Total Tasks** | 96 |
| **Phase 1 Setup** | 5 |
| **Phase 2 Foundational** | 4 |
| **Phase 3 US1 Beginner** | 21 |
| **Phase 4 US2 Intermediate** | 30 |
| **Phase 5 US3 Adv Nav** | 12 |
| **Phase 6 US4 Adv RL** | 16 |
| **Phase 7 Polish** | 8 |
| **Parallel Opportunities** | 40+ tasks marked [P] |
| **MVP Scope** | Phase 1-3 (30 tasks) |

---

## Notes

- [P] tasks = different files, no dependencies
- [US1-US4] labels map tasks to specific user stories
- Each tier is independently completable and testable
- Constitution principles embedded: AI prompts (III), safety warnings (VII), simulation-first (II)
- All diagrams require alt-text (NFR-001) and color-blind safe palette (NFR-002)
- All lessons follow Theory then Code then Exercise then Summary (NFR-005)
- Commit after each task or logical group
- Stop at any checkpoint to validate tier independently
