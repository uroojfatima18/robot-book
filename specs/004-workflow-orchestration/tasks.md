# Tasks: Chapter 4 - Workflow Orchestration

**Input**: Design documents from `/specs/004-workflow-orchestration/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Included - FR-X04 requires automated assessment tests per tier

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md project structure:
- **Content**: `chapters/04-workflow-orchestration/content/{tier}/`
- **Code**: `chapters/04-workflow-orchestration/code/ros2_ws/src/`
- **Exercises**: `chapters/04-workflow-orchestration/exercises/{tier}/`
- **Diagrams**: `chapters/04-workflow-orchestration/diagrams/`
- **Assessments**: `chapters/04-workflow-orchestration/assessments/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, directory structure, and ROS 2 package scaffolding

- [ ] T001 Create chapter directory structure per plan.md in chapters/04-workflow-orchestration/
- [ ] T002 [P] Create README.md chapter overview in chapters/04-workflow-orchestration/README.md
- [ ] T003 [P] Initialize workflow_examples ROS 2 package with package.xml and setup.py in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_examples/
- [ ] T004 [P] Initialize workflow_mocks ROS 2 package with package.xml and setup.py in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_mocks/
- [ ] T005 [P] Initialize workflow_tests ROS 2 package with package.xml and setup.py in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_tests/
- [ ] T006 [P] Create YAML pipeline configuration template in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_examples/config/pipeline_params.yaml
- [ ] T007 Verify workspace builds with colcon build --symlink-install

**Checkpoint**: ROS 2 workspace structure ready, all packages build successfully

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core code components that ALL user stories depend on

**CRITICAL**: No user story work can begin until this phase is complete

- [ ] T008 Create RobotState enum and SimpleFSM class in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_examples/workflow_examples/state_machine.py
- [ ] T009 [P] Create mock_lidar node in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_mocks/workflow_mocks/mock_lidar.py
- [ ] T010 [P] Create mock_path_planner node in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_mocks/workflow_mocks/mock_path_planner.py
- [ ] T011 [P] Create mock_motor_controller node in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_mocks/workflow_mocks/mock_motor_controller.py
- [ ] T012 Create simple_pipeline base node in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_examples/workflow_examples/simple_pipeline.py
- [ ] T013 [P] Create pipeline_demo.launch.py in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_examples/launch/pipeline_demo.launch.py
- [ ] T014 Verify mock nodes launch and communicate via ros2 topic list and ros2 topic echo

**Checkpoint**: Foundation ready - mock pipeline runs, FSM implemented, user story implementation can now begin

---

## Phase 3: User Story 1 - Understanding Workflow Concepts (Priority: P1) MVP

**Goal**: Beginner students can visualize and conceptualize robotic workflows with diagrams and simple code examples

**Independent Test**: Student can draw and explain a pipeline diagram, predict output of a 4-stage pipeline

### Tests for User Story 1

- [ ] T015 [P] [US1] Create beginner tier assessment test in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_tests/test/test_beginner_tier.py
- [ ] T016 [P] [US1] Create assessment criteria document in chapters/04-workflow-orchestration/assessments/tier-beginner.md

### Diagrams for User Story 1

- [ ] T017 [P] [US1] Create pipeline-flow.mmd diagram in chapters/04-workflow-orchestration/diagrams/pipeline-flow.mmd
- [ ] T018 [P] [US1] Create state-machine.mmd diagram in chapters/04-workflow-orchestration/diagrams/state-machine.mmd

### Content for User Story 1

- [ ] T019 [US1] Write B1-pipelines-flows-triggers.md lesson in chapters/04-workflow-orchestration/content/beginner/B1-pipelines-flows-triggers.md
- [ ] T020 [US1] Write B2-state-machines-concepts.md lesson in chapters/04-workflow-orchestration/content/beginner/B2-state-machines-concepts.md
- [ ] T021 [US1] Write B3-data-handoff.md lesson in chapters/04-workflow-orchestration/content/beginner/B3-data-handoff.md

### Exercises for User Story 1

- [ ] T022 [P] [US1] Create beginner exercise 1 (diagram labeling) in chapters/04-workflow-orchestration/exercises/beginner/exercise-01-diagram-labeling.md
- [ ] T023 [P] [US1] Create beginner exercise 2 (pipeline decomposition) in chapters/04-workflow-orchestration/exercises/beginner/exercise-02-pipeline-decomposition.md
- [ ] T024 [P] [US1] Create beginner exercise 3 (data flow tracing) in chapters/04-workflow-orchestration/exercises/beginner/exercise-03-data-flow-tracing.md

### RAG Prompts for User Story 1

- [ ] T025 [US1] Add RAG prompts section to each beginner lesson (debugging, explanation, generation)

**Checkpoint**: Beginner tier complete - student can conceptualize workflows, draw diagrams, and predict pipeline behavior

---

## Phase 4: User Story 2 - Building Sequential Task Pipelines (Priority: P1)

**Goal**: Intermediate students can implement and launch ROS 2 node pipelines with correct execution order

**Independent Test**: Launch file starts nodes in order, logs show correct timestamps, service calls trigger sequences

### Tests for User Story 2

- [ ] T026 [P] [US2] Create intermediate tier assessment test in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_tests/test/test_intermediate_tier.py
- [ ] T027 [P] [US2] Create assessment criteria document in chapters/04-workflow-orchestration/assessments/tier-intermediate.md

### Code for User Story 2

- [ ] T028 [P] [US2] Implement service-based pipeline triggering in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_examples/workflow_examples/pipeline_controller.py
- [ ] T029 [P] [US2] Implement fallback path handler in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_examples/workflow_examples/fallback_handler.py
- [ ] T030 [US2] Create conditional_pipeline.launch.py with service branching in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_examples/launch/conditional_pipeline.launch.py

### Content for User Story 2

- [ ] T031 [US2] Write I1-launch-files.md lesson in chapters/04-workflow-orchestration/content/intermediate/I1-launch-files.md
- [ ] T032 [US2] Write I2-inter-node-data-passing.md lesson in chapters/04-workflow-orchestration/content/intermediate/I2-inter-node-data-passing.md
- [ ] T033 [US2] Write I3-fallback-paths.md lesson in chapters/04-workflow-orchestration/content/intermediate/I3-fallback-paths.md

### Exercises for User Story 2

- [ ] T034 [P] [US2] Create intermediate exercise 1 (launch file creation) in chapters/04-workflow-orchestration/exercises/intermediate/exercise-01-launch-files.md
- [ ] T035 [P] [US2] Create intermediate exercise 2 (topic remapping) in chapters/04-workflow-orchestration/exercises/intermediate/exercise-02-topic-remapping.md
- [ ] T036 [P] [US2] Create intermediate exercise 3 (fallback implementation) in chapters/04-workflow-orchestration/exercises/intermediate/exercise-03-fallback-implementation.md

### RAG Prompts for User Story 2

- [ ] T037 [US2] Add RAG prompts section to each intermediate lesson (debugging, explanation, generation)

**Checkpoint**: Intermediate tier complete - student can launch pipelines, trace data flow, implement fallback paths

---

## Phase 5: User Story 3 - Implementing Failure Recovery (Priority: P2)

**Goal**: Advanced students can implement watchdog supervisors and recovery mechanisms for resilient robot operation

**Independent Test**: Simulated node crash triggers recovery within 5 seconds, system resumes without full restart

### Tests for User Story 3

- [ ] T038 [P] [US3] Create advanced tier assessment test in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_tests/test/test_advanced_tier.py
- [ ] T039 [P] [US3] Create assessment criteria document in chapters/04-workflow-orchestration/assessments/tier-advanced.md

### Diagrams for User Story 3

- [ ] T040 [P] [US3] Create watchdog-architecture.mmd diagram in chapters/04-workflow-orchestration/diagrams/watchdog-architecture.mmd

### Code for User Story 3

- [ ] T041 [US3] Implement watchdog_supervisor node in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_examples/workflow_examples/watchdog_supervisor.py
- [ ] T042 [US3] Implement recovery_handler node in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_examples/workflow_examples/recovery_handler.py
- [ ] T043 [P] [US3] Implement sensor_dropout_detector in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_examples/workflow_examples/sensor_dropout_detector.py
- [ ] T044 [US3] Create watchdog_demo.launch.py in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_examples/launch/watchdog_demo.launch.py

### Content for User Story 3

- [ ] T045 [US3] Write A1-watchdogs-supervisors.md lesson in chapters/04-workflow-orchestration/content/advanced/A1-watchdogs-supervisors.md
- [ ] T046 [US3] Write A2-sensor-dropout-handling.md lesson in chapters/04-workflow-orchestration/content/advanced/A2-sensor-dropout-handling.md
- [ ] T047 [US3] Write A3-value-based-routing.md lesson in chapters/04-workflow-orchestration/content/advanced/A3-value-based-routing.md

### Exercises for User Story 3

- [ ] T048 [P] [US3] Create advanced exercise 1 (watchdog implementation) in chapters/04-workflow-orchestration/exercises/advanced/exercise-01-watchdog-implementation.md
- [ ] T049 [P] [US3] Create advanced exercise 2 (recovery logic) in chapters/04-workflow-orchestration/exercises/advanced/exercise-02-recovery-logic.md
- [ ] T050 [P] [US3] Create advanced exercise 3 (decision routing) in chapters/04-workflow-orchestration/exercises/advanced/exercise-03-decision-routing.md

### RAG Prompts for User Story 3

- [ ] T051 [US3] Add RAG prompts section to each advanced lesson (debugging, explanation, generation)

### Safety Warnings for User Story 3

- [ ] T052 [US3] Add safety callouts to motor control and recovery code sections

**Checkpoint**: Advanced tier complete - student can implement watchdogs, handle sensor dropouts, build resilient systems

---

## Phase 6: User Story 4 - Performance Monitoring and Optimization (Priority: P3)

**Goal**: Advanced practitioners can instrument, measure, and optimize workflow performance

**Independent Test**: Profiling session produces latency metrics, student identifies bottleneck and proposes optimization

### Code for User Story 4

- [ ] T053 [P] [US4] Implement latency_monitor node in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_examples/workflow_examples/latency_monitor.py
- [ ] T054 [P] [US4] Implement pipeline_profiler utility in chapters/04-workflow-orchestration/code/ros2_ws/src/workflow_examples/workflow_examples/pipeline_profiler.py

### Content for User Story 4

- [ ] T055 [US4] Add performance monitoring section to A3-value-based-routing.md in chapters/04-workflow-orchestration/content/advanced/A3-value-based-routing.md
- [ ] T056 [P] [US4] Create performance optimization guide in chapters/04-workflow-orchestration/content/advanced/performance-optimization-guide.md

### Exercises for User Story 4

- [ ] T057 [P] [US4] Create profiling exercise (latency measurement) in chapters/04-workflow-orchestration/exercises/advanced/exercise-04-profiling.md
- [ ] T058 [P] [US4] Create optimization exercise (bottleneck identification) in chapters/04-workflow-orchestration/exercises/advanced/exercise-05-optimization.md

**Checkpoint**: Performance tier complete - student can measure, analyze, and optimize workflow performance

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Quality improvements that affect multiple user stories

- [ ] T059 [P] Ensure all code examples follow ROS 2 Python conventions (PEP 8, rclpy patterns)
- [ ] T060 [P] Add inline comments to all code examples explaining key concepts
- [ ] T061 [P] Verify all diagrams render correctly in Mermaid
- [ ] T062 Run colcon test to verify all assessment tests pass
- [ ] T063 [P] Update quickstart.md with final commands and examples in specs/004-workflow-orchestration/quickstart.md
- [ ] T064 [P] Add table of contents and navigation links to chapter README.md
- [ ] T065 Validate constitution compliance (RAG prompts, safety warnings, code blocks per lesson)
- [ ] T066 Final content review for beginner → intermediate → advanced progression

---

## Summary

| Phase | Tasks | Parallel Tasks | User Story |
|-------|-------|----------------|------------|
| Phase 1: Setup | 7 | 5 | N/A |
| Phase 2: Foundational | 7 | 4 | N/A |
| Phase 3: US1 Concepts | 11 | 8 | Understanding Workflow Concepts (P1) |
| Phase 4: US2 Pipelines | 12 | 6 | Building Sequential Pipelines (P1) |
| Phase 5: US3 Recovery | 15 | 7 | Implementing Failure Recovery (P2) |
| Phase 6: US4 Monitoring | 6 | 5 | Performance Monitoring (P3) |
| Phase 7: Polish | 8 | 5 | N/A |
| **Total** | **66** | **40** | **4 User Stories** |

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - US1 and US2 are both P1 priority - can proceed in parallel if staffed
  - US3 (P2) can start after Foundational, but logically builds on US2 content
  - US4 (P3) can start after Foundational, but logically extends US3 content
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational - No dependencies on other stories (conceptual content)
- **User Story 2 (P1)**: Can start after Foundational - No dependencies (practical implementation)
- **User Story 3 (P2)**: Can start after Foundational - Uses code from US2 but content is independent
- **User Story 4 (P3)**: Can start after Foundational - Extends US3 but content is independent

### Within Each User Story

- Tests MUST be written and verified FIRST
- Diagrams can be created in parallel with tests
- Code implementation before content (content references code)
- Content before exercises (exercises reference content)
- RAG prompts added after content is finalized
- Safety warnings added to relevant code sections

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T002-T006)
- All mock nodes marked [P] can run in parallel (T009-T011)
- All test tasks marked [P] can run in parallel within each story
- All diagram tasks marked [P] can run in parallel
- All exercise tasks marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all diagrams and tests for User Story 1 together:
Task: "Create pipeline-flow.mmd diagram in diagrams/pipeline-flow.mmd"
Task: "Create state-machine.mmd diagram in diagrams/state-machine.mmd"
Task: "Create beginner tier assessment test in workflow_tests/test/test_beginner_tier.py"
Task: "Create assessment criteria document in assessments/tier-beginner.md"

# Launch all exercises for User Story 1 together:
Task: "Create beginner exercise 1 (diagram labeling)"
Task: "Create beginner exercise 2 (pipeline decomposition)"
Task: "Create beginner exercise 3 (data flow tracing)"
```

---

## Parallel Example: User Story 2

```bash
# Launch all code implementations for User Story 2 together:
Task: "Implement service-based pipeline triggering in pipeline_controller.py"
Task: "Implement fallback path handler in fallback_handler.py"

# Launch all exercises for User Story 2 together:
Task: "Create intermediate exercise 1 (launch file creation)"
Task: "Create intermediate exercise 2 (topic remapping)"
Task: "Create intermediate exercise 3 (fallback implementation)"
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 2)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Beginner content)
4. Complete Phase 4: User Story 2 (Intermediate code + content)
5. **STOP and VALIDATE**: Run all tier tests, verify quickstart works
6. Deploy/demo if ready - students can learn beginner + intermediate concepts

### Incremental Delivery

1. Setup + Foundational → ROS 2 workspace ready
2. Add User Story 1 → Test independently → Beginner tier complete
3. Add User Story 2 → Test independently → Intermediate tier complete (MVP!)
4. Add User Story 3 → Test independently → Advanced tier complete
5. Add User Story 4 → Test independently → Performance extensions complete
6. Each story adds educational value without breaking previous tiers

### Constitution Compliance Checklist

Per Phase 7 (T065):
- [ ] Principle III: Each lesson has 2+ RAG prompts
- [ ] Principle V: Each lesson has 1-2 executable code blocks
- [ ] Principle VII: Safety warnings on motor control and recovery code

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify assessment tests fail before implementing code
- Commit after each task or logical group
- Stop at any checkpoint to validate tier independently
- All code must build with `colcon build --symlink-install`
- All code must work without Gazebo (mock nodes required)
