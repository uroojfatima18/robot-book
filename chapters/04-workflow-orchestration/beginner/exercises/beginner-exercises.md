# Beginner Exercises - Workflow Orchestration

These exercises test your understanding of pipelines, state machines, and data handoff concepts. Complete them before moving to the Intermediate tier.

---

## Exercise 1: Design a Delivery Robot Workflow

**Difficulty**: Beginner
**Time**: 20-30 minutes
**Concepts**: Pipelines, state machines, triggers

### Scenario

You're designing a delivery robot for a hospital that transports medication from the pharmacy to patient rooms. The robot must:
- Receive delivery requests
- Navigate to the pharmacy
- Wait for staff to load medication
- Navigate to the patient room
- Wait for staff to unload medication
- Return to a charging station when battery is low

### Tasks

1. **Design the Pipeline**
   - List all the processing stages from request to completion
   - Identify the input and output of each stage
   - Draw a diagram showing the data flow

2. **Design the State Machine**
   - Identify all possible states the robot can be in
   - Define transitions between states
   - Specify what triggers each transition

3. **Identify Triggers**
   - List all triggers in your system
   - Classify each as time-based, event-based, or condition-based
   - Explain when each trigger should fire

### Acceptance Criteria

- [ ] Pipeline has at least 5 distinct stages
- [ ] State machine has at least 6 states
- [ ] All transitions have clear triggers
- [ ] Diagram clearly shows data flow
- [ ] Battery management is included

### Deliverables

Create a document (text or diagram) showing:
1. Pipeline diagram with stages and data flow
2. State machine diagram with states and transitions
3. List of triggers with classifications

---

## Exercise 2: Analyze a Traffic Light System

**Difficulty**: Beginner
**Time**: 15-20 minutes
**Concepts**: State machines, triggers

### Scenario

A traffic light system controls an intersection with two roads (North-South and East-West). The system cycles through states to allow traffic flow in different directions.

### Tasks

1. **Identify States**
   - List all possible states for the traffic light system
   - Consider both roads and all light colors (red, yellow, green)

2. **Define Transitions**
   - Specify how the system moves from one state to another
   - Identify what triggers each transition

3. **Add Emergency Mode**
   - Design an emergency state for ambulances
   - Specify how the system enters and exits emergency mode

### Acceptance Criteria

- [ ] All states are identified (at least 4)
- [ ] Transitions are clearly defined
- [ ] Emergency mode is integrated
- [ ] Timing considerations are noted

### Deliverables

A state machine diagram showing:
1. All states with clear labels
2. Transitions with trigger conditions
3. Emergency mode integration

---

## Exercise 3: Data Flow Analysis

**Difficulty**: Beginner
**Time**: 15-20 minutes
**Concepts**: Data handoff, synchronous vs asynchronous

### Scenario

A mobile robot uses multiple sensors to navigate:
- **Camera**: 30 FPS, provides RGB images
- **LIDAR**: 10 Hz, provides distance measurements
- **IMU**: 100 Hz, provides orientation and acceleration

All sensor data must be processed by a navigation system that runs at 10 Hz.

### Tasks

1. **Identify Data Flow Pattern**
   - Determine if this is pipeline, fan-out, or fan-in
   - Explain your reasoning

2. **Choose Communication Type**
   - For each sensor, decide: synchronous or asynchronous?
   - Justify your choices based on data rates

3. **Design Buffering Strategy**
   - Identify which sensors need buffering
   - Specify buffer sizes and overflow handling

### Acceptance Criteria

- [ ] Data flow pattern is correctly identified
- [ ] Communication types are justified
- [ ] Buffering strategy handles rate mismatches
- [ ] Overflow scenarios are addressed

### Deliverables

A document describing:
1. Data flow pattern with diagram
2. Communication type for each sensor with justification
3. Buffering strategy with buffer sizes

---

## Exercise 4: Implement a Simple State Machine

**Difficulty**: Beginner
**Time**: 30-40 minutes
**Concepts**: State machines, implementation

### Scenario

Implement a state machine for a robot vacuum cleaner with these states:
- **IDLE**: Waiting for start command
- **CLEANING**: Actively cleaning
- **RETURNING**: Returning to charging dock
- **CHARGING**: Charging battery
- **ERROR**: Something went wrong

### Tasks

1. **Define Transitions**
   - Specify all possible transitions between states
   - Define trigger conditions for each transition

2. **Implement in Python**
   - Use the provided template below
   - Add transition logic
   - Test with different scenarios

3. **Add Error Handling**
   - Implement error detection
   - Define recovery behaviors

### Template

```python
from enum import Enum

class VacuumState(Enum):
    IDLE = "idle"
    CLEANING = "cleaning"
    RETURNING = "returning"
    CHARGING = "charging"
    ERROR = "error"

class VacuumStateMachine:
    def __init__(self):
        self.state = VacuumState.IDLE
        self.battery_level = 100
        self.error_detected = False

    def start_cleaning(self):
        """Transition to CLEANING state"""
        # TODO: Implement transition logic
        pass

    def battery_low(self):
        """Transition to RETURNING state"""
        # TODO: Implement transition logic
        pass

    def reached_dock(self):
        """Transition to CHARGING state"""
        # TODO: Implement transition logic
        pass

    def fully_charged(self):
        """Transition to IDLE state"""
        # TODO: Implement transition logic
        pass

    def detect_error(self):
        """Transition to ERROR state"""
        # TODO: Implement transition logic
        pass

    def get_state(self):
        return self.state.value

# Test your implementation
vacuum = VacuumStateMachine()
print(f"Initial state: {vacuum.get_state()}")

# TODO: Add test scenarios
```

### Acceptance Criteria

- [ ] All transitions are implemented
- [ ] State machine handles invalid transitions gracefully
- [ ] Error state is reachable and recoverable
- [ ] Test scenarios cover all states

### Deliverables

1. Completed Python code with all transitions implemented
2. Test output showing state transitions
3. Brief explanation of design decisions

---

## Exercise 5: Design a Multi-Component Workflow

**Difficulty**: Intermediate
**Time**: 30-40 minutes
**Concepts**: Pipelines, state machines, data handoff

### Scenario

Design a complete workflow for a robot that inspects products on an assembly line:

**Components:**
1. **Vision System**: Captures images of products
2. **Defect Detector**: Analyzes images for defects
3. **Quality Classifier**: Classifies products as pass/fail
4. **Sorting Actuator**: Physically sorts products
5. **Database Logger**: Records inspection results

**Requirements:**
- Process 10 products per minute
- Handle camera failures gracefully
- Log all inspections to database
- Alert operator on repeated failures

### Tasks

1. **Design the Pipeline**
   - Define the complete data flow
   - Specify data format at each stage
   - Identify bottlenecks

2. **Design State Machines**
   - Create a state machine for the overall system
   - Create a state machine for error handling
   - Define recovery behaviors

3. **Choose Data Passing Patterns**
   - Specify synchronous vs asynchronous for each connection
   - Design buffering strategy
   - Handle component failures

4. **Define Triggers**
   - List all triggers in the system
   - Specify trigger priorities
   - Handle conflicting triggers

### Acceptance Criteria

- [ ] Complete pipeline diagram with all components
- [ ] State machines for system and error handling
- [ ] Data passing patterns are justified
- [ ] Failure scenarios are addressed
- [ ] Performance requirements are met

### Deliverables

A comprehensive design document including:
1. Pipeline architecture diagram
2. State machine diagrams
3. Data flow specifications
4. Failure handling strategy
5. Performance analysis

---

## Self-Assessment Questions

After completing the exercises, test your understanding:

### Conceptual Understanding

1. **What is the difference between a pipeline and a state machine?**
   - When would you use each?
   - Can they be combined?

2. **Why is buffering important in robotic workflows?**
   - What happens without buffering?
   - How do you size buffers appropriately?

3. **What are the tradeoffs between synchronous and asynchronous communication?**
   - When is each appropriate?
   - What are the failure modes?

### Practical Application

4. **How would you debug a workflow that's getting stuck?**
   - What tools would you use?
   - What would you check first?

5. **How do you handle component failures in a workflow?**
   - What recovery strategies exist?
   - How do you prevent cascading failures?

---

## Additional Challenges

If you want to go deeper:

### Challenge 1: Hierarchical State Machines
Design a hierarchical state machine where states contain sub-states. Example: A "CLEANING" state that has sub-states for different cleaning modes.

### Challenge 2: Priority Queues
Implement a priority queue system where urgent tasks (e.g., emergency stop) preempt normal tasks.

### Challenge 3: Workflow Optimization
Analyze a workflow and identify opportunities for parallelization to improve throughput.

---

## Getting Help

If you're stuck on an exercise:

1. **Review the Lessons**: Go back to the relevant lesson for concepts
2. **Use AI Assistance**: Check [Beginner AI Prompts](../../ai-prompts/beginner-prompts.md)
3. **Start Simple**: Break complex exercises into smaller parts
4. **Draw Diagrams**: Visual representations often clarify thinking

---

## Next Steps

After completing these exercises:

1. **Review Your Solutions**: Compare with the concepts from lessons
2. **Self-Assess**: Use the questions above to test understanding
3. **Move to Intermediate Tier**: Ready to implement in ROS 2!

**Congratulations on completing the Beginner tier!** You now understand the fundamental concepts of workflow orchestration. The Intermediate tier will teach you to implement these concepts in real ROS 2 systems.
