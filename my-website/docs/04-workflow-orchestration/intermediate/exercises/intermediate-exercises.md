---
id: intermediate_exercises_workflow
title: "Intermediate Tier Exercises"
sidebar_position: 58
tier: intermediate
chapter: chapter_4_workflow
section: exercises
---

# Intermediate Tier Exercises

This document consolidates all exercises from the Intermediate tier lessons for hands-on practice with ROS 2 workflow implementation.

---

## Exercise 01: Delivery Robot State Machine

**Objective**: Implement a complete state machine for a delivery robot in ROS 2.

**Scenario**: A delivery robot needs to navigate to a destination, deliver a package, and return to base.

**Requirements**:

1. **States**: IDLE, NAVIGATING_TO_DESTINATION, DELIVERING, NAVIGATING_TO_BASE, CHARGING, ERROR
2. **Transitions**: Define clear transition conditions
3. **ROS 2 Integration**: Publish current state on `/robot_state` topic
4. **Simulation**: Test in Gazebo or similar

**Tasks**:

1. Create a ROS 2 Python node with state machine logic
2. Implement state transitions based on events (battery level, goal reached, etc.)
3. Publish state changes to a topic
4. Add logging for debugging
5. Test all state transitions

**Acceptance Criteria**:
- [ ] All 6 states implemented
- [ ] State transitions work correctly
- [ ] State published on topic
- [ ] Tested in simulation
- [ ] Code is well-documented

**Starter Code**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum

class RobotState(Enum):
    IDLE = "idle"
    NAVIGATING_TO_DESTINATION = "navigating_to_destination"
    DELIVERING = "delivering"
    NAVIGATING_TO_BASE = "navigating_to_base"
    CHARGING = "charging"
    ERROR = "error"

class DeliveryRobotStateMachine(Node):
    def __init__(self):
        super().__init__('delivery_robot_sm')
        self.state = RobotState.IDLE

        # TODO: Create state publisher
        # TODO: Create timers for state updates
        # TODO: Create subscribers for events

    def update_state(self):
        # TODO: Implement state transition logic
        pass

    def publish_state(self):
        # TODO: Publish current state
        pass

def main():
    rclpy.init()
    node = DeliveryRobotStateMachine()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Exercise 02: Sensor Processing Pipeline

**Objective**: Create a multi-node pipeline that processes sensor data through multiple stages.

**Pipeline Stages**:
1. **Sensor Node**: Publishes simulated LIDAR data
2. **Filter Node**: Filters out noise and invalid readings
3. **Processor Node**: Detects obstacles from filtered data
4. **Controller Node**: Makes decisions based on obstacles

**Requirements**:

1. Each stage is a separate ROS 2 node
2. Nodes communicate via topics
3. Launch file starts all nodes
4. Parameters configure each node

**Tasks**:

1. Implement all 4 nodes
2. Create appropriate message types or use standard messages
3. Write a launch file to start the pipeline
4. Add parameters for tuning (filter threshold, detection range, etc.)
5. Test the complete pipeline

**Acceptance Criteria**:
- [ ] All 4 nodes implemented and working
- [ ] Data flows correctly through pipeline
- [ ] Launch file starts entire system
- [ ] Parameters are configurable
- [ ] Pipeline tested end-to-end

---

## Exercise 03: Multi-Node Launch File

**Objective**: Create a comprehensive launch file for a robotic workflow.

**System Components**:
- State machine node
- Sensor nodes (camera, LIDAR)
- Navigation node
- Controller node

**Requirements**:

1. Start all nodes with one command
2. Configure parameters for each node
3. Remap topics for proper connections
4. Set up logging and output

**Tasks**:

1. Write a Python launch file
2. Configure each node with appropriate parameters
3. Set up topic remappings
4. Add conditional node launching (e.g., simulation vs. real hardware)
5. Test the launch file

**Acceptance Criteria**:
- [ ] All nodes start correctly
- [ ] Parameters are properly set
- [ ] Topics are correctly remapped
- [ ] Conditional launching works
- [ ] System operates as expected

**Starter Code**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # TODO: Declare launch arguments

    # TODO: Create node configurations

    # TODO: Return LaunchDescription with all nodes

    return LaunchDescription([
        # Add nodes here
    ])
```

---

## Exercise 04: Debug a Broken Workflow

**Objective**: Identify and fix issues in a multi-node workflow.

**Scenario**: You're given a workflow that doesn't work correctly. Your job is to debug and fix it.

**Common Issues to Look For**:
- Topic name mismatches
- Message type incompatibilities
- Missing dependencies
- Incorrect QoS settings
- Race conditions
- Parameter errors

**Tasks**:

1. Run the provided broken workflow
2. Use ROS 2 debugging tools to identify issues
3. Fix all problems
4. Verify the workflow works correctly

**Debugging Tools to Use**:
```bash
ros2 node list
ros2 topic list
ros2 topic info /topic_name
ros2 topic echo /topic_name
ros2 node info /node_name
ros2 param list /node_name
```

**Acceptance Criteria**:
- [ ] All issues identified
- [ ] All issues fixed
- [ ] Workflow operates correctly
- [ ] Documentation of what was wrong and how it was fixed

---

## Capstone Project: Complete Navigation Workflow

**Objective**: Build a complete navigation workflow with state machine, sensors, planning, and control.

**System Architecture**:

```
Sensor Nodes → Perception → State Machine → Path Planner → Controller → Robot
     ↓                                                                      ↓
  Raw Data                                                            Feedback
```

**Requirements**:

1. **State Machine**: Manages robot behavior (IDLE, PLANNING, NAVIGATING, AVOIDING, GOAL_REACHED)
2. **Sensor Processing**: Processes LIDAR/camera data
3. **Path Planning**: Calculates paths to goals
4. **Controller**: Executes motion commands
5. **Launch System**: Orchestrates all components
6. **Error Handling**: Handles sensor failures, planning failures, etc.

**Tasks**:

1. Design the complete system architecture
2. Implement all nodes
3. Create launch files
4. Test in simulation
5. Document the system

**Acceptance Criteria**:
- [ ] Complete system implemented
- [ ] All components communicate correctly
- [ ] State machine manages workflow
- [ ] Robot navigates to goals successfully
- [ ] Error handling works
- [ ] System is well-documented

**Deliverables**:
- Source code for all nodes
- Launch files
- Configuration files
- README with setup and usage instructions
- Video demonstration in simulation

---

## Self-Assessment Checklist

After completing all intermediate exercises, verify you can:

### State Machine Implementation
- [ ] Implement FSM in ROS 2 Python nodes
- [ ] Manage state transitions based on events
- [ ] Publish state information for monitoring
- [ ] Handle edge cases and errors

### Multi-Node Systems
- [ ] Create multi-node workflows
- [ ] Write launch files for orchestration
- [ ] Configure nodes with parameters
- [ ] Remap topics for flexible connections

### Communication Patterns
- [ ] Choose appropriate communication patterns (topics/services/actions)
- [ ] Design effective message structures
- [ ] Configure QoS for reliability
- [ ] Debug communication issues

### Ready for Advanced?
If you checked all boxes above, you're ready to proceed to the Advanced tier where you'll add fault tolerance and production-ready features!

---

## Next Steps

Continue to [Advanced Tier: Fault Tolerance & Production Systems](../../advanced/README.md)
