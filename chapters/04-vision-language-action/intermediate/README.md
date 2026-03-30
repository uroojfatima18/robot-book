# Intermediate Tier: Implementing ROS 2 Workflows

**Duration**: 2-4 hours | **Prerequisite**: Beginner Tier Completion

---

## Overview

Welcome to the Intermediate tier! You've learned workflow concepts - now it's time to implement them in ROS 2. You'll build state machines, coordinate multiple nodes, and create production-ready workflows for mobile robot navigation.

In this tier, you'll use ROS 2 topics, services, and launch files to orchestrate complex robotic behaviors with proper error handling and recovery.

---

## Learning Objectives

By the end of this tier, you will be able to:

1. **Implement** finite state machines in Python for ROS 2 nodes
2. **Coordinate** multiple ROS 2 nodes in a workflow
3. **Create** launch files that start complete workflow pipelines
4. **Pass data** between nodes using topics and services
5. **Handle errors** and implement recovery behaviors
6. **Monitor** workflow execution and debug issues
7. **Build** a complete navigation workflow with state management

---

## What You'll Learn

### ROS 2 State Machine Implementation
- Implementing FSMs in Python ROS 2 nodes
- State transition logic and event handling
- Publishing state information for monitoring
- Integrating state machines with ROS 2 lifecycle

### Multi-Node Coordination
- Launching multiple nodes with dependencies
- Topic-based communication between workflow stages
- Service calls for synchronous operations
- Action servers for long-running tasks

### Launch File Orchestration
- Creating complex launch files
- Managing node dependencies and startup order
- Passing parameters between nodes
- Conditional node launching

### Error Handling and Recovery
- Detecting failures in workflow components
- Implementing fallback behaviors
- Timeout handling and watchdogs
- Graceful degradation strategies

---

## Lesson Structure

This tier contains **3 comprehensive lessons**:

### Lesson I1: Implementing ROS 2 State Machines
**File**: [I1-ros2-state-machines.md](./I1-ros2-state-machines.md)
**Duration**: 60-90 minutes

Build state machines in ROS 2:
- FSM implementation patterns in Python
- State transition logic
- Publishing state for monitoring
- Integration with ROS 2 nodes

**Includes**: Complete Python ROS 2 node with FSM.

### Lesson I2: Multi-Node Workflow Coordination
**File**: [I2-multi-node-coordination.md](./I2-multi-node-coordination.md)
**Duration**: 60-90 minutes

Coordinate multiple nodes:
- Topic-based data passing
- Service-based synchronization
- Action-based long-running tasks
- Node dependency management

**Includes**: Multi-node workflow example with launch file.

### Lesson I3: Launch Files and Pipeline Startup
**File**: [I3-launch-pipeline-startup.md](./I3-launch-pipeline-startup.md)
**Duration**: 60-90 minutes

Master launch file orchestration:
- Complex launch file creation
- Parameter passing and configuration
- Conditional node launching
- Startup sequencing and dependencies

**Includes**: Production-ready launch file examples.

---

## Prerequisites

### Knowledge Prerequisites
- **Beginner Tier Completion**: Strong understanding of workflow concepts
- **Chapter 1 Intermediate**: ROS 2 nodes, topics, services, actions
- **Chapter 3 Intermediate**: Navigation basics (for examples)
- **Python Proficiency**: Classes, async/await, decorators

### Technical Prerequisites
- **ROS 2 Humble or Iron** installed and working
- **Gazebo Classic** with TurtleBot3 or similar robot
- **Nav2** installed (for navigation examples)
- **Python 3.10+** with ROS 2 packages
- **Workspace**: `~/ros2_ws` set up from previous chapters

---

## Time Commitment

| Activity | Time |
|----------|------|
| Lesson I1: State Machines | 60-90 min |
| Lesson I2: Multi-Node Coordination | 60-90 min |
| Lesson I3: Launch Files | 60-90 min |
| Exercises | 45-60 min |
| **Total** | **3-5 hours** |

---

## Learning Path

```
Beginner Tier Complete
    ↓
I1: ROS 2 State Machines (60-90 min)
    ↓
I2: Multi-Node Coordination (60-90 min)
    ↓
I3: Launch Files & Pipeline Startup (60-90 min)
    ↓
Intermediate Exercises (45-60 min)
    ↓
Ready for Advanced Tier!
```

---

## What You'll Build

By the end of this tier, you'll have:

1. **Navigation Workflow**: Complete state machine for autonomous navigation
2. **Multi-Node Pipeline**: Coordinated sensor processing and decision making
3. **Launch System**: Production-ready launch files for complex workflows
4. **Error Handling**: Robust failure detection and recovery
5. **Monitoring Tools**: State visualization and debugging capabilities

### Example Project: Waypoint Navigation Workflow
You'll build a complete system where a robot:
- Receives a list of waypoints
- Navigates to each waypoint in sequence
- Handles obstacles and failures
- Reports progress and status
- Recovers from navigation failures

---

## Success Criteria

You're ready to move to the Advanced tier when you can:

- [ ] Implement a finite state machine in a ROS 2 node
- [ ] Coordinate multiple nodes using topics and services
- [ ] Create launch files that start complex workflows
- [ ] Pass data between nodes reliably
- [ ] Implement error detection and recovery
- [ ] Debug workflow issues using ROS 2 tools
- [ ] Build a complete navigation workflow from scratch

---

## Development Environment

### Workspace Structure
```bash
~/ros2_ws/
├── src/
│   ├── workflow_orchestration/
│   │   ├── workflow_orchestration/
│   │   │   ├── __init__.py
│   │   │   ├── navigation_fsm.py
│   │   │   ├── waypoint_manager.py
│   │   │   └── workflow_monitor.py
│   │   ├── launch/
│   │   │   ├── navigation_workflow.launch.py
│   │   │   └── multi_node_pipeline.launch.py
│   │   ├── config/
│   │   │   └── workflow_params.yaml
│   │   ├── package.xml
│   │   └── setup.py
│   └── [other packages from previous chapters]
├── build/
├── install/
└── log/
```

### Required Tools
- **VS Code** with ROS 2 extensions
- **RViz2** for visualization
- **rqt_graph** for node visualization
- **rqt_console** for log monitoring
- **Gazebo** for simulation

---

## Common Challenges

### Challenge 1: State Transitions Not Working
**Symptoms**: State machine gets stuck or transitions incorrectly
**Solutions**:
- Add logging to track state changes
- Verify transition conditions are correct
- Check for race conditions in callbacks
- Use state visualization tools

### Challenge 2: Nodes Not Communicating
**Symptoms**: Data not flowing between nodes
**Solutions**:
- Verify topic names match exactly
- Check QoS compatibility
- Use `ros2 topic echo` to verify publishing
- Ensure nodes are running (`ros2 node list`)

### Challenge 3: Launch File Fails
**Symptoms**: Nodes don't start or crash immediately
**Solutions**:
- Check Python syntax in launch file
- Verify all node executables exist
- Review parameter file syntax
- Check log files for specific errors

### Challenge 4: Timing Issues
**Symptoms**: Nodes start before dependencies are ready
**Solutions**:
- Add startup delays where needed
- Use lifecycle nodes for controlled startup
- Implement readiness checks
- Add retry logic for service calls

### Challenge 5: Memory Leaks
**Symptoms**: Memory usage grows over time
**Solutions**:
- Properly destroy subscriptions and timers
- Avoid circular references
- Use weak references where appropriate
- Profile with memory tools

---

## Development Workflow

### Iterative Development
1. **Start with Single Node**: Get one component working first
2. **Add Nodes Incrementally**: Add one node at a time
3. **Test Integration**: Verify communication after each addition
4. **Add Error Handling**: Implement recovery after basic functionality works
5. **Optimize**: Tune performance once everything works

### Debugging Strategy
1. **Check Node Status**: `ros2 node list` and `ros2 node info`
2. **Verify Topics**: `ros2 topic list` and `ros2 topic echo`
3. **Monitor Logs**: `ros2 run rqt_console rqt_console`
4. **Visualize Graph**: `ros2 run rqt_graph rqt_graph`
5. **Test Components**: Test nodes individually before integration

---

## Code Examples

All lessons include production-ready code:
- **Complete ROS 2 Nodes**: Fully functional, not just snippets
- **Error Handling**: Robust to common failures
- **Type Hints**: Clear interfaces
- **Documentation**: Docstrings and comments
- **Best Practices**: Follow ROS 2 guidelines

### Example: Simple State Machine Node

```python
# Simplified example - full version in I1 lesson
import rclpy
from rclpy.node import Node
from enum import Enum

class RobotState(Enum):
    IDLE = 0
    NAVIGATING = 1
    REACHED_GOAL = 2

class NavigationFSM(Node):
    def __init__(self):
        super().__init__('navigation_fsm')
        self.state = RobotState.IDLE
        self.timer = self.create_timer(0.1, self.state_machine_update)

    def state_machine_update(self):
        if self.state == RobotState.IDLE:
            self.get_logger().info('Waiting for goal...')
        elif self.state == RobotState.NAVIGATING:
            self.get_logger().info('Navigating to goal...')
        # ... more state logic
```

---

## Assets Provided

### Code Examples
- `code/navigation_fsm.py` - Complete navigation state machine
- `code/waypoint_manager.py` - Waypoint sequencing node
- `code/workflow_monitor.py` - Workflow monitoring and visualization
- `code/error_handler.py` - Error detection and recovery

### Launch Files
- `launch/navigation_workflow.launch.py` - Complete navigation workflow
- `launch/multi_node_pipeline.launch.py` - Multi-node coordination example
- `launch/monitored_workflow.launch.py` - Workflow with monitoring

### Configuration Files
- `config/workflow_params.yaml` - Workflow parameters
- `config/fsm_config.yaml` - State machine configuration
- `config/recovery_behaviors.yaml` - Recovery behavior settings

---

## Tools and Resources

### ROS 2 Tools
- `ros2 node list/info` - Node inspection
- `ros2 topic list/echo/hz` - Topic monitoring
- `ros2 service list/call` - Service testing
- `ros2 action list/send_goal` - Action testing
- `ros2 launch` - Launch file execution

### Debugging Tools
- **rqt_graph**: Visualize node connections
- **rqt_console**: Monitor logs in real-time
- **rqt_plot**: Plot numeric data over time
- **RViz2**: Visualize robot state and navigation

### External Resources
- [ROS 2 Launch Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [Nav2 Behavior Trees](https://navigation.ros.org/behavior_trees/index.html)

---

## Best Practices

### State Machine Design
- Keep states simple and focused
- Make transitions explicit and well-defined
- Log all state changes for debugging
- Publish current state for monitoring

### Node Communication
- Use topics for continuous data streams
- Use services for request-response patterns
- Use actions for long-running tasks with feedback
- Choose appropriate QoS profiles

### Launch File Organization
- Group related nodes together
- Use parameters for configuration
- Document node dependencies
- Include comments explaining complex logic

### Error Handling
- Detect errors early
- Implement graceful degradation
- Log errors with context
- Provide recovery mechanisms

---

## Next Steps

After completing this tier:

1. **Complete the Intermediate Exercises**: Build a complete workflow system
2. **Experiment**: Try different workflow patterns and recovery strategies
3. **Move to Advanced Tier**: Learn production-ready fault tolerance and monitoring
4. **Test on Real Hardware**: Deploy your workflows to a physical robot

---

## Ready to Implement?

**Start with Lesson I1**: [Implementing ROS 2 State Machines](./I1-ros2-state-machines.md)

You'll learn to implement finite state machines in ROS 2 nodes, the foundation for all workflow orchestration.

---

**Pro Tip**: Start simple and add complexity incrementally. Get a basic state machine working first, then add more states, error handling, and monitoring. Test each addition before moving forward.

**Let's build production-ready workflows!**
