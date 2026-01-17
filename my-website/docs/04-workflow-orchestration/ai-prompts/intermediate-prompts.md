---
id: intermediate_prompts_workflow
title: "Intermediate AI Prompts"
sidebar_position: 65
tier: intermediate
chapter: chapter_4_workflow
---

# Intermediate AI Prompts for Workflow Orchestration

Use these prompts with your AI assistant for implementation help with ROS 2 workflows.

---

## Implementation Questions

### State Machines in ROS 2

**Prompt 1: Basic Implementation**
```
How do I implement a state machine in a ROS 2 Python node?
Show me the basic structure with state enum, transitions, and publishing.
```

**Prompt 2: State Publishing**
```
What's the best way to publish state information in ROS 2?
Should I use a custom message or standard message types?
```

**Prompt 3: Event Handling**
```
How do I handle events that trigger state transitions in ROS 2?
Should I use subscribers, timers, or service callbacks?
```

**Prompt 4: State Persistence**
```
How can I save and restore state machine state in ROS 2?
What if my node crashes and needs to recover?
```

### Launch Files

**Prompt 5: Launch File Basics**
```
Explain the structure of a ROS 2 Python launch file.
What are the key components I need to include?
```

**Prompt 6: Parameter Configuration**
```
How do I pass parameters to nodes in a launch file?
Show me examples of different parameter types (int, float, string, list).
```

**Prompt 7: Topic Remapping**
```
How do I remap topics in a launch file?
When would I need to do this in a multi-node workflow?
```

**Prompt 8: Conditional Launching**
```
How do I conditionally launch nodes based on arguments?
For example, launch different nodes for simulation vs. real hardware.
```

### Multi-Node Communication

**Prompt 9: Topics vs Services vs Actions**
```
I'm building a workflow where nodes need to communicate.
When should I use topics, services, or actions? Give me decision criteria.
```

**Prompt 10: Message Design**
```
How should I design custom messages for my workflow?
What fields should I include? What are best practices?
```

**Prompt 11: QoS Configuration**
```
Explain QoS profiles in ROS 2 for workflow reliability.
What settings should I use for critical vs. non-critical data?
```

**Prompt 12: Synchronization**
```
How do I synchronize data from multiple sensors in a pipeline?
What if they publish at different rates?
```

---

## Debugging and Troubleshooting

**Prompt 13: Node Not Starting**
```
My node doesn't start when I run the launch file.
What are common reasons and how do I debug this?
```

**Prompt 14: Topics Not Connecting**
```
My publisher and subscriber aren't connecting.
How do I debug topic communication issues in ROS 2?
```

**Prompt 15: State Machine Not Transitioning**
```
My state machine is stuck and won't transition.
How do I add debugging to see what's happening?
```

**Prompt 16: Performance Issues**
```
My multi-node workflow is slow and laggy.
How do I profile and optimize ROS 2 performance?
```

**Prompt 17: Launch File Errors**
```
I'm getting errors when running my launch file:
[paste error message]
What does this mean and how do I fix it?
```

---

## Architecture and Design

**Prompt 18: Pipeline Architecture**
```
I need to design a sensor processing pipeline with 4 stages.
How should I structure the nodes? What communication patterns should I use?
```

**Prompt 19: State Machine Design**
```
Help me design a state machine for [describe your robot task].
What states do I need? What are the transitions?
```

**Prompt 20: Error Handling**
```
How should I handle errors in a multi-node workflow?
Where should error handling logic go? In each node or centralized?
```

**Prompt 21: Scalability**
```
How do I design my workflow to be scalable?
What if I need to add more nodes or sensors later?
```

---

## Code Review and Improvement

**Prompt 22: Review My State Machine**
```
Review this state machine implementation:
[paste your code]
What can be improved? Are there any issues?
```

**Prompt 23: Review My Launch File**
```
Review this launch file:
[paste your launch file]
Is this the right way to structure it? Any improvements?
```

**Prompt 24: Code Organization**
```
How should I organize my ROS 2 workflow code?
What files should go where? What's the best package structure?
```

---

## Testing and Validation

**Prompt 25: Testing State Machines**
```
How do I test a state machine in ROS 2?
What should I test? How do I simulate different scenarios?
```

**Prompt 26: Integration Testing**
```
How do I test a complete multi-node workflow?
What tools are available for ROS 2 integration testing?
```

**Prompt 27: Simulation Testing**
```
How do I test my workflow in Gazebo simulation?
What should I verify before deploying to real hardware?
```

---

## Specific Implementation Patterns

**Prompt 28: Timer-Based State Machine**
```
Show me how to implement a state machine that transitions based on timers.
For example, stay in each state for a specific duration.
```

**Prompt 29: Event-Driven State Machine**
```
Show me how to implement a state machine that transitions based on events
from subscribers. How do I handle multiple event sources?
```

**Prompt 30: Pipeline with Feedback**
```
How do I implement a pipeline where later stages can send feedback
to earlier stages? For example, controller feedback to planner.
```

**Prompt 31: Parallel Processing**
```
How do I implement parallel processing in a ROS 2 workflow?
For example, processing camera and LIDAR data simultaneously.
```

**Prompt 32: Action-Based Workflow**
```
How do I use ROS 2 actions in a workflow?
Show me how to implement an action server and client for a long-running task.
```

---

## Real-World Scenarios

**Prompt 33: Navigation Workflow**
```
Help me implement a complete navigation workflow:
- Sensor input
- Obstacle detection
- Path planning
- Motion control
What nodes do I need? How do they communicate?
```

**Prompt 34: Manipulation Workflow**
```
I'm building a pick-and-place workflow for a robot arm.
What's the best way to structure this? State machine or pipeline?
```

**Prompt 35: Multi-Robot Coordination**
```
How do I coordinate multiple robots in a workflow?
What changes in my architecture? How do they communicate?
```

---

## Performance and Optimization

**Prompt 36: Reducing Latency**
```
My workflow has too much latency between nodes.
How can I reduce the delay? What are common bottlenecks?
```

**Prompt 37: Memory Management**
```
My workflow uses too much memory over time.
How do I identify and fix memory leaks in ROS 2 nodes?
```

**Prompt 38: CPU Usage**
```
One of my nodes is using too much CPU.
How do I profile and optimize it?
```

---

## Integration with Other Systems

**Prompt 39: Integrating with Gazebo**
```
How do I integrate my workflow with Gazebo simulation?
What changes do I need to make to my launch files?
```

**Prompt 40: Integrating with Nav2**
```
How do I integrate my state machine with Nav2 navigation stack?
What topics and actions do I need to use?
```

**Prompt 41: Integrating with MoveIt**
```
How do I integrate my workflow with MoveIt for manipulation?
How do I coordinate planning and execution?
```

---

## Best Practices

**Prompt 42: Logging Best Practices**
```
What are best practices for logging in ROS 2 workflows?
What should I log? What log levels should I use?
```

**Prompt 43: Parameter Best Practices**
```
What are best practices for using parameters in workflows?
What should be configurable? What should be hardcoded?
```

**Prompt 44: Error Recovery**
```
What are best practices for error recovery in workflows?
How do I make my system resilient to failures?
```

---

## Tips for Using These Prompts

1. **Provide Context**: Include relevant code snippets or error messages
2. **Be Specific**: Describe your exact use case and requirements
3. **Ask for Examples**: Request working code examples when possible
4. **Iterate**: Start with basic questions, then ask follow-ups
5. **Test Suggestions**: Always test AI-provided code in your environment

---

## Next Steps

After mastering intermediate concepts, continue to [Advanced AI Prompts](./advanced-prompts.md) for production-ready systems.
