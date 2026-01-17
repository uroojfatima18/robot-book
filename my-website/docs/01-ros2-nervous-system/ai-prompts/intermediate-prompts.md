---
id: intermediate_ai_prompts
title: "Intermediate Tier AI Learning Prompts"
sidebar_position: 9
tier: intermediate
chapter: chapter_1_ros2
---

# AI-Assisted Learning Prompts: Intermediate Tier

This document provides curated prompts for AI assistants to help reinforce learning from the Intermediate tier lessons on creating ROS 2 Python nodes.

---

## How to Use These Prompts

1. **Copy** a prompt matching your learning goal
2. **Paste** into your AI assistant (Claude, ChatGPT, etc.)
3. **Include** your specific code or error messages when debugging
4. **Verify** responses against ROS 2 documentation

---

## I1: Nodes, Topics, Services, and Actions

### Conceptual Understanding

#### Publisher/Subscriber Deep Dive
```
I'm learning about ROS 2 publishers and subscribers. Can you explain:
1. What happens internally when I call publisher.publish()?
2. How does the message get from publisher to subscriber?
3. What role does the queue size (depth) play?
4. When would messages be dropped or lost?
```

#### QoS Explained
```
I'm confused about Quality of Service (QoS) in ROS 2. Can you explain:
1. What's the difference between RELIABLE and BEST_EFFORT?
2. When should I use TRANSIENT_LOCAL durability?
3. How do I choose the right QoS for camera images vs motor commands?
4. What happens when publisher and subscriber have incompatible QoS?
```

#### Services vs Topics
```
I understand both topics and services, but I'm not sure when to use each.

Scenario: I want to get the current robot position.
- Option A: Subscribe to /robot/pose topic
- Option B: Call /get_robot_pose service

When would each option be better? What are the tradeoffs?
```

#### Understanding Spin
```
I'm confused about rclpy.spin() and how callbacks work. Can you explain:
1. What does "spinning" actually do?
2. Why do I need to spin? What happens if I don't?
3. What's the difference between spin(), spin_once(), and spin_until_future_complete()?
4. How does spin interact with timers and subscriptions?
```

### Debugging Help

#### Subscriber Not Receiving
```
My subscriber callback never gets called, but the publisher is definitely running.

Setup:
- Publisher publishes to `/sensor/data` at 10 Hz
- Subscriber subscribes to `/sensor/data`
- `ros2 topic echo /sensor/data` shows messages
- My callback function is never called

Code:
[PASTE YOUR SUBSCRIBER CODE HERE]

What should I check?
```

#### Service Timeout
```
My service client keeps timing out even though the server is running.

Error: "Service /my_service not available"

I verified:
- `ros2 service list` shows /my_service
- `ros2 service type /my_service` shows correct type
- Server node is running

What could cause this? How do I debug?
```

#### Message Type Mismatch
```
I'm getting this error when trying to publish:
[PASTE ERROR HERE]

My code:
[PASTE RELEVANT CODE]

How do I fix the message type issue?
```

### Extension Ideas

#### Combined Publisher/Subscriber
```
I want to create a node that:
1. Subscribes to /input topic
2. Processes the data
3. Publishes result to /output topic

What's the best pattern for this? Should I use the same callback group?
Show me a clean implementation.
```

#### Custom Message Types
```
I want to create a custom message type for my humanoid robot's joint states.
It should include:
- Joint names (string array)
- Positions (float array)
- Velocities (float array)
- Timestamp

How do I:
1. Define the message?
2. Build it in my package?
3. Use it in my Python code?
```

---

## I2: Python ROS Bridge (rclpy)

### Conceptual Understanding

#### Node Lifecycle
```
I'm learning about ROS 2 lifecycle nodes. Can you explain:
1. What are the lifecycle states and transitions?
2. When would I use a lifecycle node vs a regular node?
3. How do I implement configure/activate/deactivate callbacks?
4. How do lifecycle nodes help with hardware drivers?
```

#### Parameters Deep Dive
```
I want to understand ROS 2 parameters better. Can you explain:
1. What types of parameters are supported?
2. How do I make a parameter read-only?
3. Can I have nested parameters (like YAML structures)?
4. How do parameter events work for monitoring changes?
```

#### Executors and Threading
```
I'm confused about executors and callback groups. Can you explain:
1. What's the difference between SingleThreaded and MultiThreaded executors?
2. When do I need MutuallyExclusiveCallbackGroup vs ReentrantCallbackGroup?
3. Can a timer callback and subscription callback run at the same time?
4. How many threads should I use in MultiThreadedExecutor?
```

#### Launch Files
```
I want to understand ROS 2 launch files better. Can you explain:
1. Why are launch files Python instead of XML like ROS 1?
2. What's the difference between LaunchConfiguration and a Python variable?
3. How do I conditionally launch nodes based on arguments?
4. How do I include another launch file?
```

### Debugging Help

#### Parameter Not Found
```
I declared a parameter but get "Parameter not declared" error.

My code:
```python
self.declare_parameter('my_param', 'default')
value = self.get_parameter('my_param').value
```

Error: [PASTE ERROR]

What am I doing wrong?
```

#### Launch File Issues
```
My launch file runs but nodes can't find each other.

Launch file:
[PASTE LAUNCH FILE]

Issue: Publisher and subscriber don't communicate even though both start.

How do I debug namespace and topic issues in launch files?
```

#### Callback Not Thread-Safe
```
I'm using MultiThreadedExecutor and getting race conditions.

Symptoms:
- Variable values are inconsistent
- Occasional crashes
- Different results on different runs

My code:
[PASTE CODE WITH SHARED STATE]

How do I make my callbacks thread-safe?
```

### Extension Ideas

#### Dynamic Reconfigure Pattern
```
I want to implement a pattern where:
1. Node reads initial config from YAML file
2. Parameters can be changed at runtime via ros2 param set
3. Some parameters trigger immediate action (like reconnecting)
4. Invalid values are rejected with helpful error messages

Show me the implementation pattern for this.
```

#### Complex Launch System
```
I'm launching a multi-robot system. I need:
1. Launch 3 robots with different namespaces
2. Each robot has 5 nodes
3. Some nodes should wait for others to start
4. Parameters should be loaded from per-robot YAML files

How do I structure this launch system?
```

#### Composable Nodes
```
I've heard about composable nodes for better performance.
1. What are composable nodes?
2. When should I use them vs regular nodes?
3. How do I convert my existing node to composable?
4. What are the limitations?
```

---

## Practice Scenarios

### Scenario 1: Debug This System
```
A robot system has these nodes:
- sensor_node: publishes to /imu/data at 100 Hz
- filter_node: subscribes to /imu/data, publishes to /imu/filtered at 100 Hz
- logger_node: subscribes to /imu/filtered, logs every 10th message

Problem: logger_node only receives about 50% of messages.

Given this information, what are the likely causes and how would you debug?
```

### Scenario 2: Design a Node
```
Design a ROS 2 node for a gripper controller with these requirements:

Inputs:
- Subscribe to /gripper/command (open/close commands)
- Subscribe to /gripper/force (force sensor readings)

Outputs:
- Publish to /gripper/state (current state)
- Service /gripper/calibrate (run calibration routine)

Parameters:
- max_force: Maximum grip force
- close_speed: How fast to close

Provide:
1. Node class structure
2. Callback group strategy
3. Parameter handling approach
```

### Scenario 3: Launch File Challenge
```
Create a launch file for a robot simulation:

Requirements:
- Start Gazebo with a robot model
- Launch robot_state_publisher for TF
- Launch a teleop node if 'teleop' argument is true
- Use simulation time
- Load parameters from config/robot.yaml

Arguments:
- world: path to world file (default: empty.world)
- teleop: enable keyboard control (default: false)
- namespace: robot namespace (default: "robot")

Sketch the launch file structure.
```

---

## Prompt Templates

### General Learning
```
I'm working on [TOPIC] in ROS 2 Python development.

Current understanding: [WHAT YOU KNOW]

Specific question: [YOUR QUESTION]

Code context (if relevant):
[YOUR CODE]
```

### Debugging Template
```
I'm getting [ERROR/UNEXPECTED BEHAVIOR] in my ROS 2 Python node.

Environment:
- ROS 2 version: [HUMBLE/IRON/JAZZY]
- Python version: [VERSION]
- OS: [OS]

What I'm trying to do:
[GOAL]

My code:
[RELEVANT CODE SECTIONS]

Error/Behavior:
[ERROR MESSAGE OR DESCRIPTION]

What I've tried:
[TROUBLESHOOTING STEPS]
```

### Code Review Template
```
Please review my ROS 2 Python code for:
1. Correctness
2. Best practices
3. Potential issues
4. Performance considerations

My code:
[YOUR CODE]

Specific concerns:
[ANYTHING YOU'RE UNSURE ABOUT]
```

---

## Tips for Effective Learning

1. **Run the code**: Don't just read examples—run them and modify them

2. **Use ros2 CLI**: Tools like `ros2 topic list`, `ros2 node info`, and `ros2 param list` are essential for debugging

3. **Check QoS compatibility**: Many communication issues are QoS mismatches

4. **Start simple**: Get basic pub/sub working before adding parameters, lifecycle, etc.

5. **Use logging**: `self.get_logger().info()` is your friend for debugging

6. **Read the source**: rclpy is open source—reading it helps understand behavior

---

## Next Steps

After mastering intermediate concepts, proceed to:
- [Advanced AI Prompts](./advanced-prompts.md) - URDF, actions, complex patterns
