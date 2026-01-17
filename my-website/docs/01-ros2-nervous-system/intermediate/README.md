---
id: chapter_1_intermediate_tier
title: "Intermediate Tier: Hands-On ROS 2 Programming"
sidebar_position: 5
tier: intermediate
chapter: chapter_1_ros2
estimated_time: "2-4 hours"
---

# Intermediate Tier: Hands-On ROS 2 Programming

## Welcome to the Intermediate Tier

Now that you understand ROS 2 concepts, it's time to write code. This tier teaches you *how* to create ROS 2 nodes in Python, implement the pub/sub pattern, build services, and orchestrate multiple nodes.

---

## Tier Overview

```
🟡 INTERMEDIATE TIER - Implementation & Practice
════════════════════════════════════════════════════

What You'll Learn:
• Creating ROS 2 nodes in Python using rclpy
• Publisher/Subscriber pattern with working code
• Services and Service servers
• Launch files for orchestrating nodes
• Parameters and configurations
• Debugging and monitoring ROS 2 systems

What You'll Build:
• A minimal publisher node
• A subscriber node that receives and processes data
• A service server
• A launch file that starts multiple nodes together
• Real communication between your custom nodes
```

---

## Learning Objectives

By the end of the Intermediate tier, you will be able to:

1. **Create** a ROS 2 Python node from scratch
2. **Implement** a publisher that sends messages to a topic
3. **Implement** a subscriber that receives and processes messages
4. **Build** a service server and client for request-response patterns
5. **Write** launch files to coordinate multiple nodes
6. **Use** parameters for runtime configuration
7. **Debug** ROS 2 systems using command-line tools
8. **Understand** executors and callback groups
9. **Apply** best practices for ROS 2 Python development

---

## Prerequisites

Before starting this tier, you must have completed:

- **The Beginner Tier** ✅
  - You understand nodes, topics, services, and actions conceptually
  - You have ROS 2 installed and verified
  - You've run the talker/listener demo

- **Python 3.10+ Basics** (language features, not robotics):
  - Classes and inheritance
  - Functions and decorators
  - Import statements and module structure
  - Exception handling (try/except)

- **Command-line Proficiency**:
  - Running ROS 2 commands: `ros2 run`, `ros2 topic`, `ros2 service`
  - Navigating and editing files
  - Setting environment variables

**Important**: If you haven't completed the Beginner tier, go back now. This tier assumes solid conceptual understanding.

---

## Lessons in This Tier

### Lesson I1: Nodes, Topics, Services, and Actions
**Duration**: 1-2 hours

Deep dive into ROS 2 communication patterns with working Python code. Understand how to create nodes, how publishers/subscribers work, when to use services, and how actions differ.

**Key Topics**:
- The Node class in rclpy
- Creating Publishers and Subscribers
- Message types and custom messages
- Services: Request-response pattern
- Actions: Long-running tasks with feedback
- QoS (Quality of Service) profiles
- Best practices for node design

**Hands-On Activities**:
- Create a minimal publisher node
- Create a subscriber that prints messages
- Implement a simple service
- Test pub/sub communication using `ros2 topic` CLI

**Outcomes**:
- ✅ First working Python ROS 2 node
- ✅ Understanding pub/sub in practice
- ✅ Service implementation basics

**File**: [I1: Nodes, Topics, Services, and Actions](./01-nodes-topics.md)

---

### Lesson I2: Python ROS Bridge (rclpy)
**Duration**: 1-2 hours

Master the rclpy library: executors, timers, parameters, launch files, and debugging. Learn how to build production-ready ROS 2 Python applications.

**Key Topics**:
- rclpy architecture: Nodes, executors, and callbacks
- Timer callbacks for periodic tasks
- Parameter handling and dynamic reconfiguration
- Launch files (Python syntax)
- Debugging with logging
- Common patterns: spin vs. spin_once
- Callback groups and execution models
- Multi-threaded ROS 2 nodes

**Hands-On Activities**:
- Build a node with a timer callback
- Create parameters and access them
- Write a launch file that starts two nodes
- Use logging to debug node behavior

**Outcomes**:
- ✅ Multi-node ROS 2 systems
- ✅ Parameter-driven configuration
- ✅ Professional-grade code structure

**File**: [I2: Python ROS Bridge (rclpy)](./02-python-ros-bridge.md)

---

## Progression & Scaffolding

The Intermediate tier builds on Beginner foundations and prepares for Advanced patterns:

```
Beginner Tier (Concept)          Intermediate Tier (Code)           Advanced Tier (Patterns)
└─ Nodes exist                   └─ Create nodes                     └─ Complex architectures
└─ Topics carry messages         └─ Publish/Subscribe code           └─ Action servers with feedback
└─ Services do request/response  └─ Implement services               └─ URDF models and TF2
                                    └─ Build launch files                └─ AI integration patterns
                                       └─ Debug with tools
```

---

## Estimated Timeline

| Lesson | Duration | Cumulative | Notes |
|--------|----------|-----------|-------|
| I1: Nodes, Topics, Services, Actions | 1-2 hours | 1-2 hours | Your first Python nodes |
| I2: Python ROS Bridge (rclpy) | 1-2 hours | 2-4 hours | Professional patterns |
| **Intermediate Total** | **2-4 hours** | **4-8 hours (cumulative with Beginner)** | Pure hands-on coding |

---

## Code Examples in This Tier

All working code examples are in the `code/intermediate/` directory:

```
code/intermediate/
├── minimal_publisher.py          # Basic publisher
├── minimal_subscriber.py          # Basic subscriber
├── simple_service.py              # Service server
├── service_client.py              # Service client
├── timer_callback_node.py         # Node with periodic tasks
├── parameters_example.py          # Using node parameters
├── launch/
│   └── talker_listener_launch.py  # Launch two nodes together
└── README.md                       # Setup and usage instructions
```

All code is **production-ready** and includes:
- Error handling
- Proper logging
- Documentation
- Type hints

---

## Hands-On Exercises

At the end of this tier, you'll complete:

- **Exercise I1**: Write a publisher node from scratch
- **Exercise I2**: Write a subscriber node that processes data
- **Exercise I3**: Implement a service server (e.g., math operations)
- **Exercise I4**: Create a launch file that starts multiple nodes
- **Exercise I5**: Debug a broken ROS 2 system using CLI tools
- **Checkpoint Project**: Create a two-node system (publisher ↔ subscriber) that communicates

All exercises are in [Intermediate Exercises](./exercises/intermediate-exercises.md).

---

## AI-Assisted Learning

Stuck on coding? Use these AI prompts:

- **Syntax Help**: "Write a ROS 2 publisher node in Python that publishes to /chatter"
- **Debugging**: "My subscriber doesn't receive messages. What could be wrong?"
- **Architecture**: "Should I use a service or a topic for this use case?"
- **Patterns**: "Show me the pattern for a node with multiple publishers"

See [Intermediate AI Prompts](../ai-prompts/intermediate-prompts.md) for a full library.

---

## Best Practices in This Tier

### Node Design
- One responsibility per node (modularity)
- Clear naming conventions (`node_name`, `topic_name`, `service_name`)
- Proper error handling and logging
- Documentation in docstrings

### Code Quality
- Type hints on functions
- Constants defined at module level
- Proper resource cleanup (destroying nodes)
- Testing with simple test scripts

### ROS 2 Patterns
- Use appropriate QoS profiles
- Avoid blocking operations in callbacks
- Use callbacks instead of busy-wait loops
- Document expected message/service contracts

---

## What You WILL Do in This Tier

- ✅ Write Python code
- ✅ Create multiple nodes
- ✅ Test pub/sub communication
- ✅ Build services
- ✅ Use launch files
- ✅ Debug real issues
- ✅ Understand rclpy internals

---

## What You Won't Do (Yet)

- URDF models (Advanced)
- Action servers with feedback (Advanced)
- Advanced state machines (Advanced)
- Real-world robot deployment (beyond this chapter)

---

## Debugging & Monitoring Tools

Learn to use ROS 2 CLI tools:

```bash
# View all topics
ros2 topic list

# Echo (print) messages from a topic
ros2 topic echo /topic_name

# Show topic information
ros2 topic info /topic_name

# List all nodes
ros2 node list

# Monitor node information
ros2 node info /node_name

# View service calls
ros2 service call /service_name Service_Type arguments
```

All tools are covered in the lessons with examples.

---

## What's Next?

After completing this tier:

1. **Consolidate** your Python ROS 2 knowledge
2. **Complete** all exercises and checkpoint project
3. **Experiment** by modifying example code
4. **Review** AI prompts for clarification
5. **Move Forward** to **Advanced Tier** where you'll handle complex patterns

The Advanced tier assumes you can write working ROS 2 Python nodes. There, you'll learn URDF, action servers, and integration patterns.

---

## Resources

- **rclpy API Documentation**: https://docs.ros.org/en/humble/Concepts/Intermediate/About-ROS-2-Client-Libraries.html
- **ROS 2 Tutorials**: https://docs.ros.org/en/humble/Tutorials.html
- **ROS 2 Design**: https://design.ros2.org/
- **Python Code Examples**: https://github.com/ros2/examples/tree/humble/rclpy
- **Debugging ROS 2**: https://docs.ros.org/en/humble/Tutorials/Advanced/ROS2-with-IDE.html

---

## Ready to Start Coding?

Begin with **[Lesson I1: Nodes, Topics, Services, and Actions](./01-nodes-topics.md)**.

---

*"Code is poetry. ROS 2 code is poetry that moves robots."*
