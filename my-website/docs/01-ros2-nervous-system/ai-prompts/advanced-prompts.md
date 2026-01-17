---
id: advanced_ai_prompts
title: "Advanced Tier AI Learning Prompts"
sidebar_position: 10
tier: advanced
chapter: chapter_1_ros2
---

# AI-Assisted Learning Prompts: Advanced Tier

**Chapter**: Chapter 1 - The Robotic Nervous System (ROS 2)
**Tier**: Advanced
**Purpose**: RAG-compatible prompts for AI-assisted learning

---

## Prompt Categories

### Conceptual (Understanding Core Ideas)
### Debugging (Solving Problems)
### Extension (Going Beyond the Basics)
### Real-World (Practical Applications)

---

## URDF Prompts (Lesson A1)

### Conceptual

#### Prompt: URDF vs XACRO
```yaml
id: urdf_vs_xacro
category: conceptual
prompt: "What's the difference between URDF and XACRO? When should I use each format for robot description?"
context: "Reader has created basic URDF files and wants to understand macro-based robot descriptions"
expected_topics:
  - URDF is pure XML, XACRO adds macros and variables
  - XACRO reduces repetition for symmetric robots
  - XACRO requires preprocessing before use
  - When to choose each format
tags:
  - "#advanced"
  - "#urdf"
  - "#xacro"
  - "#robot-description"
```

#### Prompt: Inertia Calculation
```yaml
id: inertia_calculation
category: conceptual
prompt: "How do I calculate the inertia values for my robot links? What happens if they're wrong?"
context: "Reader is adding inertial properties to URDF for simulation"
expected_topics:
  - Moment of inertia tensor (ixx, iyy, izz, ixy, ixz, iyz)
  - Formulas for common shapes (box, cylinder, sphere)
  - Effect of incorrect inertia on simulation
  - Tools for calculating inertia from CAD models
tags:
  - "#advanced"
  - "#urdf"
  - "#simulation"
  - "#physics"
```

#### Prompt: Collision vs Visual Geometry
```yaml
id: collision_vs_visual
category: conceptual
prompt: "Why do I need separate collision and visual geometry in URDF? Can't I just use the same geometry for both?"
context: "Reader is building URDF and confused about geometry duplication"
expected_topics:
  - Visual geometry is for rendering (can be complex)
  - Collision geometry is for physics (should be simple)
  - Performance implications of complex collision meshes
  - Best practices for collision approximation
tags:
  - "#advanced"
  - "#urdf"
  - "#collision"
  - "#optimization"
```

### Debugging

#### Prompt: URDF Not Displaying
```yaml
id: urdf_not_displaying
category: debugging
prompt: "My robot model appears broken or disconnected in RViz. Some links are floating in space. What should I check?"
context: "Reader's URDF loads but displays incorrectly in RViz2"
expected_topics:
  - Check joint parent-child relationships
  - Verify origin transformations are correct
  - Ensure all joints have valid parent links
  - Use check_urdf to find structural errors
  - Check for missing joint definitions
tags:
  - "#advanced"
  - "#urdf"
  - "#rviz"
  - "#debugging"
```

#### Prompt: TF Frame Errors
```yaml
id: tf_frame_errors
category: debugging
prompt: "RViz says 'No transform from base_link to world' or shows TF warnings. How do I fix frame transformation issues?"
context: "Reader is trying to visualize URDF but getting transform errors"
expected_topics:
  - robot_state_publisher role in TF tree
  - Fixed frame setting in RViz
  - Static transforms and world frame
  - Checking TF tree with ros2 run tf2_tools view_frames
tags:
  - "#advanced"
  - "#urdf"
  - "#tf"
  - "#debugging"
```

### Extension

#### Prompt: Adding Sensors to URDF
```yaml
id: adding_sensors_urdf
category: extension
prompt: "How do I add sensors like cameras and LIDAR to my robot URDF? Do I need special plugins?"
context: "Reader wants to extend URDF with sensor models for simulation"
expected_topics:
  - Sensor links as fixed children
  - Gazebo plugins for sensor simulation
  - Camera and LIDAR reference frames
  - Sensor-specific URDF tags
tags:
  - "#advanced"
  - "#urdf"
  - "#sensors"
  - "#gazebo"
```

#### Prompt: URDF for Mobile Robots
```yaml
id: urdf_mobile_robots
category: extension
prompt: "Can I use URDF to describe a wheeled robot instead of a humanoid? What's different about mobile robot URDFs?"
context: "Reader wants to apply URDF knowledge to different robot types"
expected_topics:
  - Continuous joints for wheels
  - Differential drive configuration
  - Caster wheels and fixed joints
  - Base footprint vs base_link conventions
tags:
  - "#advanced"
  - "#urdf"
  - "#mobile-robots"
  - "#wheels"
```

---

## Action Prompts (Lesson A2)

### Conceptual

#### Prompt: Action vs Service
```yaml
id: action_vs_service
category: conceptual
prompt: "When should I use an action instead of a service? What are the tradeoffs between them?"
context: "Reader is deciding between action and service for a new feature"
expected_topics:
  - Services are synchronous, actions are asynchronous
  - Actions provide feedback during execution
  - Actions support cancellation
  - Service for quick operations, action for long-running tasks
  - Resource and complexity considerations
tags:
  - "#advanced"
  - "#actions"
  - "#services"
  - "#architecture"
```

#### Prompt: Concurrent Goals
```yaml
id: concurrent_goals
category: conceptual
prompt: "How do I handle multiple concurrent action goals in my robot system? Should I allow them or queue them?"
context: "Reader is designing action server that may receive multiple goals"
expected_topics:
  - Goal policies (reject, queue, preempt)
  - MultiThreadedExecutor and callback groups
  - Resource management with concurrent goals
  - Goal ID tracking
tags:
  - "#advanced"
  - "#actions"
  - "#concurrency"
  - "#architecture"
```

#### Prompt: Action Feedback Design
```yaml
id: action_feedback_design
category: conceptual
prompt: "How should I design my action feedback messages? What information should I include?"
context: "Reader is creating custom action definitions"
expected_topics:
  - Progress percentage for UI display
  - Current state for debugging
  - Estimated time remaining
  - Enough info for client decisions
  - Balance between detail and bandwidth
tags:
  - "#advanced"
  - "#actions"
  - "#message-design"
  - "#api-design"
```

### Debugging

#### Prompt: Action Server Not Completing
```yaml
id: action_not_completing
category: debugging
prompt: "My action server accepts goals but never completes. The execute_callback seems to run but the result never arrives. What should I check?"
context: "Reader's action server is stuck in execution"
expected_topics:
  - Ensure goal_handle.succeed() or .abort() is called
  - Check for exceptions in execute_callback
  - Verify result is returned from callback
  - Check executor type (single vs multi-threaded)
  - Look for blocking code without yielding
tags:
  - "#advanced"
  - "#actions"
  - "#debugging"
  - "#callbacks"
```

#### Prompt: Action Client Timeout
```yaml
id: action_client_timeout
category: debugging
prompt: "The action client times out waiting for the server even though the server is running. How do I debug action connectivity?"
context: "Reader's action client can't connect to server"
expected_topics:
  - Check action name matches exactly
  - Verify namespace if using
  - Use ros2 action list to confirm server presence
  - Check DDS configuration for multi-machine setup
  - Examine QoS compatibility
tags:
  - "#advanced"
  - "#actions"
  - "#debugging"
  - "#networking"
```

### Extension

#### Prompt: Preemptive Action Server
```yaml
id: preemptive_action_server
category: extension
prompt: "How would I implement a preemptive action server that cancels the current goal when a new one arrives?"
context: "Reader wants to implement goal preemption"
expected_topics:
  - Handle_accepted callback for preemption
  - Cancel current goal before accepting new
  - Clean transition between goals
  - State preservation if needed
tags:
  - "#advanced"
  - "#actions"
  - "#preemption"
  - "#state-management"
```

#### Prompt: Behavior Trees with Actions
```yaml
id: behavior_trees_actions
category: extension
prompt: "Can I chain multiple actions together to create a behavior tree? How do behavior trees integrate with ROS 2 actions?"
context: "Reader wants to create complex robot behaviors"
expected_topics:
  - BehaviorTree.CPP library
  - Action nodes in behavior trees
  - Parallel and sequential execution
  - Fallback and recovery behaviors
  - Integration patterns
tags:
  - "#advanced"
  - "#actions"
  - "#behavior-trees"
  - "#planning"
```

---

## AI Integration Prompts

### Conceptual

#### Prompt: AI Agent ROS Integration
```yaml
id: ai_agent_ros_integration
category: conceptual
prompt: "How can an AI agent (like an LLM) safely control a robot through ROS 2? What patterns should I use?"
context: "Reader is exploring AI-robot integration"
expected_topics:
  - Actions for high-level commands
  - Topics for perception
  - Safety layer between AI and actuators
  - Feedback loop for AI decisions
  - Human override considerations
tags:
  - "#advanced"
  - "#ai"
  - "#integration"
  - "#safety"
```

#### Prompt: LLM Robot Control
```yaml
id: llm_robot_control
category: conceptual
prompt: "What are the challenges of using LLMs to control robots? How do I handle uncertainty and errors?"
context: "Reader wants to build LLM-controlled robot systems"
expected_topics:
  - LLM output parsing challenges
  - Uncertainty in natural language commands
  - Error handling and recovery
  - Safety constraints enforcement
  - Feedback and confirmation patterns
tags:
  - "#advanced"
  - "#ai"
  - "#llm"
  - "#safety"
```

### Real-World

#### Prompt: Industrial Action Patterns
```yaml
id: industrial_action_patterns
category: real-world
prompt: "How are actions used in real industrial robots? What patterns are common in production systems?"
context: "Reader wants to understand professional robotics practices"
expected_topics:
  - Pick and place operations
  - Navigation and mapping
  - Inspection and quality control
  - Error recovery strategies
  - Logging and monitoring requirements
tags:
  - "#advanced"
  - "#actions"
  - "#industrial"
  - "#production"
```

#### Prompt: Humanoid Robot Actions
```yaml
id: humanoid_robot_actions
category: real-world
prompt: "What kinds of actions does a humanoid robot typically need? How do you design actions for walking, grasping, and manipulation?"
context: "Reader is designing action interfaces for humanoid robots"
expected_topics:
  - Locomotion actions (walk, turn, balance)
  - Manipulation actions (grasp, place, pour)
  - Coordination between actions
  - Safety during transitions
  - Human-robot interaction patterns
tags:
  - "#advanced"
  - "#actions"
  - "#humanoid"
  - "#manipulation"
```

---

## Usage Guidelines

### For Students

1. **Start with conceptual prompts** to understand the "why"
2. **Move to debugging prompts** when you encounter issues
3. **Explore extension prompts** after mastering basics
4. **Try real-world prompts** to connect theory to practice

### For AI Assistants (RAG Integration)

When answering these prompts:
1. Reference specific code examples from the chapter
2. Suggest hands-on exercises to reinforce learning
3. Point to official ROS 2 documentation for deeper dives
4. Acknowledge the reader's current skill level

### Prompt Template

```yaml
id: unique_identifier
category: conceptual | debugging | extension | real-world
prompt: "The question the reader might ask"
context: "Background on what the reader is trying to do"
expected_topics:
  - Topic 1 the response should cover
  - Topic 2 the response should cover
  - Topic 3 the response should cover
tags:
  - "#tier"
  - "#topic1"
  - "#topic2"
```

---

## Index by Tag

### #actions
- action_vs_service
- concurrent_goals
- action_feedback_design
- action_not_completing
- action_client_timeout
- preemptive_action_server
- behavior_trees_actions
- industrial_action_patterns
- humanoid_robot_actions

### #urdf
- urdf_vs_xacro
- inertia_calculation
- collision_vs_visual
- urdf_not_displaying
- tf_frame_errors
- adding_sensors_urdf
- urdf_mobile_robots

### #ai
- ai_agent_ros_integration
- llm_robot_control

### #debugging
- urdf_not_displaying
- tf_frame_errors
- action_not_completing
- action_client_timeout
