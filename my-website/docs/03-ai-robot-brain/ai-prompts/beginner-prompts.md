---
id: chapter_3_beginner_prompts
title: "Beginner AI Prompts"
sidebar_position: 43
chapter: chapter_3_ai_brain
tier: beginner
---

# Beginner AI Prompts for Chapter 3

Use these prompts with AI assistants (like Claude, ChatGPT, or your embedded RAG chatbot) to get help understanding perception, SLAM, and navigation concepts.

---

## How to Use These Prompts

1. **Copy the prompt** that matches your question
2. **Paste it** into your AI assistant
3. **Add context** if needed (e.g., "I'm working on Lesson B2")
4. **Ask follow-ups** to dive deeper

---

## Understanding Perception

### What is Perception?

```
Explain robotic perception as if I'm completely new to robotics.
What problem does it solve? Use simple analogies.
```

### Perception Pipeline

```
Break down the perception pipeline (Sensing → Preprocessing → Feature Extraction → Interpretation)
with a concrete example of a robot detecting a door.
```

### Why is Perception Hard?

```
What makes robotic perception challenging? Explain sensor noise,
real-time constraints, and environmental variations in simple terms.
```

### Perception vs. Computer Vision

```
What's the difference between computer vision and robotic perception?
When would I use each term?
```

---

## Understanding Sensors

### Choosing Sensors

```
I'm building a mobile robot. How do I decide between RGB cameras,
depth cameras, and LIDAR? What are the trade-offs?
```

### RGB Cameras

```
Explain how RGB cameras work in robotics. What can they detect?
What are their limitations? Give examples of use cases.
```

### Depth Cameras

```
What is a depth camera and how does it differ from a regular camera?
How do robots use depth information? Explain RGB-D data.
```

### LIDAR Basics

```
Explain LIDAR as if I've never heard of it. How does it work?
What does a LIDAR scan look like? Why is it used for navigation?
```

### IMU Sensors

```
What is an IMU and what does it measure? How do robots use IMU data
for balance and orientation? Explain in simple terms.
```

### Sensor Fusion

```
What is sensor fusion and why do robots need multiple sensors?
Give an example of combining camera and LIDAR data.
```

---

## Understanding SLAM

### SLAM Basics

```
Explain SLAM (Simultaneous Localization and Mapping) using a real-world analogy.
Why is it called "simultaneous"? What problem does it solve?
```

### Localization vs. Mapping

```
What's the difference between localization and mapping?
Why can't robots just do one or the other?
```

### Loop Closure

```
What is loop closure in SLAM? Why is it important?
Use an analogy to explain how it corrects drift.
```

### Occupancy Grids

```
What is an occupancy grid? How does it represent the environment?
Explain free space, occupied space, and unknown space.
```

### SLAM Challenges

```
What makes SLAM difficult? Explain the challenges of sensor noise,
dynamic environments, and computational constraints.
```

### SLAM in Practice

```
How do real robots use SLAM? Walk me through a robot building a map
of a room step by step.
```

---

## Understanding Navigation

### Autonomous Navigation

```
What does autonomous navigation mean? How is it different from
remote control? What capabilities does a robot need?
```

### Global vs. Local Planning

```
Explain the difference between global planning and local planning
in robot navigation. Why do we need both?
```

### Costmaps

```
What is a costmap in robot navigation? How does it represent
where the robot can safely go? Use simple terms.
```

### Path Planning

```
How do robots plan paths from point A to point B?
Explain the basic idea without complex math.
```

### Obstacle Avoidance

```
How do robots avoid obstacles while navigating?
What happens when a new obstacle appears in the path?
```

### Recovery Behaviors

```
What are recovery behaviors in navigation? Why do robots need them?
Give examples of when they're used.
```

### Nav2 Stack

```
What is Nav2? What components does it include?
Explain its role in ROS 2 navigation in simple terms.
```

---

## Visualization & Tools

### RViz2 for Perception

```
How do I use RViz2 to visualize sensor data?
What displays should I add to see camera images, depth data, and LIDAR scans?
```

### Understanding TF Frames

```
What are coordinate frames (TF) in robotics? Why do robots need multiple frames?
Explain map, odom, and base_link.
```

### Debugging Perception

```
My robot can't see obstacles. How do I debug perception issues?
What should I check first?
```

---

## Conceptual Questions

### Perception Pipeline Stages

```
Walk me through each stage of the perception pipeline with a specific example
(e.g., detecting a person in a camera image).
```

### SLAM vs. GPS

```
Why can't indoor robots just use GPS for localization?
What advantages does SLAM provide?
```

### Simulation vs. Real World

```
What's different between perception in simulation (Gazebo) and real robots?
What challenges appear in the real world?
```

### Sensor Data Formats

```
What do different sensor messages look like in ROS 2?
Explain Image, PointCloud2, and LaserScan message types.
```

---

## Troubleshooting

### Can't See Sensor Data

```
I launched my robot in Gazebo but don't see sensor data in RViz2.
What could be wrong? How do I troubleshoot?
```

### SLAM Not Working

```
I'm running SLAM Toolbox but the map looks wrong or doesn't update.
What are common issues and how do I fix them?
```

### Navigation Fails

```
My robot won't navigate to the goal. It just sits there or spins.
What should I check?
```

### Frame Errors

```
I'm getting TF frame errors. What does "frame does not exist" mean?
How do I fix frame relationship issues?
```

---

## Learning Strategies

### Study Approach

```
I'm new to AI robotics. What's the best order to learn perception, SLAM, and navigation?
Should I focus on theory first or jump into practice?
```

### Hands-On Practice

```
What hands-on activities can I do to understand perception better?
I have access to Gazebo simulation.
```

### Key Concepts to Master

```
What are the most important concepts in this chapter that I absolutely must understand
before moving to intermediate implementation?
```

---

## Analogies & Explanations

### Perception Analogy

```
Explain robotic perception using the analogy of human vision and understanding.
How is it similar? How is it different?
```

### SLAM Analogy

```
Give me a non-robotics analogy for SLAM. Maybe something about exploring
a dark building with a flashlight?
```

### Navigation Analogy

```
Explain robot navigation using the analogy of driving a car with GPS.
What parts are similar? What's different?
```

---

## Quick Reference

### Sensor Comparison

```
Create a simple comparison table of RGB cameras, depth cameras, and LIDAR.
Include: what they measure, range, use cases, and limitations.
```

### SLAM Components

```
List and briefly explain the main components of a SLAM system.
Keep it simple for a beginner.
```

### Navigation Pipeline

```
Describe the complete navigation pipeline from "I want to go there"
to "I arrived" in simple steps.
```

---

## Tips for Using These Prompts

- **Be Specific**: Add details about what you're working on
- **Ask Follow-ups**: Don't hesitate to ask "Can you explain that differently?"
- **Request Examples**: Ask for concrete examples if explanations are too abstract
- **Visual Aids**: Request diagrams or step-by-step breakdowns
- **Check Understanding**: Ask the AI to quiz you on concepts

---

**Remember**: There are no stupid questions. If you don't understand something, ask for clarification. AI assistants are here to help you learn at your own pace.

---

**Next**: [Intermediate AI Prompts](./intermediate-prompts.md) (after completing Beginner tier)
