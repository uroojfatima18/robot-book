---
id: beginner_ai_prompts
title: "Beginner Tier AI Learning Prompts"
tier: beginner
chapter: chapter_1_ros2
---

# AI-Assisted Learning Prompts: Beginner Tier

This document provides curated prompts for AI assistants (Claude, ChatGPT, etc.) to help reinforce learning from the Beginner tier lessons. These prompts are designed to deepen understanding through explanation, debugging help, and exploration.

---

## How to Use These Prompts

1. **Copy** a prompt that matches your learning goal
2. **Paste** it into your preferred AI assistant
3. **Follow up** with specific questions about your situation
4. **Verify** AI responses against ROS 2 documentation when learning new concepts

> **Tip**: AI assistants work best when you provide context. Include error messages, code snippets, or describe your setup when asking debugging questions.

---

## B1: Introduction to ROS 2 Prompts

### Conceptual Understanding

#### What is ROS 2?
```
I'm learning ROS 2 for humanoid robotics. Can you explain:
1. Why ROS 2 is called a "robot operating system" even though it's not an OS?
2. What problems does ROS 2 solve that I couldn't solve with regular Python/C++?
3. Give me a simple analogy comparing ROS 2 to something from everyday life.
```

#### Topics vs Services vs Actions
```
I'm confused about when to use Topics, Services, and Actions in ROS 2. Can you:
1. Explain each one with a humanoid robot example
2. Create a comparison table showing: communication style, when to use, example use case
3. Give me a rule of thumb for choosing between them
```

#### ROS 2 vs ROS 1
```
My robotics course mentions both ROS 1 and ROS 2. I'm starting fresh with ROS 2.
Can you explain the key differences between them, especially:
1. Why was ROS 2 created?
2. What can I do in ROS 2 that I couldn't in ROS 1?
3. Should I ever learn ROS 1, or is it obsolete?
```

### Debugging Help

#### Installation Issues
```
I'm trying to install ROS 2 Humble on Ubuntu 22.04 but getting this error:
[PASTE YOUR ERROR HERE]

My system:
- Ubuntu version: [YOUR VERSION]
- I followed these steps: [LIST STEPS]

Can you help me diagnose and fix this?
```

#### Command Not Found
```
When I run `ros2 --version` I get "command not found" even though I installed ROS 2.
I'm on Ubuntu 22.04 with ROS 2 Humble.
What are the most likely causes and how do I fix them?
```

#### Talker/Listener Not Communicating
```
I'm running the ROS 2 talker/listener demo but the listener isn't receiving messages.
- Talker shows: "Publishing: 'Hello World: X'"
- Listener shows: nothing
- `ros2 topic list` shows: /chatter
- Both are in separate terminals, both sourced ROS 2

What could be wrong and how do I debug this?
```

### Extension and Exploration

#### Modifying the Demo
```
I have the ROS 2 talker/listener demo working. Now I want to modify it to:
1. Publish a different message type (not String)
2. Include a timestamp in each message
3. Publish at a variable rate based on some condition

Can you guide me through one of these modifications step by step?
```

#### Multi-Machine ROS 2
```
I want to run ROS 2 nodes on different computers (like a robot and a laptop).
Can you explain:
1. What network configuration is needed?
2. How does ROS 2 discovery work?
3. What's the simplest way to test this with the talker/listener demo?
```

---

## B2: Basic Sensors Overview Prompts

### Conceptual Understanding

#### IMU Deep Dive
```
I'm learning about IMU sensors for humanoid robots. Can you explain:
1. How does an accelerometer actually measure acceleration?
2. Why does the Z-axis show ~9.8 m/s² when the robot is stationary?
3. What's "sensor drift" and why do IMUs suffer from it?
4. How is IMU data used for balance control in humanoids?
```

#### LIDAR Explained
```
I'm trying to understand LIDAR for robot navigation. Can you explain:
1. How does LIDAR measure distance using light?
2. What's the difference between 2D and 3D LIDAR?
3. Why is LIDAR better than cameras for obstacle avoidance?
4. What are LIDAR's limitations (when does it fail)?
```

#### Camera Types
```
My robot has both an RGB camera and a depth camera. Can you explain:
1. How does a depth camera measure distance to each pixel?
2. What's the difference between stereo depth and time-of-flight?
3. When would I use RGB vs depth vs both together?
4. What resolution and frame rate should I expect?
```

#### Force Sensors
```
I'm confused about force/torque sensors on humanoid hands. Can you explain:
1. What's the difference between a force sensor and a torque sensor?
2. How does a robot use force feedback to grasp objects gently?
3. What are the units and typical ranges for hand F/T sensors?
4. How would the robot detect that it successfully picked up an object?
```

### Debugging Help

#### IMU Data Issues
```
My IMU is returning strange values:
- Orientation quaternion: (0, 0, 0, 0) - shouldn't w be 1?
- Linear acceleration z: 0.0 instead of ~9.8
- The robot is stationary on a flat surface

What could cause this and how do I diagnose?
```

#### LIDAR Range Problems
```
My LIDAR scan has issues:
- Many readings show 'inf' or 'nan'
- Some readings show 0.0
- The scan doesn't form a coherent picture of the room

The sensor is a [SENSOR MODEL] connected via [USB/ETHERNET].
What should I check?
```

#### Camera Calibration
```
My depth camera's distance measurements seem inaccurate:
- Objects at 1 meter show as 0.8-1.2 meters (20% error)
- The RGB and depth images don't align perfectly

Is this a calibration issue? How do I recalibrate a depth camera in ROS 2?
```

### Extension and Exploration

#### Sensor Fusion Basics
```
I've read that robots "fuse" sensor data. Can you explain:
1. What is sensor fusion and why is it needed?
2. How would I combine IMU and camera data for better pose estimation?
3. What's a Kalman filter and when would I use one?
4. Are there ROS 2 packages that do sensor fusion automatically?
```

#### Choosing Sensors
```
I'm designing a humanoid robot and need to choose sensors. Can you help me:
1. Compare different depth camera options (RealSense vs Azure Kinect vs ZED)
2. Explain what specs matter most (range, accuracy, frame rate, etc.)
3. Suggest a good beginner-friendly sensor setup
4. Estimate a budget for a basic sensor suite
```

#### SLAM Overview
```
I keep seeing "SLAM" mentioned with LIDAR. Can you explain:
1. What is SLAM and why do robots need it?
2. How does LIDAR enable SLAM?
3. Can cameras do SLAM too? How is it different?
4. What ROS 2 packages implement SLAM?
```

---

## Practice Scenarios

### Scenario 1: Debug This Robot
```
A humanoid robot is having balance problems. Here's what we know:
- IMU readings look normal
- The robot falls backward when trying to walk
- Force sensors in feet show uneven weight distribution

As a debugging exercise, can you:
1. List 5 possible causes for this problem
2. Suggest specific sensor readings to check
3. Propose a systematic debugging approach
```

### Scenario 2: Design a Sensor System
```
I'm adding sensors to a simple wheeled robot that needs to:
- Navigate indoors without hitting obstacles
- Recognize specific objects (like a red ball)
- Know its position in a room

Can you recommend:
1. Which sensors I need (minimum viable)
2. Where to mount them
3. What ROS 2 message types I'll work with
4. Any gotchas or common mistakes to avoid
```

### Scenario 3: Interpret Sensor Data
```
Here's a snapshot of sensor readings from a humanoid robot:

IMU:
  orientation: (x=0.02, y=0.15, z=0.01, w=0.99)
  angular_velocity: (x=0.0, y=0.5, z=0.0)
  linear_acceleration: (x=0.3, y=0.0, z=9.6)

Right foot force sensor:
  force: (x=0, y=0, z=450)

Left foot force sensor:
  force: (x=0, y=0, z=50)

What can you tell me about what the robot is doing right now?
```

---

## Prompt Templates

### General Learning Template
```
I'm studying [TOPIC] in ROS 2 for humanoid robotics.

My current understanding: [WHAT YOU KNOW]

I'm confused about: [SPECIFIC QUESTION]

Can you explain this in a way that connects to [SOMETHING YOU UNDERSTAND]?
```

### Debugging Template
```
I'm getting an error/unexpected behavior in ROS 2.

Setup:
- ROS 2 version: [HUMBLE/IRON/JAZZY]
- OS: [UBUNTU VERSION]
- Hardware: [IF RELEVANT]

What I'm trying to do: [GOAL]

What I did: [STEPS]

What happened: [ERROR MESSAGE OR BEHAVIOR]

What I expected: [EXPECTED BEHAVIOR]

What I've already tried: [TROUBLESHOOTING STEPS]
```

### Exploration Template
```
I finished the [LESSON NAME] lesson and want to go deeper.

I'm particularly interested in: [TOPIC]

My skill level: [BEGINNER/INTERMEDIATE]

Time available: [HOURS]

Can you suggest:
1. A hands-on project to try
2. Additional reading/resources
3. What I should learn next
```

---

## Tips for Effective AI Learning

1. **Be Specific**: "My LIDAR shows inf values" is better than "my sensor doesn't work"

2. **Provide Context**: Include ROS 2 version, OS, and relevant code/errors

3. **Ask Follow-ups**: Don't accept the first answer—dig deeper

4. **Verify Information**: AI can be wrong; cross-reference with official docs

5. **Try First**: Attempt the problem yourself, then ask AI to review your approach

6. **Explain Back**: Ask AI to verify your understanding by explaining concepts back

---

## Next Steps

After mastering these beginner concepts, move to:
- [Intermediate AI Prompts](./intermediate-prompts.md) - Writing nodes, working with messages
- [Advanced AI Prompts](./advanced-prompts.md) - URDF, actions, complex systems
