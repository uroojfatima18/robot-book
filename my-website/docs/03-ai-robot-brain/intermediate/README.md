---
id: chapter_3_intermediate_tier
title: "Intermediate Tier: Hands-On Perception & Navigation"
sidebar_position: 30
tier: intermediate
chapter: chapter_3_ai_brain
estimated_time: "3-5 hours"
---

# Intermediate Tier: Hands-On Perception & Navigation

## Welcome to the Intermediate Tier

Now that you understand perception, SLAM, and navigation concepts, it's time to implement them. This tier teaches you *how* to build perception nodes, configure SLAM Toolbox, manage coordinate frames with TF2, and deploy Nav2 for autonomous navigation.

---

## Tier Overview

```
🟡 INTERMEDIATE TIER - Implementation & Practice
════════════════════════════════════════════════════

What You'll Learn:
• Processing camera and depth data with cv_bridge and OpenCV
• Managing coordinate frames using TF2
• Configuring and running SLAM Toolbox for mapping
• Launching Nav2 and sending navigation goals programmatically
• Debugging perception and navigation systems

What You'll Build:
• Perception nodes that process sensor data
• Working SLAM system that generates maps
• Autonomous navigation with Nav2
• Complete perception-to-action pipeline
```

---

## Learning Objectives

By the end of the Intermediate tier, you will be able to:

1. **Create** perception nodes that subscribe to camera topics
2. **Use** cv_bridge to convert between ROS and OpenCV formats
3. **Process** RGB and depth images to extract features and measurements
4. **Manage** coordinate frames using TF2 (broadcasting and lookup)
5. **Configure** SLAM Toolbox for 2D mapping
6. **Generate** maps from sensor data in simulation
7. **Launch** the Nav2 stack with proper configuration
8. **Send** navigation goals programmatically from Python
9. **Monitor** navigation status and handle feedback
10. **Debug** perception and navigation issues using ROS 2 tools

---

## Prerequisites

Before starting this tier, you must have completed:

- **The Beginner Tier** ✅
  - You understand perception pipelines conceptually
  - You know sensor types and their uses
  - You understand SLAM and navigation architecture

- **Python Programming Skills**:
  - Classes and object-oriented programming
  - NumPy for array operations
  - Basic OpenCV (or willingness to learn)

- **ROS 2 Proficiency**:
  - Creating nodes, publishers, and subscribers
  - Launch files and parameters
  - Using ROS 2 CLI tools

**Important**: If you haven't completed the Beginner tier, go back now. This tier assumes solid conceptual understanding.

---

## Lessons in This Tier

### Lesson I1: Camera and Depth Data Processing
**Duration**: 60-90 minutes

Build your first perception pipeline. Learn to subscribe to camera topics, convert ROS messages to OpenCV format, process images, and publish results.

**Key Topics**:
- Subscribing to Image topics
- cv_bridge: ROS ↔ OpenCV conversion
- Processing RGB images (edge detection, filtering)
- Processing depth images (distance extraction)
- Publishing processed data
- Perception node architecture

**Hands-On Activities**:
- Create a camera subscriber node
- Convert and display images using OpenCV
- Extract depth measurements from specific regions
- Implement edge detection pipeline
- Publish processed images to new topics

**Outcomes**:
- ✅ Working perception node
- ✅ Understanding of cv_bridge
- ✅ Image processing skills
- ✅ Depth data extraction

**File**: [I1: Camera and Depth Data Processing](./I1-camera-depth-processing.md)

---

### Lesson I2: TF2 Coordinate Frame Management
**Duration**: 60-90 minutes

Master coordinate frame management with TF2. Learn to broadcast transforms, look up frame relationships, and handle time synchronization for multi-sensor systems.

**Key Topics**:
- Understanding frame relationships (map → odom → base_link → sensors)
- Broadcasting static and dynamic transforms
- Looking up transforms between frames
- Time synchronization and latency handling
- Visualizing TF trees
- Common frame conventions

**Hands-On Activities**:
- Broadcast a static transform for a sensor
- Create a dynamic transform broadcaster
- Look up transforms in code
- Visualize the TF tree with view_frames
- Transform points between frames

**Outcomes**:
- ✅ Understanding of TF2 architecture
- ✅ Broadcasting transforms
- ✅ Looking up frame relationships
- ✅ Debugging TF issues

**File**: [I2: TF2 Coordinate Frame Management](./I2-tf2-coordinate-frames.md)

---

### Lesson I3: SLAM Toolbox Configuration
**Duration**: 60-90 minutes

Configure and run SLAM Toolbox to generate maps from laser scan data. Learn online and offline SLAM modes, parameter tuning, and map saving/loading.

**Key Topics**:
- SLAM Toolbox architecture
- Online vs. offline SLAM
- Configuration parameters (resolution, range, loop closure)
- Running SLAM in simulation
- Saving and loading maps
- Map quality assessment
- Integration with navigation

**Hands-On Activities**:
- Launch SLAM Toolbox with default config
- Drive robot to build a map
- Tune SLAM parameters for your environment
- Save map to file
- Load existing map for localization
- Visualize SLAM process in RViz2

**Outcomes**:
- ✅ Working SLAM system
- ✅ Generated maps
- ✅ Parameter tuning skills
- ✅ Map management

**File**: [I3: SLAM Toolbox Configuration](./I3-slam-toolbox.md)

---

### Lesson I4: Nav2 Basics
**Duration**: 60-90 minutes

Launch the Nav2 navigation stack and send goals programmatically. Learn the navigation pipeline, goal sending, status monitoring, and basic troubleshooting.

**Key Topics**:
- Nav2 architecture and components
- Launching Nav2 with configuration
- Navigation goals and poses
- Action client for NavigateToPose
- Monitoring navigation status
- Handling navigation feedback
- Basic costmap visualization
- Recovery behaviors

**Hands-On Activities**:
- Launch Nav2 with a pre-built map
- Send navigation goals via RViz2
- Write Python code to send goals programmatically
- Monitor navigation status and feedback
- Visualize costmaps in RViz2
- Handle navigation failures

**Outcomes**:
- ✅ Working Nav2 system
- ✅ Programmatic goal sending
- ✅ Status monitoring
- ✅ Basic troubleshooting

**File**: [I4: Nav2 Basics](./I4-nav2-basics.md)

---

## Progression & Scaffolding

The Intermediate tier builds on Beginner foundations and prepares for Advanced tuning:

```
Beginner (Concept)          Intermediate (Implementation)    Advanced (Optimization)
└─ Perception pipeline      └─ Build perception nodes        └─ Advanced processing
└─ Sensor types             └─ Process camera/depth data     └─ Sensor fusion
└─ SLAM concepts            └─ Run SLAM Toolbox              └─ SLAM tuning
└─ Navigation architecture  └─ Deploy Nav2                   └─ Costmap config
                            └─ Send goals programmatically   └─ Planner tuning
```

---

## Estimated Timeline

| Lesson | Duration | Cumulative | Notes |
|--------|----------|-----------|-------|
| I1: Camera and Depth Processing | 60-90 min | 60-90 min | First perception node |
| I2: TF2 Coordinate Frames | 60-90 min | 2-3 hours | Frame management |
| I3: SLAM Toolbox | 60-90 min | 3-4.5 hours | Map generation |
| I4: Nav2 Basics | 60-90 min | 4-6 hours | Autonomous navigation |
| **Intermediate Total** | **4-6 hours** | **6.5-9.5 hours (cumulative with Beginner)** | Hands-on implementation |

---

## Code Examples in This Tier

All working code examples are in the respective lesson directories:

```
intermediate/
├── code/
│   ├── camera_subscriber.py          # Basic camera processing
│   ├── depth_processor.py             # Depth data extraction
│   ├── perception_node.py             # Complete perception pipeline
│   ├── tf2_broadcaster.py             # Transform broadcasting
│   ├── tf2_listener.py                # Transform lookup
│   └── nav2_goal_sender.py            # Navigation goal client
├── launch/
│   ├── perception_launch.py           # Launch perception nodes
│   ├── slam_launch.py                 # SLAM configuration
│   └── navigation_launch.py           # Nav2 launch
└── config/
    ├── slam_params.yaml               # SLAM Toolbox parameters
    └── nav2_params.yaml               # Nav2 configuration
```

All code is **production-ready** and includes:
- Error handling
- Proper logging
- Documentation
- Type hints

---

## Hands-On Exercises

At the end of this tier, you'll complete:

- **Exercise I1**: Build a perception node that detects edges in camera images
- **Exercise I2**: Create a depth-based obstacle detector
- **Exercise I3**: Broadcast transforms for a custom sensor
- **Exercise I4**: Generate a complete map of a simulated environment
- **Exercise I5**: Send navigation goals to multiple waypoints
- **Checkpoint Project**: Build a complete perception-to-navigation pipeline

All exercises are in [Intermediate Exercises](../exercises/intermediate-exercises.md).

---

## AI-Assisted Learning

Stuck on implementation? Use these AI prompts:

- **Code Help**: "Write a ROS 2 node that subscribes to /camera/image and publishes edge-detected images"
- **Debugging**: "My cv_bridge conversion fails with encoding error. How do I fix it?"
- **Configuration**: "What SLAM Toolbox parameters should I tune for a large environment?"
- **Architecture**: "How do I structure a perception node with multiple subscribers?"
- **TF2 Issues**: "My TF lookup fails with 'frame does not exist'. What's wrong?"

See [Intermediate AI Prompts](../ai-prompts/intermediate-prompts.md) for a full library.

---

## Best Practices in This Tier

### Perception Nodes
- Use QoS profiles appropriate for sensor data
- Handle encoding conversions properly
- Add error checking for invalid images
- Log processing times for performance monitoring

### TF2 Management
- Follow standard frame naming conventions
- Use static transforms for fixed sensors
- Handle transform lookup exceptions
- Visualize TF tree regularly

### SLAM Configuration
- Start with default parameters
- Tune incrementally based on results
- Save maps frequently during exploration
- Monitor loop closure events

### Navigation
- Verify map quality before navigation
- Set appropriate goal tolerances
- Handle navigation failures gracefully
- Monitor costmaps for issues

---

## What You WILL Do in This Tier

- ✅ Write perception code
- ✅ Process sensor data
- ✅ Manage coordinate frames
- ✅ Generate maps with SLAM
- ✅ Deploy autonomous navigation
- ✅ Send goals programmatically
- ✅ Debug real issues

---

## What You Won't Do (Yet)

- Advanced costmap tuning (Advanced)
- Custom planner configuration (Advanced)
- Reinforcement learning (Advanced)
- Sim-to-real deployment (Advanced)

---

## Debugging & Monitoring Tools

Learn to use these ROS 2 tools:

```bash
# View all image topics
ros2 topic list | grep image

# Echo image info
ros2 topic echo /camera/image --no-arr

# View TF tree
ros2 run tf2_tools view_frames

# Check TF relationships
ros2 run tf2_ros tf2_echo map base_link

# Monitor Nav2 status
ros2 topic echo /navigate_to_pose/_action/status

# Visualize costmaps
ros2 run rviz2 rviz2
```

All tools are covered in the lessons with examples.

---

## What's Next?

After completing this tier:

1. **Consolidate** your implementation skills
2. **Complete** all exercises and checkpoint project
3. **Experiment** by modifying parameters and code
4. **Review** AI prompts for clarification
5. **Move Forward** to **Advanced Tier** for optimization and RL

The Advanced tier assumes you can implement working perception and navigation systems. There, you'll learn advanced configuration, RL fundamentals, and sim-to-real transfer.

---

## Resources

- **cv_bridge Tutorial**: https://docs.ros.org/en/humble/Tutorials/Intermediate/CvBridge.html
- **TF2 Tutorials**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2.html
- **SLAM Toolbox Documentation**: https://github.com/SteveMacenski/slam_toolbox
- **Nav2 Getting Started**: https://navigation.ros.org/getting_started/
- **OpenCV Python Tutorials**: https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html

---

## Ready to Start Implementing?

Begin with **[Lesson I1: Camera and Depth Data Processing](./I1-camera-depth-processing.md)**.

---

*"Code is the bridge between understanding and capability. Let's build that bridge."*
