# Intermediate Tier: Hands-On Perception and Navigation

**Duration**: 3-4 hours | **Prerequisite**: Beginner Tier Completion

---

## Overview

Welcome to the Intermediate tier! You've built a strong conceptual foundation. Now it's time to get hands-on with real ROS 2 tools and packages.

In this tier, you'll process camera and depth data, manage coordinate frames with TF2, run SLAM to generate maps, and use Nav2 to navigate autonomously. By the end, you'll have a complete perception and navigation system running in simulation.

---

## Learning Objectives

By the end of this tier, you will be able to:

1. **Process** camera and depth sensor data using cv_bridge and OpenCV
2. **Manage** coordinate frame transformations with TF2
3. **Generate** maps using SLAM Toolbox in simulation
4. **Configure** Nav2 for autonomous navigation
5. **Send** navigation goals programmatically
6. **Debug** perception and navigation issues
7. **Integrate** multiple components into a working system

---

## What You'll Learn

### Image Processing Skills
- Converting ROS 2 Image messages to OpenCV format
- Processing RGB and depth data
- Applying computer vision algorithms
- Publishing processed images back to ROS 2

### Coordinate Frame Management
- Understanding the TF2 transform tree
- Publishing and looking up transforms
- Managing multiple sensor frames
- Debugging TF2 issues

### SLAM Implementation
- Configuring SLAM Toolbox parameters
- Running SLAM in simulation
- Monitoring map building progress
- Saving and loading maps

### Navigation Implementation
- Configuring Nav2 for your robot
- Sending navigation goals via Python
- Monitoring navigation progress
- Handling navigation failures

---

## Lesson Structure

This tier contains **4 comprehensive lessons**:

### Lesson I1: Camera and Depth Data Processing
**File**: [I1-camera-depth-processing.md](./I1-camera-depth-processing.md)
**Duration**: 60-90 minutes

Master image processing in ROS 2:
- cv_bridge for image conversion
- Processing RGB camera data
- Working with depth images
- Synchronizing multiple image streams

**Includes**: Complete Python code examples and exercises.

### Lesson I2: TF2 Coordinate Frames
**File**: [I2-tf2-coordinate-frames.md](./I2-tf2-coordinate-frames.md)
**Duration**: 45-60 minutes

Learn coordinate frame management:
- Understanding the TF tree
- Publishing transforms
- Looking up transforms
- Transforming points between frames

**Includes**: TF2 visualization and debugging tools.

### Lesson I3: SLAM Toolbox Configuration
**File**: [I3-slam-toolbox.md](./I3-slam-toolbox.md)
**Duration**: 60-90 minutes

Generate maps with SLAM:
- Installing and configuring SLAM Toolbox
- Running SLAM in Gazebo
- Tuning SLAM parameters
- Saving and loading maps

**Includes**: Configuration files and launch examples.

### Lesson I4: Nav2 Basics
**File**: [I4-nav2-basics.md](./I4-nav2-basics.md)
**Duration**: 60-90 minutes

Implement autonomous navigation:
- Nav2 architecture and components
- Configuration for your robot
- Sending navigation goals
- Monitoring and debugging

**Includes**: Python navigation client and configuration examples.

---

## Prerequisites

### Knowledge Prerequisites
- **Beginner Tier Completion**: Strong conceptual understanding
- **Chapter 1 Advanced**: URDF, launch files, parameters
- **Chapter 2 Intermediate**: Gazebo world building, robot spawning
- **Python Proficiency**: Classes, functions, ROS 2 patterns

### Technical Prerequisites
- **ROS 2 Humble or Iron** installed and working
- **Nav2 packages**: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
- **SLAM Toolbox**: `sudo apt install ros-humble-slam-toolbox`
- **Vision packages**: `sudo apt install ros-humble-cv-bridge ros-humble-image-transport`
- **TF2 packages**: `sudo apt install ros-humble-tf2-ros ros-humble-tf2-tools`
- **Gazebo Classic** with a robot model from Chapter 2

---

## Time Commitment

| Activity | Time |
|----------|------|
| Lesson I1: Camera & Depth | 60-90 min |
| Lesson I2: TF2 Frames | 45-60 min |
| Lesson I3: SLAM Toolbox | 60-90 min |
| Lesson I4: Nav2 Basics | 60-90 min |
| Exercises | 45-60 min |
| **Total** | **3-5 hours** |

---

## Learning Path

```
Beginner Tier Complete
    ↓
I1: Camera and Depth Processing (60-90 min)
    ↓
I2: TF2 Coordinate Frames (45-60 min)
    ↓
I3: SLAM Toolbox (60-90 min)
    ↓
I4: Nav2 Basics (60-90 min)
    ↓
Intermediate Exercises (45-60 min)
    ↓
Ready for Advanced Tier!
```

---

## What You'll Build

By the end of this tier, you'll have:

1. **Image Processing Node**: Process camera and depth data in real-time
2. **TF2 System**: Properly configured coordinate frames
3. **SLAM System**: Generate maps of simulated environments
4. **Navigation System**: Autonomous navigation with Nav2
5. **Complete Integration**: All components working together

### Example Project: Autonomous Mobile Robot
You'll build a complete system where a robot:
- Perceives its environment through cameras and LIDAR
- Builds a map using SLAM
- Navigates autonomously to goals
- Avoids obstacles dynamically

---

## Success Criteria

You're ready to move to the Advanced tier when you can:

- [ ] Process camera images and convert them with cv_bridge
- [ ] Visualize and debug TF2 frame transformations
- [ ] Run SLAM Toolbox and generate a map
- [ ] Configure Nav2 for a robot
- [ ] Send navigation goals programmatically
- [ ] Debug common perception and navigation issues
- [ ] Integrate all components into a working system

---

## Development Environment Setup

### Workspace Organization
```bash
~/ros2_ws/
├── src/
│   ├── my_robot_perception/    # Your perception nodes
│   ├── my_robot_navigation/    # Your navigation config
│   └── my_robot_description/   # Robot URDF (from Ch 2)
├── build/
├── install/
└── log/
```

### Recommended Tools
- **VS Code** with ROS 2 extensions
- **RViz2** for visualization
- **rqt_graph** for node visualization
- **rqt_image_view** for image debugging
- **Gazebo** for simulation

---

## Common Challenges

### Challenge 1: cv_bridge Encoding Issues
**Symptoms**: Image conversion fails with encoding errors
**Solutions**:
- Verify image encoding matches cv_bridge conversion
- Check for mono8, bgr8, rgb8, 32FC1 (depth) encodings
- Use `sensor_msgs/Image` message inspection

### Challenge 2: TF Frame Not Found
**Symptoms**: "Frame X does not exist" errors
**Solutions**:
- Use `ros2 run tf2_tools view_frames` to visualize TF tree
- Verify all required transforms are being published
- Check frame names match exactly (case-sensitive)

### Challenge 3: SLAM Map Quality Poor
**Symptoms**: Blurry map, drift, inconsistencies
**Solutions**:
- Improve odometry quality
- Tune SLAM parameters (resolution, update rate)
- Ensure sufficient visual features in environment
- Check sensor data quality

### Challenge 4: Nav2 Won't Start
**Symptoms**: Lifecycle nodes fail to activate
**Solutions**:
- Verify all required topics are published
- Check parameter file syntax
- Ensure map is loaded correctly
- Review logs for specific errors

### Challenge 5: Robot Gets Stuck
**Symptoms**: Robot stops moving, won't reach goal
**Solutions**:
- Check costmap configuration
- Verify obstacle detection is working
- Tune planner parameters
- Configure recovery behaviors

---

## Development Workflow

### Iterative Development
1. **Start Simple**: Get each component working independently
2. **Test Incrementally**: Verify each addition before moving on
3. **Use Visualization**: RViz is essential for debugging
4. **Check Topics**: Use `ros2 topic echo` to verify data flow
5. **Read Logs**: Error messages usually point to the problem

### Debugging Strategy
1. **Verify Inputs**: Check that sensor data is being published
2. **Check Transforms**: Ensure TF tree is complete and correct
3. **Inspect Parameters**: Verify configuration is loaded
4. **Monitor Performance**: Watch CPU, memory, and message rates
5. **Isolate Issues**: Test components separately before integrating

---

## Code Examples

All lessons include production-ready code examples:
- **Fully commented**: Understand what each line does
- **Error handling**: Robust to common failures
- **Type hints**: Clear interfaces and expectations
- **ROS 2 best practices**: Follow official guidelines

### Example: Camera Subscriber Node
```python
# Simplified example - full version in I1 lesson
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Process image here
        cv2.imshow('Camera', cv_image)
        cv2.waitKey(1)
```

---

## Assets Provided

### Code Examples
- `code/camera_subscriber.py` - Camera image processing
- `code/depth_processor.py` - Depth data processing
- `code/tf2_broadcaster.py` - Transform publishing
- `code/nav2_goal_sender.py` - Navigation goal client

### Configuration Files
- `config/slam_params.yaml` - SLAM Toolbox configuration
- `config/nav2_params.yaml` - Nav2 configuration
- `launch/slam_launch.py` - SLAM launch file
- `launch/navigation_launch.py` - Nav2 launch file

### Diagrams
- TF tree examples
- SLAM process visualization
- Nav2 architecture diagram

---

## Tools and Resources

### ROS 2 Tools
- `ros2 topic list/echo/hz` - Topic inspection
- `ros2 node list/info` - Node inspection
- `ros2 param list/get/set` - Parameter management
- `ros2 run tf2_tools view_frames` - TF visualization
- `ros2 run tf2_ros tf2_echo` - Transform monitoring

### Visualization Tools
- **RViz2**: Primary visualization tool
- **rqt_image_view**: Image debugging
- **rqt_graph**: Node graph visualization
- **Gazebo**: Simulation environment

### External Resources
- [cv_bridge Tutorials](http://wiki.ros.org/cv_bridge/Tutorials)
- [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [SLAM Toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Documentation](https://navigation.ros.org/)

---

## Next Steps

After completing this tier:

1. **Complete the Intermediate Exercises**: Build a complete autonomous system
2. **Experiment**: Try different environments, sensors, parameters
3. **Move to Advanced Tier**: Learn costmap configuration, behavior trees, and RL
4. **Test on Real Hardware**: If available, deploy to a physical robot

---

## Ready to Code?

**Start with Lesson I1**: [Camera and Depth Data Processing](./I1-camera-depth-processing.md)

You'll learn to process visual data from cameras and depth sensors - the foundation of robotic perception.

---

**Pro Tip**: Keep RViz open while developing. Visualizing data in real-time makes debugging much easier. Use multiple RViz displays to see images, transforms, maps, and paths simultaneously.

**Let's build intelligent robots!**
