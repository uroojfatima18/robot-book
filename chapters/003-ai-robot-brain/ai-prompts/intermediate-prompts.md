# Intermediate AI Prompts - AI-Robot Brain

These prompts help you implement perception, SLAM, and navigation systems in ROS 2. Use these when working through the Intermediate tier lessons.

---

## Camera and Depth Processing

### Understanding cv_bridge

```
I'm learning to use cv_bridge for image processing. Can you help me:
1. Understand what cv_bridge does
2. Show me how to convert ROS Image messages to OpenCV format
3. Explain the different encoding types (bgr8, rgb8, mono8, etc.)
4. Debug conversion errors

I want to process camera images in my ROS 2 node.
```

### Processing Camera Images

```
I'm writing a node to process camera images. Can you help me:
1. Subscribe to camera topics correctly
2. Convert images using cv_bridge
3. Apply OpenCV operations (filtering, edge detection, etc.)
4. Publish processed images back to ROS 2

Here's my current code:
[PASTE YOUR CODE]

What I'm trying to do: [DESCRIBE YOUR GOAL]
```

### Working with Depth Data

```
I'm trying to process depth camera data. Can you help me:
1. Understand the depth image format
2. Convert depth values to actual distances
3. Handle invalid depth readings (NaN, inf)
4. Combine depth with RGB data

Sensor I'm using: [SENSOR TYPE]
Error (if any): [PASTE ERROR]
```

### Image Synchronization

```
I need to synchronize RGB and depth images. Can you help me:
1. Understand message_filters and ApproximateTimeSynchronizer
2. Set up synchronized subscribers
3. Handle timing mismatches
4. Debug synchronization issues

My setup:
- RGB topic: [TOPIC NAME]
- Depth topic: [TOPIC NAME]
- Expected rate: [HZ]
```

---

## TF2 Coordinate Frames

### Understanding TF2

```
I'm learning about TF2 and coordinate frames. Can you help me:
1. Understand what TF2 does and why it's needed
2. Explain common frames (base_link, odom, map, camera_link)
3. Understand the transform tree structure
4. Visualize my robot's TF tree

I'm working with: [DESCRIBE YOUR ROBOT]
```

### Publishing Transforms

```
I need to publish a transform between frames. Can you help me:
1. Understand StaticTransformBroadcaster vs TransformBroadcaster
2. Write code to publish a transform
3. Set the correct parent and child frames
4. Handle timing and update rates

Transform I need: [DESCRIBE FRAMES AND RELATIONSHIP]
```

### Looking Up Transforms

```
I'm trying to look up transforms in my code. Can you help me:
1. Use tf2_ros.Buffer and TransformListener
2. Look up transforms between specific frames
3. Handle exceptions when transforms aren't available
4. Transform points between frames

What I'm trying to do: [DESCRIBE YOUR USE CASE]
```

### Debugging TF Issues

```
I'm having TF2 problems. Can you help me:
1. Use view_frames to visualize my TF tree
2. Identify missing or broken transforms
3. Debug "frame does not exist" errors
4. Fix timing issues with transforms

Error message: [PASTE ERROR]
My TF tree: [DESCRIBE OR PASTE view_frames OUTPUT]
```

---

## SLAM Toolbox

### Installing and Configuring SLAM Toolbox

```
I'm setting up SLAM Toolbox. Can you help me:
1. Verify installation is correct
2. Understand the configuration parameters
3. Choose appropriate settings for my robot
4. Create a launch file for SLAM

My robot:
- Type: [MOBILE ROBOT TYPE]
- Sensors: [LIST SENSORS]
- Environment: [INDOOR/OUTDOOR, SIZE]
```

### Running SLAM

```
I'm trying to run SLAM Toolbox. Can you help me:
1. Launch SLAM with my robot
2. Verify it's receiving sensor data
3. Monitor map building progress
4. Troubleshoot if mapping isn't working

My launch command: [PASTE COMMAND]
Topics I'm publishing: [LIST TOPICS]
Issue: [DESCRIBE PROBLEM IF ANY]
```

### Tuning SLAM Parameters

```
My SLAM results aren't good. Can you help me:
1. Understand key tuning parameters
2. Adjust resolution and update rates
3. Configure loop closure detection
4. Optimize for my environment

Current issues:
- [DESCRIBE PROBLEMS: drift, poor quality, etc.]

Environment characteristics: [DESCRIBE]
```

### Saving and Loading Maps

```
I want to save and load maps with SLAM Toolbox. Can you help me:
1. Save a map after SLAM session
2. Load a saved map for localization
3. Understand map file formats
4. Use saved maps with Nav2

Commands I'm using: [PASTE COMMANDS IF ANY]
```

---

## Nav2 Basics

### Setting Up Nav2

```
I'm setting up Nav2 for the first time. Can you help me:
1. Verify Nav2 installation
2. Understand the required configuration files
3. Create a basic Nav2 launch file
4. Configure parameters for my robot

My robot specifications:
- Footprint: [DIMENSIONS]
- Max velocity: [VALUES]
- Sensors: [LIST]
```

### Sending Navigation Goals

```
I want to send navigation goals programmatically. Can you help me:
1. Write a Python node to send goals
2. Use the NavigateToPose action
3. Monitor goal progress and status
4. Handle goal success/failure

Here's my current code:
[PASTE YOUR CODE]

What I'm trying to achieve: [DESCRIBE]
```

### Visualizing in RViz

```
I'm trying to visualize Nav2 in RViz. Can you help me:
1. Configure RViz for navigation
2. Add the right displays (map, costmap, path, etc.)
3. Understand what I'm seeing
4. Debug visualization issues

What I see: [DESCRIBE OR SCREENSHOT]
What I expect: [DESCRIBE]
```

### Nav2 Lifecycle Management

```
I'm confused about Nav2 lifecycle nodes. Can you help me:
1. Understand lifecycle node states
2. Configure and activate Nav2 nodes
3. Handle lifecycle transitions
4. Debug lifecycle issues

Error I'm seeing: [PASTE ERROR IF ANY]
```

---

## Integration and Debugging

### Integrating Perception with Navigation

```
I want to integrate my perception system with Nav2. Can you help me:
1. Connect sensor data to Nav2 costmaps
2. Configure obstacle detection
3. Ensure proper frame transformations
4. Debug integration issues

My setup:
- Sensors: [LIST]
- Nav2 config: [DESCRIBE]
- Issue: [DESCRIBE PROBLEM]
```

### Debugging Perception Pipeline

```
My perception pipeline isn't working correctly. Can you help me:
1. Verify sensor data is being published
2. Check image processing is working
3. Ensure transforms are correct
4. Debug performance issues

Symptoms: [DESCRIBE WHAT'S WRONG]
Topics: [LIST RELEVANT TOPICS]
```

### Debugging SLAM Issues

```
My SLAM isn't working well. Can you help me:
1. Diagnose the problem (drift, poor quality, crashes)
2. Check sensor data quality
3. Verify odometry is correct
4. Tune parameters to fix issues

Current behavior: [DESCRIBE]
Expected behavior: [DESCRIBE]
Environment: [DESCRIBE]
```

### Debugging Navigation Issues

```
Nav2 isn't working as expected. Can you help me:
1. Identify why the robot won't move
2. Debug path planning failures
3. Understand why goals are rejected
4. Fix obstacle avoidance problems

Symptoms: [DESCRIBE BEHAVIOR]
Configuration: [DESCRIBE YOUR SETUP]
Logs: [PASTE RELEVANT LOGS]
```

---

## Performance Optimization

### Optimizing Image Processing

```
My image processing is too slow. Can you help me:
1. Profile where the bottleneck is
2. Optimize OpenCV operations
3. Reduce image resolution if appropriate
4. Consider GPU acceleration

Current performance: [FPS OR LATENCY]
Target performance: [DESIRED FPS OR LATENCY]
Processing steps: [DESCRIBE]
```

### Optimizing SLAM Performance

```
SLAM is running slowly on my robot. Can you help me:
1. Identify performance bottlenecks
2. Tune parameters for better performance
3. Reduce computational load
4. Balance accuracy vs speed

Current performance: [DESCRIBE]
Hardware: [CPU, RAM]
Map size: [DIMENSIONS]
```

### Optimizing Navigation Performance

```
Nav2 is slow or unresponsive. Can you help me:
1. Profile navigation performance
2. Tune planner parameters
3. Optimize costmap updates
4. Reduce computational overhead

Issues: [DESCRIBE PERFORMANCE PROBLEMS]
Hardware: [DESCRIBE]
```

---

## Code Review and Best Practices

### Review My Camera Processing Node

```
I've written a camera processing node. Can you review it for:
1. Correct ROS 2 patterns
2. Proper cv_bridge usage
3. Error handling
4. Performance optimization

Here's my code:
[PASTE YOUR CODE]

What it should do: [DESCRIBE FUNCTIONALITY]
```

### Review My TF2 Broadcasting

```
I've implemented TF2 broadcasting. Can you review it for:
1. Correct frame relationships
2. Proper timing and update rates
3. Static vs dynamic transforms
4. Best practices

Here's my code:
[PASTE YOUR CODE]

Frame structure: [DESCRIBE YOUR TF TREE]
```

### Review My Nav2 Configuration

```
I've configured Nav2 for my robot. Can you review it for:
1. Appropriate parameter values
2. Correct costmap configuration
3. Suitable planner choices
4. Potential issues

Here's my configuration:
[PASTE YOUR CONFIG FILES]

Robot specs: [DESCRIBE]
Environment: [DESCRIBE]
```

---

## Launch Files and Configuration

### Creating Complex Launch Files

```
I need to create a launch file that starts multiple nodes. Can you help me:
1. Launch Gazebo, SLAM, and Nav2 together
2. Handle node dependencies
3. Pass parameters correctly
4. Debug launch file errors

What I need to launch: [DESCRIBE YOUR SYSTEM]
Current launch file: [PASTE IF YOU HAVE ONE]
```

### Parameter Configuration

```
I'm configuring parameters for my navigation stack. Can you help me:
1. Understand which parameters are most important
2. Choose appropriate values for my robot
3. Organize parameters in YAML files
4. Override parameters at launch time

Robot specifications: [DESCRIBE]
Use case: [DESCRIBE]
```

### Multi-Robot Configuration

```
I want to run multiple robots with SLAM/Nav2. Can you help me:
1. Set up proper namespacing
2. Avoid topic conflicts
3. Configure separate maps
4. Launch multiple instances

Number of robots: [NUMBER]
Robot types: [DESCRIBE]
```

---

## Real-World Considerations

### Preparing for Real Hardware

```
I've been working in simulation. Can you help me prepare for real hardware:
1. Identify what will be different
2. Understand sensor calibration needs
3. Plan for safety considerations
4. Anticipate common issues

My robot: [DESCRIBE]
Sensors: [LIST]
Environment: [DESCRIBE]
```

### Sensor Calibration

```
I need to calibrate my sensors. Can you help me:
1. Understand what calibration is needed
2. Calibrate camera intrinsics
3. Calibrate camera-LIDAR extrinsics
4. Verify calibration quality

Sensors I have: [LIST]
Calibration tools available: [DESCRIBE]
```

### Safety Considerations

```
I'm deploying to real hardware. Can you help me:
1. Implement safety checks
2. Add emergency stop functionality
3. Limit velocities appropriately
4. Handle sensor failures gracefully

Robot type: [DESCRIBE]
Environment: [DESCRIBE]
Safety concerns: [LIST]
```

---

## Tips for Intermediate Learners

1. **Test Incrementally**: Get each component working before integrating
2. **Use Visualization**: RViz is your friend for debugging
3. **Check Topics**: Use `ros2 topic echo` to verify data flow
4. **Read Logs**: Error messages usually tell you what's wrong
5. **Start Simple**: Use default configs first, then tune

---

## When to Ask for Help

- **Integration Issues**: Connecting components often reveals edge cases
- **Performance Problems**: Profiling and optimization require expertise
- **Configuration Confusion**: So many parameters - which matter?
- **Debugging Deadlocks**: When you're stuck, get a fresh perspective
- **Best Practices**: Learn from experienced developers

---

**Remember**: Implementation is iterative. Your first version won't be perfect. Test, debug, refine, repeat. Focus on getting it working first, then optimize.
