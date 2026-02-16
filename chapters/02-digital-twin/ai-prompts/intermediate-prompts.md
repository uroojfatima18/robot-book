# Intermediate AI Prompts - Digital Twin & Simulation

These prompts help you build custom simulation worlds and spawn robots programmatically. Use these when working through the Intermediate tier lessons.

---

## World Building Prompts

### Understanding SDF Format

```
I'm learning to create Gazebo world files in SDF format. Can you help me:
1. Understand the basic structure of an SDF file
2. Explain the key sections (world, physics, lighting, models)
3. Show me a minimal working example
4. Identify common mistakes beginners make

I want to understand the format before writing my own world file.
```

### Physics Configuration

```
I'm configuring physics parameters in my world file. Can you explain:
1. What the different physics engines are (ODE, Bullet, etc.)
2. How gravity settings work
3. What solver iterations mean and how to tune them
4. How physics settings affect simulation performance

Please include examples of good default values.
```

### Lighting Setup

```
I'm trying to set up lighting in my custom world. Can you help me:
1. Understand the difference between sun, ambient, and point lights
2. Configure realistic lighting for indoor vs outdoor scenes
3. Optimize lighting for camera sensor simulation
4. Debug why my world looks too dark or washed out

My use case: [DESCRIBE YOUR ROBOT AND ENVIRONMENT]
```

### Ground Plane and Geometry

```
I want to add a ground plane and basic geometry to my world. Can you:
1. Show me how to create a ground plane with proper collision
2. Explain how to add simple shapes (boxes, cylinders, spheres)
3. Help me understand collision vs visual geometry
4. Demonstrate how to position and orient objects

I'm building: [DESCRIBE YOUR ENVIRONMENT]
```

---

## Robot Spawning Prompts

### Using spawn_entity.py

```
I'm trying to spawn a URDF robot using spawn_entity.py. Can you help me:
1. Understand the command-line arguments
2. Set the initial position and orientation
3. Configure the robot namespace
4. Debug why my robot won't spawn

My command: [PASTE YOUR COMMAND]
Error (if any): [PASTE ERROR MESSAGE]
```

### URDF Preparation for Gazebo

```
My URDF works in RViz but not in Gazebo. Can you help me:
1. Understand what Gazebo-specific tags are needed
2. Add proper inertia to all links
3. Configure collision geometry
4. Add transmission elements for joint control

Here's my URDF: [PASTE RELEVANT SECTIONS OR DESCRIBE STRUCTURE]
```

### Joint State Monitoring

```
I've spawned my robot but can't see joint states. Can you help me:
1. Identify which topics publish joint states
2. Show me how to echo joint state messages
3. Understand the JointState message format
4. Verify my robot's joints are being published

My robot has [NUMBER] joints: [LIST JOINT NAMES]
```

### Joint Control Implementation

```
I want to control my robot's joints programmatically. Can you help me:
1. Identify which topics to publish to for joint control
2. Show me a Python example for commanding joints
3. Explain the message types used (JointTrajectory, etc.)
4. Debug why my joint commands aren't working

My robot type: [DESCRIBE ROBOT]
Control interface: [DESCRIBE WHAT YOU'RE TRYING TO DO]
```

---

## Launch File Prompts

### Creating Launch Files

```
I'm writing my first ROS 2 launch file for Gazebo. Can you help me:
1. Understand the Python launch file structure
2. Show me how to launch Gazebo with a world file
3. Add robot spawning to the launch file
4. Configure parameters and remappings

I want to launch: [DESCRIBE YOUR SETUP]
```

### Multi-Node Coordination

```
I need to launch multiple nodes together (Gazebo, spawner, controller). Can you:
1. Show me how to launch nodes in sequence
2. Explain how to pass parameters between nodes
3. Help me handle node dependencies
4. Debug launch file errors

My setup: [DESCRIBE YOUR NODES AND DEPENDENCIES]
Error (if any): [PASTE ERROR]
```

### Parameter Configuration

```
I'm trying to configure ROS 2 parameters in my launch file. Can you help me:
1. Understand how to pass parameters to nodes
2. Load parameters from YAML files
3. Override parameters at launch time
4. Debug parameter-related issues

Parameters I need: [LIST YOUR PARAMETERS]
```

---

## Debugging Prompts

### Model Explodes on Spawn

```
My robot explodes or behaves erratically when spawned. Can you help me:
1. Understand what causes this (usually inertia issues)
2. Check if my inertia tensors are valid
3. Verify collision geometry is correct
4. Fix the URDF to prevent explosions

My robot URDF: [PASTE RELEVANT SECTIONS]
Behavior: [DESCRIBE WHAT HAPPENS]
```

### Joints Won't Move

```
I'm sending joint commands but nothing happens. Can you help me:
1. Verify the controller is loaded and running
2. Check if transmission tags are correct
3. Confirm topic names match
4. Debug the control pipeline

My setup:
- Robot: [DESCRIBE]
- Controller: [TYPE]
- Topics I'm publishing to: [LIST]
- What I see: [DESCRIBE BEHAVIOR]
```

### Slow Simulation Performance

```
My custom world runs slowly (RTF < 0.5). Can you help me:
1. Profile what's causing the slowdown
2. Simplify collision geometry
3. Optimize physics settings
4. Reduce visual complexity

My world includes: [DESCRIBE YOUR WORLD]
System specs: [DESCRIBE YOUR HARDWARE]
```

### World File Won't Load

```
Gazebo gives an error when loading my world file. Can you help me:
1. Validate my SDF syntax
2. Check for missing required elements
3. Verify file paths are correct
4. Fix the error

My world file: [PASTE RELEVANT SECTIONS]
Error message: [PASTE FULL ERROR]
```

---

## Code Review Prompts

### Review My World File

```
I've created a custom world file. Can you review it for:
1. Syntax errors or missing elements
2. Performance optimization opportunities
3. Best practices I should follow
4. Potential issues I might encounter

Here's my world file:
[PASTE YOUR WORLD FILE]

My use case: [DESCRIBE WHAT YOU'RE BUILDING]
```

### Review My Launch File

```
I've written a launch file for my simulation. Can you review it for:
1. Correct node launching sequence
2. Proper parameter passing
3. Error handling
4. Best practices

Here's my launch file:
[PASTE YOUR LAUNCH FILE]

What it should do: [DESCRIBE EXPECTED BEHAVIOR]
```

### Review My Joint Controller

```
I've written a Python node to control joints. Can you review it for:
1. Correct ROS 2 patterns
2. Proper message handling
3. Error handling and safety
4. Performance optimization

Here's my code:
[PASTE YOUR CODE]

What it should do: [DESCRIBE FUNCTIONALITY]
```

---

## Optimization Prompts

### Improving Simulation Performance

```
I want to optimize my simulation for better performance. Can you help me:
1. Identify performance bottlenecks
2. Simplify collision meshes
3. Tune physics parameters
4. Reduce unnecessary visual detail

Current RTF: [YOUR RTF]
Target RTF: [DESIRED RTF]
World complexity: [DESCRIBE YOUR WORLD]
```

### Reducing Latency

```
I'm experiencing latency between commands and robot response. Can you help me:
1. Measure where the latency is coming from
2. Optimize message passing
3. Tune QoS settings
4. Improve controller responsiveness

Observed latency: [DESCRIBE]
Control frequency: [HZ]
```

---

## Integration Prompts

### Adding Sensors to Simulation

```
I want to add sensors to my simulated robot. Can you help me:
1. Understand what sensors Gazebo can simulate
2. Add a camera/LIDAR/IMU to my URDF
3. Configure sensor parameters
4. Access sensor data through ROS 2 topics

Sensor I want to add: [SENSOR TYPE]
My robot: [DESCRIBE]
```

### Multi-Robot Simulation

```
I want to spawn multiple robots in the same world. Can you help me:
1. Understand namespace management
2. Spawn robots with unique names
3. Control multiple robots independently
4. Avoid topic name conflicts

Number of robots: [NUMBER]
Robot type: [DESCRIBE]
```

### Connecting to External Controllers

```
I want to connect my simulation to an external controller. Can you help me:
1. Understand the topic interface
2. Set up bidirectional communication
3. Handle timing and synchronization
4. Debug connection issues

Controller type: [DESCRIBE]
Communication pattern: [DESCRIBE]
```

---

## Project Planning Prompts

### Designing a Custom Environment

```
I'm planning a custom simulation environment. Can you help me:
1. Identify what elements I need
2. Plan the world file structure
3. Consider performance implications
4. Outline the implementation steps

My project: [DESCRIBE YOUR ROBOTICS PROJECT]
Requirements: [LIST REQUIREMENTS]
```

### Preparing for Advanced Tier

```
I've completed the Intermediate tier. Can you help me:
1. Review key concepts I should understand
2. Assess if I'm ready for bridge nodes and synchronization
3. Identify any gaps in my knowledge
4. Preview what's coming in the Advanced tier

What I've built: [DESCRIBE YOUR WORK]
```

---

## Tips for Intermediate Learners

1. **Start Simple**: Build minimal worlds first, add complexity incrementally
2. **Test Frequently**: Test after each change to catch issues early
3. **Use Version Control**: Save working versions before making changes
4. **Read Error Messages**: They usually tell you exactly what's wrong
5. **Check Examples**: Look at existing world files and launch files for patterns

---

## When to Ask for Help

- **Syntax Errors**: SDF and launch file syntax can be tricky
- **Performance Issues**: Optimization requires understanding bottlenecks
- **Integration Problems**: Connecting components often reveals edge cases
- **Design Decisions**: Get feedback on architecture before implementing
- **Code Review**: Have your work reviewed before moving to production

---

**Remember**: Building simulation environments is iterative. Don't expect perfection on the first try. Test, debug, refine, repeat.
