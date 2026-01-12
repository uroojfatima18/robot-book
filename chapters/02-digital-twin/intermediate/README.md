# Intermediate Tier: Building Custom Simulation Worlds

**Duration**: 2-4 hours | **Prerequisite**: Beginner Tier Completion

---

## Overview

Welcome to the Intermediate tier! Now that you understand digital twin concepts and can run pre-built simulations, it's time to create your own custom simulation worlds and spawn robots programmatically.

In this tier, you'll transition from **using** simulations to **building** them. You'll learn to design custom environments, spawn URDF models, and control robots through ROS 2.

---

## Learning Objectives

By the end of this tier, you will be able to:

1. **Create** custom Gazebo world files from scratch
2. **Configure** physics parameters, lighting, and environment properties
3. **Spawn** URDF robot models programmatically using ROS 2
4. **Control** robot joints through ROS 2 topics
5. **Write** launch files that orchestrate simulation startup
6. **Debug** common simulation issues
7. **Optimize** simulation performance for real-time operation

---

## What You'll Learn

### World Building Skills
- SDF (Simulation Description Format) syntax
- Physics engine configuration
- Lighting and rendering setup
- Ground plane and environment design
- Model placement and organization

### Robot Integration Skills
- Spawning URDF models with spawn_entity.py
- Joint state monitoring and control
- ROS 2 topic communication with simulation
- Launch file creation for multi-node systems
- Parameter configuration and remapping

### Practical Development
- Iterative world design workflow
- Performance optimization techniques
- Debugging simulation issues
- Testing robot behaviors safely

---

## Lesson Structure

This tier contains **2 comprehensive lessons**:

### Lesson I1: Building Simulation Worlds
**File**: [I1-building-worlds.md](./I1-building-worlds.md)
**Duration**: 60-90 minutes

Learn to create custom Gazebo worlds:
- SDF file structure and syntax
- Physics configuration (gravity, solver settings)
- Lighting setup (sun, ambient, shadows)
- Ground plane and basic geometry
- Model library integration

**Includes**: Complete world file examples and templates.

### Lesson I2: Spawning and Controlling Models
**File**: [I2-spawning-models.md](./I2-spawning-models.md)
**Duration**: 60-90 minutes

Master robot spawning and control:
- Using spawn_entity.py from gazebo_ros
- URDF model preparation for Gazebo
- Joint state topics and control interfaces
- Writing ROS 2 launch files
- Multi-node coordination

**Includes**: Python code examples and launch file templates.

---

## Prerequisites

### Knowledge Prerequisites
- **Beginner Tier Completion**: Understanding of digital twin concepts and Gazebo basics
- **Chapter 1 Advanced**: URDF fundamentals (links, joints, transmissions)
- **ROS 2 Intermediate**: Topics, services, launch files, parameters

### Technical Prerequisites
- **Gazebo Classic 11.x** installed and working
- **ROS 2 Humble** with gazebo_ros_pkgs
- **Text editor** (VS Code recommended with XML/Python extensions)
- **Basic Python** knowledge for control scripts

---

## Time Commitment

| Activity | Time |
|----------|------|
| Lesson I1: Building Worlds | 60-90 min |
| Lesson I2: Spawning Models | 60-90 min |
| Exercises | 45-60 min |
| **Total** | **2-4 hours** |

---

## Learning Path

```
Beginner Tier Complete
    ↓
I1: Building Simulation Worlds (60-90 min)
    ↓
I2: Spawning and Controlling Models (60-90 min)
    ↓
Intermediate Exercises (45-60 min)
    ↓
Ready for Advanced Tier!
```

---

## What You'll Build

By the end of this tier, you'll have:

1. **Custom World File**: A simulation environment you designed from scratch
2. **Robot Spawning System**: Launch files that spawn and configure robots
3. **Joint Controller**: Python node that commands robot joints
4. **Complete Simulation Stack**: Integrated system ready for testing

### Example Project: Laboratory Environment
You'll build a simulated robotics lab with:
- Custom floor and walls
- Proper lighting for camera sensors
- Spawn points for multiple robots
- ROS 2 integration for control

---

## Success Criteria

You're ready to move to the Advanced tier when you can:

- [ ] Write a world file from scratch with physics and lighting
- [ ] Spawn a URDF robot at a specific position and orientation
- [ ] Monitor joint states through ROS 2 topics
- [ ] Command robot joints programmatically
- [ ] Create a launch file that starts Gazebo and spawns a robot
- [ ] Debug common spawning and control issues
- [ ] Optimize world files for real-time performance

---

## Assets Provided

### Templates
- `assets/simple_lab.world` - Minimal world file template
- `assets/launch/spawn_humanoid.launch.py` - Launch file template
- `assets/src/joint_commander.py` - Joint control example

### Code Examples
All code examples are production-ready and include:
- Error handling
- Documentation
- Type hints (Python)
- ROS 2 best practices

---

## Common Challenges

### Challenge 1: Model Won't Spawn
**Symptoms**: spawn_entity.py fails or model appears incorrectly
**Solutions**:
- Verify URDF is valid with `check_urdf`
- Ensure all mesh files are accessible
- Check Gazebo is fully started before spawning

### Challenge 2: Joints Don't Move
**Symptoms**: Joint commands sent but robot doesn't respond
**Solutions**:
- Verify transmission tags in URDF
- Check controller is loaded and running
- Confirm topic names match

### Challenge 3: Slow Simulation
**Symptoms**: RTF < 0.5, laggy interface
**Solutions**:
- Simplify collision geometry
- Reduce physics iterations
- Close unnecessary applications

---

## Development Workflow

### Iterative World Design
1. Start with minimal world (ground + lighting)
2. Test in Gazebo to verify it loads
3. Add one element at a time
4. Test after each addition
5. Optimize once complete

### Robot Integration
1. Verify URDF works in RViz first
2. Add Gazebo-specific tags (inertia, collision)
3. Test spawning in empty world
4. Add to custom world
5. Implement control interface

---

## Tools and Resources

### Recommended Tools
- **VS Code** with XML and Python extensions
- **RViz2** for URDF visualization
- **check_urdf** for URDF validation
- **gz stats** for performance monitoring

### External Resources
- [SDF Format Specification](http://sdformat.org/)
- [Gazebo Model Database](https://github.com/osrf/gazebo_models)
- [gazebo_ros_pkgs Documentation](https://github.com/ros-simulation/gazebo_ros_pkgs)

---

## Next Steps

After completing this tier:

1. **Complete the Intermediate Exercises**: Build a complete simulation environment
2. **Move to Advanced Tier**: Learn bidirectional synchronization and bridge nodes
3. **Experiment**: Try adding sensors, multiple robots, or complex environments

---

## Ready to Build?

**Start with Lesson I1**: [Building Simulation Worlds](./I1-building-worlds.md)

You'll create your first custom world file and learn the fundamentals of SDF syntax and Gazebo configuration.

---

**Pro Tip**: Keep your world files simple at first. It's easier to debug a minimal world and add complexity incrementally than to troubleshoot a complex world that won't load.

**Let's start building!**
