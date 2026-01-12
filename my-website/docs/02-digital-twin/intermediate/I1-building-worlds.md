# I1: Building Simulation Worlds

> Create custom Gazebo world files with physics, lighting, and terrain.

## Learning Objectives

By the end of this lesson, you will be able to:
- Understand the SDF world file format
- Create a world file from scratch
- Configure physics engine parameters
- Add lighting, ground planes, and obstacles
- Test and debug world files

---

## Prerequisites

- Completed Beginner tier (B1, B2)
- Familiarity with XML/YAML syntax
- Gazebo Classic installed and working

---

## Introduction

In the Beginner tier, you launched a pre-built world. Now you'll learn to create your own simulation environments from scratch. This skill is essential for:

- Testing robots in specific scenarios
- Creating training environments for AI
- Simulating real-world deployment locations

---

## World File Anatomy

Gazebo uses **SDF (Simulation Description Format)** for world files. Here's the structure:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="my_world">
    <!-- Physics Configuration -->
    <physics type="ode">...</physics>

    <!-- Scene Settings -->
    <scene>...</scene>

    <!-- Lighting -->
    <light>...</light>

    <!-- Models (Static and Dynamic) -->
    <model name="ground">...</model>
    <model name="robot">...</model>

    <!-- Plugins -->
    <plugin>...</plugin>
  </world>
</sdf>
```

---

## Step 1: Create a Minimal World

Let's build a world step by step.

### 1.1 Empty World Template

Create a new file `simple_lab.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_lab">

    <!-- We'll add content here -->

  </world>
</sdf>
```

### 1.2 Configure Physics

Add physics configuration inside the `<world>` tag:

```xml
<physics type="ode">
  <real_time_update_rate>1000</real_time_update_rate>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

**Key Parameters**:

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `real_time_update_rate` | 1000 | Physics updates per second |
| `max_step_size` | 0.001 | Time step in seconds |
| `real_time_factor` | 1 | Target RTF (1.0 = real-time) |
| `iters` | 50 | Solver iterations (higher = more accurate) |

### 1.3 Configure Scene

Add scene settings after physics:

```xml
<scene>
  <ambient>0.4 0.4 0.4 1</ambient>
  <background>0.7 0.7 0.7 1</background>
  <shadows>true</shadows>
</scene>
```

### 1.4 Add Lighting

Add directional light (sun):

```xml
<light type="directional" name="sun">
  <cast_shadows>true</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <direction>-0.5 0.1 -0.9</direction>
</light>
```

**Light Types**:
- `directional`: Sun-like, parallel rays
- `point`: Omnidirectional (light bulb)
- `spot`: Cone-shaped (flashlight)

---

## Step 2: Add Ground and Objects

### 2.1 Ground Plane

Every world needs a ground:

```xml
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>
            <mu2>1.0</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
    <visual name="visual">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <material>
        <ambient>0.8 0.8 0.8 1</ambient>
        <diffuse>0.8 0.8 0.8 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

**Important**: `<static>true</static>` prevents the ground from falling!

### 2.2 Add an Obstacle (Box)

```xml
<model name="obstacle_box">
  <static>true</static>
  <pose>2 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.8 0.2 0.2 1</ambient>
        <diffuse>0.8 0.2 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

**Pose Format**: `<pose>x y z roll pitch yaw</pose>`
- Position in meters
- Rotation in radians

### 2.3 Add a Dynamic Object

Objects without `<static>true</static>` will respond to physics:

```xml
<model name="dynamic_ball">
  <pose>0 0 2 0 0 0</pose>
  <link name="link">
    <inertial>
      <mass>1</mass>
      <inertia>
        <ixx>0.1</ixx>
        <iyy>0.1</iyy>
        <izz>0.1</izz>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyz>0</iyz>
      </inertia>
    </inertial>
    <collision name="collision">
      <geometry>
        <sphere>
          <radius>0.2</radius>
        </sphere>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <sphere>
          <radius>0.2</radius>
        </sphere>
      </geometry>
      <material>
        <ambient>0.2 0.8 0.2 1</ambient>
        <diffuse>0.2 0.8 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

**Critical**: Dynamic objects MUST have `<inertial>` with valid mass and inertia!

---

## Step 3: Geometry Types

Gazebo supports these primitive shapes:

| Shape | Element | Parameters |
|-------|---------|------------|
| Box | `<box>` | `<size>x y z</size>` |
| Sphere | `<sphere>` | `<radius>r</radius>` |
| Cylinder | `<cylinder>` | `<radius>r</radius><length>l</length>` |
| Plane | `<plane>` | `<normal>x y z</normal><size>w h</size>` |
| Mesh | `<mesh>` | `<uri>file://path.dae</uri>` |

### Using Meshes

For complex shapes, use mesh files:

```xml
<visual name="visual">
  <geometry>
    <mesh>
      <uri>model://my_model/meshes/body.dae</uri>
      <scale>1 1 1</scale>
    </mesh>
  </geometry>
</visual>
```

**Tip**: Use simple collision geometry even with complex visual meshes for better performance.

---

## Step 4: Complete World File

Here's the complete `simple_lab.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_lab">

    <!-- Physics -->
    <physics type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
      </ode>
    </physics>

    <!-- Scene -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacle -->
    <model name="red_box">
      <static>true</static>
      <pose>3 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- ROS 2 Plugin -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>50</update_rate>
    </plugin>

  </world>
</sdf>
```

---

## Step 5: Launch and Test

### Launch Your World

```bash
source /opt/ros/humble/setup.bash
gazebo --verbose simple_lab.world
```

### Verify ROS 2 Integration

```bash
# In another terminal
ros2 topic list
```

You should see `/gazebo/model_states` and `/clock`.

### Check Performance

```bash
gz stats
```

Target: RTF >= 0.8

---

## Troubleshooting

### Common Errors

| Error | Cause | Solution |
|-------|-------|----------|
| Model falls through ground | Missing `<static>true</static>` | Add static tag to ground |
| Object explodes | Invalid inertia values | Recalculate with correct mass |
| No shadows | GPU issue or config | Check `<shadows>true</shadows>` |
| Low RTF | Too many objects | Simplify collision meshes |

### Validation Command

```bash
# Check SDF syntax
gz sdf -p simple_lab.world
```

---

## What's Next?

In the next lesson, you'll learn to spawn your URDF humanoid model into these worlds and control its joints.

**Next**: [I2: Spawning and Controlling Models](I2-spawning-models.md)

---

## AI Agent Assisted Prompts

### World Generation
```
Generate a Gazebo SDF world file for a warehouse environment with:
- 10m x 20m floor area
- Shelving units along the walls
- A loading dock area
- Proper lighting and physics settings for humanoid robot testing
```

### Performance Optimization
```
My Gazebo world has RTF 0.3 with 50 objects. Analyze common performance
bottlenecks and suggest optimizations. Include both SDF changes and
runtime flags I can use.
```

### Custom Physics
```
I need to simulate a humanoid robot walking on ice (low friction) vs rubber mat
(high friction). Show me how to configure multiple floor surfaces with different
friction coefficients in the same world file.
```

---

## Summary

- World files use SDF (XML-based) format
- Every world needs: physics, scene, lighting, ground
- Static objects don't move; dynamic objects need inertia
- Use primitive shapes for collision, meshes for visuals
- ROS 2 plugins enable topic-based communication
- Validate with `gz sdf -p` and monitor RTF

---

| Previous | Up | Next |
|----------|-----|------|
| [B2: First Simulation](../beginner/B2-first-simulation.md) | [Intermediate Tier](../README.md#intermediate-tier) | [I2: Spawning Models](I2-spawning-models.md) |
