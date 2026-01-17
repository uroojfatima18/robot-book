---
id: a_lesson1_urdf_humanoid
title: "URDF & Humanoid Robot Description"
sidebar_position: 2
tier: advanced
chapter: chapter_1_ros2
estimated_time: "2-3 hours"
prerequisites: ["i_lesson2_python_ros"]
---

# URDF & Humanoid Robot Description

## Learning Objectives

By the end of this lesson, you will be able to:

- **Understand** URDF file structure and XML syntax
- **Create** links with visual and collision geometry
- **Define** joints that connect links with appropriate constraints
- **Build** a basic humanoid skeleton URDF model
- **Visualize** robot models in RViz2

## Introduction

Every robot needs a description of its physical structure—its body, limbs, sensors, and how they connect. In ROS 2, this description is written in **URDF** (Unified Robot Description Format).

Think of URDF as a blueprint that tells ROS 2: "This robot has a torso connected to a head by a neck joint, two arms connected by shoulder joints, and legs connected by hip joints." This description enables visualization in RViz2, simulation in Gazebo, and is essential for motion planning and control.

In this lesson, you'll learn URDF from the ground up, culminating in building a basic humanoid robot model that you can visualize and extend.

---

## URDF File Structure

### Theory

URDF files are XML documents describing a robot as a **tree of links connected by joints**:

```
┌─────────────────────────────────────────────────────────────┐
│                    URDF Tree Structure                       │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│                       base_link                              │
│                           │                                  │
│              ┌────────────┼────────────┐                    │
│              │            │            │                    │
│           torso       left_hip     right_hip                │
│              │            │            │                    │
│       ┌──────┼──────┐   left_leg   right_leg               │
│       │      │      │                                       │
│    head  left_arm  right_arm                                │
│                                                              │
│    Links: Physical bodies (boxes, cylinders, meshes)        │
│    Joints: Connections with constraints (fixed, revolute)   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

**URDF Key Elements:**

| Element | Purpose |
|---------|---------|
| `<robot>` | Root element, contains all links and joints |
| `<link>` | A rigid body with visual and collision geometry |
| `<joint>` | Connects two links with motion constraints |
| `<visual>` | How the link looks (for display) |
| `<collision>` | Simplified geometry for physics calculations |
| `<inertial>` | Mass and inertia for dynamics simulation |

### Code Example: Minimal URDF

```xml
<?xml version="1.0"?>
<!--
  Minimal URDF example: A single link robot
-->
<robot name="minimal_robot">

  <!-- Every URDF must have at least one link -->
  <link name="base_link">
    <!-- Visual: How it appears in RViz -->
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
  </link>

</robot>
```

---

## Links: Bodies and Geometry

### Theory

A **link** represents a rigid body in the robot. Each link can have:

- **Visual geometry**: What you see in RViz2 (can be detailed)
- **Collision geometry**: Used for physics (usually simplified)
- **Inertial properties**: Mass and moments of inertia

**Available geometry primitives:**

| Primitive | Parameters | Use Case |
|-----------|------------|----------|
| `<box>` | `size="x y z"` | Bodies, platforms |
| `<cylinder>` | `radius="r" length="l"` | Limbs, joints |
| `<sphere>` | `radius="r"` | Wheels, heads |
| `<mesh>` | `filename="path.stl"` | Complex shapes |

### Code Example: Link with Full Properties

```xml
<link name="torso">
  <!-- Visual: What you see -->
  <visual>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>

  <!-- Collision: For physics simulation -->
  <collision>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>

  <!-- Inertial: Mass and inertia (required for simulation) -->
  <inertial>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <mass value="10.0"/>
    <inertia
      ixx="0.1" ixy="0.0" ixz="0.0"
      iyy="0.1" iyz="0.0"
      izz="0.05"/>
  </inertial>
</link>
```

**Understanding `<origin>`:**

The `<origin>` tag positions geometry relative to the link's coordinate frame:
- `xyz="x y z"`: Translation in meters
- `rpy="roll pitch yaw"`: Rotation in radians (roll=X, pitch=Y, yaw=Z)

---

## Joints: Connections and Constraints

### Theory

**Joints** connect links and define how they can move relative to each other.

**Joint types:**

| Type | Motion | Example |
|------|--------|---------|
| `fixed` | No motion | Head to neck (solid connection) |
| `revolute` | Rotation with limits | Elbow (0° to 150°) |
| `continuous` | Unlimited rotation | Wheel |
| `prismatic` | Linear sliding | Telescope arm |
| `floating` | 6 DOF (rarely used) | Free body |
| `planar` | 2D translation + rotation | Omni wheel |

### Code Example: Joint Definitions

```xml
<!-- Fixed joint: No motion allowed -->
<joint name="head_joint" type="fixed">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
</joint>

<!-- Revolute joint: Rotation with limits -->
<joint name="left_shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.15 0.1 0.4" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>  <!-- Rotation axis -->
  <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
</joint>

<!-- Continuous joint: Unlimited rotation -->
<joint name="left_wheel_joint" type="continuous">
  <parent link="left_leg"/>
  <child link="left_wheel"/>
  <origin xyz="0 0 -0.5" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>

<!-- Prismatic joint: Linear motion -->
<joint name="gripper_joint" type="prismatic">
  <parent link="wrist"/>
  <child link="gripper"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.05" effort="10" velocity="0.5"/>
</joint>
```

**Joint limit parameters:**

- `lower`, `upper`: Position limits (radians for revolute, meters for prismatic)
- `effort`: Maximum force/torque
- `velocity`: Maximum speed

---

## Visual and Collision Elements

### Theory

**Visual geometry** is for display only. You can use detailed meshes, multiple colors, and decorative elements.

**Collision geometry** is used by physics engines. It should be:
- Simpler than visual geometry (faster computation)
- Slightly larger (safety margin)
- Convex when possible (more stable physics)

### Code Example: Visual vs Collision

```xml
<link name="hand">
  <!-- Visual: Detailed hand mesh -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://my_robot/meshes/hand_detailed.stl" scale="0.001 0.001 0.001"/>
    </geometry>
    <material name="skin">
      <color rgba="0.9 0.7 0.6 1.0"/>
    </material>
  </visual>

  <!-- Collision: Simple box approximation -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.08 0.04 0.15"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

**Using STL Meshes:**

STL files from CAD software can be used for detailed visuals:
- Use `scale` to convert units (many CAD tools export in mm, ROS uses m)
- Place meshes in a `meshes/` directory in your package
- Reference with `package://package_name/meshes/file.stl`

---

## Building a Humanoid Skeleton

### Theory

A humanoid robot URDF typically follows this hierarchy:

```
base_link (pelvis/hips)
├── torso
│   ├── head
│   ├── left_shoulder → left_upper_arm → left_forearm → left_hand
│   └── right_shoulder → right_upper_arm → right_forearm → right_hand
├── left_hip → left_thigh → left_shin → left_foot
└── right_hip → right_thigh → right_shin → right_foot
```

**Design principles:**
1. Start from `base_link` (the "root" of the robot)
2. Build outward along kinematic chains
3. Use consistent naming conventions
4. Match joint axes to expected motion

### Code Example: Basic Humanoid URDF

The following is a simplified humanoid with torso, head, and one arm:

```xml
<?xml version="1.0"?>
<!--
  Chapter 1: The Robotic Nervous System (ROS 2)
  URDF: Basic Humanoid Robot
  Description: Minimal humanoid structure for RViz visualization
-->
<robot name="humanoid_basic">

  <!-- Define materials (colors) -->
  <material name="gray">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.2 0.2 0.8 1.0"/>
  </material>
  <material name="orange">
    <color rgba="0.9 0.5 0.1 1.0"/>
  </material>

  <!-- BASE LINK: The root of the robot (pelvis) -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.3 0.15"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.3 0.15"/>
      </geometry>
    </collision>
  </link>

  <!-- TORSO -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.4"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.4"/>
      </geometry>
    </collision>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.075" rpy="0 0 0"/>
  </joint>

  <!-- HEAD -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Yaw rotation (left/right) -->
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1.0"/>
  </joint>

  <!-- LEFT ARM: Upper Arm -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.15 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Pitch rotation (forward/backward) -->
    <limit lower="-2.0" upper="2.0" effort="30" velocity="1.5"/>
  </joint>

  <!-- LEFT ARM: Forearm -->
  <link name="left_forearm">
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.2"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Pitch rotation -->
    <limit lower="0" upper="2.5" effort="20" velocity="2.0"/>
  </joint>

  <!-- RIGHT ARM: Upper Arm -->
  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0.15 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="30" velocity="1.5"/>
  </joint>

  <!-- RIGHT ARM: Forearm -->
  <link name="right_forearm">
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.2"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="20" velocity="2.0"/>
  </joint>

</robot>
```

---

## RViz2 Visualization

### Theory

**RViz2** is the standard visualization tool for ROS 2. To visualize a URDF:

1. Publish the URDF to the `/robot_description` parameter
2. Run the `robot_state_publisher` node
3. Open RViz2 and add a RobotModel display

### Commands to Visualize

```bash
# Terminal 1: Start robot_state_publisher with your URDF
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat humanoid_basic.urdf)"

# Terminal 2: Start joint_state_publisher for interactive joints
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 3: Start RViz2
ros2 run rviz2 rviz2
```

### RViz2 Configuration

In RViz2:

1. Set **Fixed Frame** to `base_link`
2. Click **Add** → **RobotModel**
3. Under RobotModel, set **Description Topic** to `/robot_description`

You should see your humanoid robot! Use the joint_state_publisher_gui sliders to move the joints.

### Code Example: Launch File for Visualization

```python
#!/usr/bin/env python3
"""Launch file for humanoid URDF visualization in RViz2."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for URDF visualization."""

    # Path to URDF file
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'humanoid_basic.urdf'
    )

    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint state publisher GUI (for interactive joint control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])
```

---

## Diagrams

![URDF Tree Structure](../diagrams/urdf-structure.svg)
*Figure 1: URDF represents robots as a tree of links connected by joints, with the base_link as the root.*

![Joint Types Visualization](../diagrams/joint-types.svg)
*Figure 2: Common joint types in URDF: fixed, revolute, continuous, and prismatic, each with different motion constraints.*

---

## Hardware Notes

> **Simulation vs. Real Hardware**
>
> | Consideration | Visualization (RViz) | Simulation (Gazebo) | Real Hardware |
> |---------------|---------------------|---------------------|---------------|
> | Inertial | Not required | **Required** | Must match reality |
> | Collision | Not required | **Required** | Must match reality |
> | Visual | For display | For display | Less important |
> | Accuracy | Approximate OK | Must be accurate | Must match CAD |
>
> **Tips for real robots:**
> - Export URDF from CAD when possible (SolidWorks, Fusion 360 have exporters)
> - Measure and verify joint limits on physical hardware
> - Include sensor mounts and cable routing in collision geometry
> - Test collision boundaries thoroughly before operating

---

## Summary

In this lesson, you learned:

- **URDF** is an XML format describing robot structure as links and joints
- **Links** are rigid bodies with visual, collision, and inertial properties
- **Joints** connect links with motion constraints (fixed, revolute, prismatic, etc.)
- **Visual geometry** is for display; **collision geometry** is for physics
- **RViz2** visualizes URDF models using robot_state_publisher

---

## AI-Assisted Learning

<details>
<summary>Ask the AI Assistant</summary>

### Conceptual Questions
- "What's the difference between URDF and XACRO? When should I use each?"
- "How do I calculate inertia values for my robot links?"

### Debugging Help
- "My robot model appears broken/disconnected in RViz. What should I check?"
- "RViz says 'No transform from base_link to world'. How do I fix this?"

### Extension Ideas
- "How would I add sensors (camera, LIDAR) to my URDF?"
- "Can I use URDF to describe a robot with wheels instead of legs?"

</details>

---

## Exercises

### Exercise 1: Add Legs (Medium)

**Description**: Extend the humanoid URDF to include legs.

**Tasks**:
1. Add left_thigh, left_shin, left_foot links
2. Add corresponding right leg links
3. Create hip, knee, and ankle joints with appropriate limits

**Acceptance Criteria**:
- [ ] Both legs visible in RViz2
- [ ] Hip joints allow forward/backward motion (pitch)
- [ ] Knee joints bend in correct direction (0 to ~2.5 radians)
- [ ] URDF loads without errors

### Exercise 2: URDF Validation (Easy)

**Description**: Validate and debug URDF files.

**Tasks**:
1. Use `check_urdf` tool to validate the humanoid URDF
2. Intentionally introduce an error (missing parent link)
3. Use the error message to find and fix the problem

**Commands**:
```bash
# Install urdf tools if needed
sudo apt install liburdfdom-tools

# Validate URDF
check_urdf humanoid_basic.urdf

# Visualize URDF structure
urdf_to_graphviz humanoid_basic.urdf
```

**Acceptance Criteria**:
- [ ] `check_urdf` reports no errors
- [ ] Can interpret error messages to fix issues
- [ ] Generated graph shows correct structure

### Exercise 3: Custom Robot (Challenge)

**Description**: Design a URDF for a different robot type.

**Tasks**:
1. Choose a robot: wheeled base, robotic arm, or quadruped
2. Sketch the kinematic structure (links and joints)
3. Implement in URDF
4. Visualize in RViz2

**Acceptance Criteria**:
- [ ] URDF validates with no errors
- [ ] All joints have appropriate types and limits
- [ ] Robot is visualizable in RViz2
- [ ] Document your design decisions

---

## Navigation

| Previous | Up | Next |
|----------|-----|------|
| [I2: Python ROS Bridge (rclpy)](../intermediate/02-python-ros-bridge.md) | [Chapter 1 Home](../README.md) | [A2: Advanced ROS 2 Patterns & AI Integration](./02-advanced-patterns.md) |

---

## Next Steps

Continue to [A2: Advanced ROS 2 Patterns & AI Integration](./02-advanced-patterns.md) to learn about action servers, feedback mechanisms, and how AI agents can interact with ROS 2.
