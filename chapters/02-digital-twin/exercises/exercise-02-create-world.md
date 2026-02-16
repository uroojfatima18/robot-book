# Exercise 02: Create World

> Build a custom simulation world from scratch.

**Tier**: Intermediate
**Time**: 45-60 minutes
**Prerequisites**: Lessons I1 and I2 completed

---

## Objective

In this exercise, you will:
1. Create a custom world file with specific requirements
2. Add physics, lighting, and obstacles
3. Spawn and test your humanoid in the new world
4. Verify ROS 2 integration

---

## The Challenge

Create a **Robot Testing Arena** with these specifications:

### Required Elements

| Element | Specification |
|---------|---------------|
| Arena Size | 8m x 8m floor area |
| Ground | High-friction surface (mu=1.2) |
| Walls | At least 2 walls for boundary |
| Obstacles | Minimum 3 obstacles of different shapes |
| Lighting | Sun + at least 1 point light |
| Spawn Point | Marked center area for robot |

### Performance Target

- RTF >= 0.8 when world is loaded
- ROS 2 topics available

---

## Part 1: Create the World File

### Task 1.1: Set Up File Structure

Create a new file `robot_arena.world` in your workspace:

```bash
mkdir -p ~/ros2_ws/src/my_worlds
cd ~/ros2_ws/src/my_worlds
touch robot_arena.world
```

### Task 1.2: Add Base Structure

Start with this template:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="robot_arena">

    <!-- TODO: Add physics configuration -->

    <!-- TODO: Add scene settings -->

    <!-- TODO: Add lighting -->

    <!-- TODO: Add ground plane -->

    <!-- TODO: Add walls -->

    <!-- TODO: Add obstacles -->

    <!-- TODO: Add ROS 2 plugin -->

  </world>
</sdf>
```

### Task 1.3: Configure Physics

Add physics with these parameters:
- Update rate: 1000 Hz
- Step size: 0.001 s
- Solver iterations: 50

**Your code here:**

```xml
<!-- Replace this with your physics configuration -->
```

### Task 1.4: Add Lighting

Requirements:
- Directional sun light with shadows
- One point light in the center of the arena

**Your code here:**

```xml
<!-- Replace this with your lighting configuration -->
```

---

## Part 2: Build the Arena

### Task 2.1: Ground Plane

Create a ground plane with:
- Size: 10m x 10m (larger than arena for margin)
- High friction: mu=1.2, mu2=1.2
- Light gray color

**Checkpoint**: Ground appears when you launch the world.

### Task 2.2: Boundary Walls

Add at least 2 walls:
- Height: 1.5m
- Thickness: 0.2m
- Color: White or light gray

**Checkpoint**: Walls visible and solid (test by throwing object at them).

### Task 2.3: Obstacles

Add exactly 3 obstacles:

1. **Red Box**: 0.5m x 0.5m x 0.5m at position (2, 1, 0.25)
2. **Green Cylinder**: radius 0.3m, height 1m at position (-2, -1, 0.5)
3. **Blue Sphere**: radius 0.4m at position (0, 2, 0.4) - this should be **dynamic**

**Checkpoint**: All three obstacles visible with correct colors.

### Task 2.4: Spawn Point Marker

Add a visual-only marker at the center:
- Thin cylinder (radius 0.5m, height 0.01m)
- Blue color
- Position: (0, 0, 0.005)

**Checkpoint**: Blue circle visible at world center.

---

## Part 3: Test Your World

### Task 3.1: Launch and Verify

```bash
source /opt/ros/humble/setup.bash
gazebo --verbose robot_arena.world
```

**Verification Checklist**:
- [ ] World loads without errors
- [ ] Ground plane visible
- [ ] Walls visible
- [ ] All 3 obstacles present
- [ ] Blue sphere falls (dynamic object)
- [ ] Center marker visible

### Task 3.2: Check RTF

```bash
gz stats
```

| Metric | Your Value | Target |
|--------|------------|--------|
| RTF | _______ | >= 0.8 |

If RTF < 0.8, simplify your world (reduce wall complexity, use simpler shapes).

### Task 3.3: Verify ROS 2 Topics

```bash
ros2 topic list
```

**Required topics present**:
- [ ] `/clock`
- [ ] `/gazebo/model_states`

---

## Part 4: Spawn Your Humanoid

### Task 4.1: Spawn at Center

```bash
ros2 run gazebo_ros spawn_entity.py \
  -entity humanoid \
  -file /path/to/your/humanoid.urdf \
  -x 0 -y 0 -z 1.0
```

**Checkpoint**: Humanoid appears at center marker.

### Task 4.2: Verify Joint States

```bash
ros2 topic echo /joint_states --once
```

**Checkpoint**: Joint names and positions displayed.

---

## Part 5: Documentation

### Task 5.1: World File Header

Add a comment header to your world file:

```xml
<!--
  Robot Testing Arena - Exercise 02

  Author: [Your Name]
  Date: [Today's Date]
  Purpose: [Brief description]

  Features:
  - [List features]

  Usage:
    gazebo --verbose robot_arena.world
-->
```

### Task 5.2: Reflection Questions

1. **What physics parameters affect RTF the most?**

   Your answer: _______________________________________________

2. **Why does the blue sphere fall but the red box doesn't?**

   Your answer: _______________________________________________

3. **How would you add a second floor/platform?**

   Your answer: _______________________________________________

---

## Completion Criteria

You have successfully completed this exercise when:

- [ ] World file created with all required elements
- [ ] RTF >= 0.8 when loaded
- [ ] Dynamic object (sphere) falls correctly
- [ ] ROS 2 topics available
- [ ] Humanoid spawns at center
- [ ] Documentation header added
- [ ] Reflection questions answered

---

## Bonus Challenges

### Challenge A: Friction Zones

Create two distinct floor areas:
- Ice zone (mu=0.1) on the left side
- Rubber zone (mu=2.0) on the right side

Test by rolling the blue sphere across both.

### Challenge B: Inclined Ramp

Add a ramp leading up to a platform:
- Ramp angle: 15 degrees
- Platform height: 0.5m
- Test if humanoid can walk up it

### Challenge C: Sensor Zones

Add visual markers for sensor testing:
- Camera FOV cone visualization
- Lidar scan area circle

---

## Common Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Missing `<static>true</static>` | Objects fall through floor | Add static tag |
| Invalid inertia | Object explodes | Calculate correct inertia for mass |
| Wrong pose format | Object at wrong position | Check x,y,z,roll,pitch,yaw order |
| Missing collision | Objects pass through each other | Add collision geometry |

---

## Reference Solution

After completing, compare with the reference in:
`chapters/02-digital-twin/intermediate/assets/simple_lab.world`

---

## Next Steps

After completing this exercise:
- Experiment with different physics parameters
- Try importing mesh files for custom obstacles
- Proceed to [A1: Digital Twin Architecture](../advanced/A1-data-synchronization.md)

---

| Previous | Up | Next |
|----------|-----|------|
| [I2: Spawning Models](../intermediate/I2-spawning-models.md) | [Exercises](../README.md#exercises) | [Exercise 03: Build Bridge](exercise-03-build-bridge.md) |
