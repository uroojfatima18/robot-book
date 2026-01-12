# Exercise 01: Launch World

> Practice launching Gazebo and exploring a simulation world.

**Tier**: Beginner
**Time**: 30-45 minutes
**Prerequisites**: Lessons B1 and B2 completed

---

## Objective

In this exercise, you will:
1. Launch the humanoid lab world in Gazebo
2. Navigate the 3D environment
3. Monitor simulation performance
4. Interact with ROS 2 topics

---

## Part 1: Launch the Simulation

### Task 1.1: Basic Launch

Open a terminal and run:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Navigate to chapter assets
cd ~/ros2_ws/src  # or your book repository location

# Launch Gazebo with the humanoid lab
gazebo --verbose humanoid_lab.world
```

**Checkpoint**: Gazebo window opens with laboratory environment.

### Task 1.2: Verify Physics

In the Gazebo window, check the bottom status bar:
- [ ] Time is incrementing
- [ ] RTF displays a value (ideally >= 0.8)
- [ ] No red error messages in terminal

---

## Part 2: Navigation Challenge

### Task 2.1: View the Robot from All Angles

Using mouse controls, position the camera to see:

1. **Front view**: Face the humanoid head-on
   - [ ] Screenshot or describe what you see

2. **Top view**: Look down at the robot from above
   - [ ] Screenshot or describe what you see

3. **Close-up**: Zoom in on the humanoid's head
   - [ ] Screenshot or describe what you see

### Task 2.2: Explore the Lab

Navigate to find:
- [ ] The lab table with monitor
- [ ] All four walls
- [ ] The humanoid model

**Question**: What color is the humanoid's torso?

---

## Part 3: Performance Monitoring

### Task 3.1: Check RTF

Open a new terminal (keep Gazebo running):

```bash
# Check Gazebo statistics
gz stats
```

Record the values:
- Real-Time Factor: ______
- Sim Time: ______
- Real Time: ______

### Task 3.2: Performance Analysis

| RTF Value | Status |
|-----------|--------|
| Your RTF: | ______ |
| Target: | >= 0.8 |
| Pass/Fail: | ______ |

If RTF < 0.8, try:
```bash
# Kill current Gazebo
killall gzserver gzclient

# Launch headless (no graphics)
gzserver humanoid_lab.world &

# Attach lighter client
gzclient
```

---

## Part 4: ROS 2 Integration

### Task 4.1: List Topics

Open a new terminal:

```bash
source /opt/ros/humble/setup.bash

# List all available topics
ros2 topic list
```

**Record the topics you see**:
1. ______
2. ______
3. ______
4. ______

### Task 4.2: Echo Gazebo State

```bash
# Echo model states
ros2 topic echo /gazebo/model_states --once
```

**Question**: What models are listed in the output?

### Task 4.3: Check Clock

```bash
# Watch simulation clock
ros2 topic echo /clock
```

**Observation**: Is the simulation time incrementing? (Ctrl+C to stop)

---

## Part 5: Interactive Exploration

### Task 5.1: Pause and Resume

In Gazebo:
1. Press `Space` to pause
2. Check the terminal - time should stop
3. Press `Space` to resume
4. Verify time continues

- [ ] Pause/resume works correctly

### Task 5.2: Reset the World

From terminal:

```bash
# Reset simulation to initial state
ros2 service call /reset_simulation std_srvs/srv/Empty
```

**Observation**: Did the simulation reset?

---

## Reflection Questions

Answer these questions to solidify your understanding:

1. **What is the purpose of RTF in simulation?**

   Your answer: _______________________________________________

2. **Why do we use `--verbose` when launching Gazebo?**

   Your answer: _______________________________________________

3. **What ROS 2 topics did Gazebo automatically create?**

   Your answer: _______________________________________________

4. **How would a digital twin use these topics?**

   Your answer: _______________________________________________

---

## Troubleshooting Checklist

If you encountered issues, check these:

- [ ] ROS 2 Humble sourced in every terminal
- [ ] Gazebo installed correctly (`gazebo --version` shows 11.x)
- [ ] No other Gazebo instances running
- [ ] World file path is correct
- [ ] Graphics drivers working (try software rendering if needed)

---

## Completion Criteria

You have successfully completed this exercise when:

- [ ] Gazebo launches with humanoid_lab.world
- [ ] RTF is monitored and >= 0.8 (or headless workaround used)
- [ ] All ROS 2 topics listed and explored
- [ ] Pause/resume functionality tested
- [ ] Reflection questions answered

---

## Bonus Challenges

### Challenge A: Custom View

Create a custom camera view and save it using Gazebo's View menu.

### Challenge B: Terminal Monitoring

Create a script that continuously monitors RTF and prints a warning if it drops below 0.8:

```bash
#!/bin/bash
# rtf_monitor.sh
while true; do
    rtf=$(gz stats -p | grep "factor" | awk '{print $2}')
    if (( $(echo "$rtf < 0.8" | bc -l) )); then
        echo "WARNING: RTF dropped to $rtf"
    fi
    sleep 1
done
```

### Challenge C: Screenshot Automation

Use Gazebo's built-in screenshot feature or create a ROS 2 node that captures the camera view.

---

## Next Steps

After completing this exercise:
- Review B1 and B2 lessons if any concepts were unclear
- Proceed to [Exercise 02: Create World](exercise-02-create-world.md) (Intermediate)
- Or explore the [Intermediate Tier](../intermediate/I1-building-worlds.md)

---

| Previous | Up | Next |
|----------|-----|------|
| [B2: First Simulation](../beginner/B2-first-simulation.md) | [Exercises](../README.md#exercises) | [Exercise 02: Create World](exercise-02-create-world.md) |
