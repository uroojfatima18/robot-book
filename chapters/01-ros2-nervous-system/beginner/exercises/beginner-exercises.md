---
id: beginner_exercises
title: "Beginner Tier Exercises"
tier: beginner
chapter: chapter_1_ros2
---

# Beginner Tier Exercises

This document consolidates all exercises from the Beginner tier lessons for easy reference and self-assessment.

---

## From B1: Introduction to ROS 2

### Exercise B1.1: Explore ROS 2 Commands (Easy)

**Objective**: Become familiar with the ROS 2 CLI while the talker/listener demo is running.

**Setup**:
1. Open three terminals
2. In Terminal 1: `ros2 run demo_nodes_cpp talker`
3. In Terminal 2: `ros2 run demo_nodes_cpp listener`
4. Use Terminal 3 for exploration

**Tasks**:

1. List all running nodes:
   ```bash
   ros2 node list
   ```
   **Expected**: You should see `/talker` and `/listener`

2. Get detailed information about the talker node:
   ```bash
   ros2 node info /talker
   ```
   **Question**: What topics does the talker publish to?

3. Find what topics the listener subscribes to:
   ```bash
   ros2 node info /listener
   ```
   **Question**: What is the message type used on `/chatter`?

4. Measure the publishing rate:
   ```bash
   ros2 topic hz /chatter
   ```
   **Question**: How many messages per second is the talker publishing?

**Acceptance Criteria**:
- [ ] You can list all running nodes
- [ ] You can explain what the `/chatter` topic is used for
- [ ] You can identify the message type (`std_msgs/msg/String`)
- [ ] You know the default publishing rate (1 Hz)

<details>
<summary>Solution</summary>

1. `ros2 node list` shows `/talker` and `/listener`
2. The talker publishes to `/chatter` (and `/rosout`, `/parameter_events`)
3. The message type is `std_msgs/msg/String`
4. The default rate is approximately 1 Hz (1 message per second)

</details>

---

### Exercise B1.2: Change the Message Rate (Medium)

**Objective**: Learn how to modify node behavior through parameters.

**Tasks**:

1. Stop any running talker nodes (Ctrl+C)

2. Start the talker with a higher frequency:
   ```bash
   ros2 run demo_nodes_cpp talker --ros-args -p frequency:=5.0
   ```

3. Observe the faster output in the talker terminal

4. Verify the new rate:
   ```bash
   ros2 topic hz /chatter
   ```

5. Try different frequencies: 0.5, 2.0, 10.0

**Acceptance Criteria**:
- [ ] Messages appear 5 times per second instead of once
- [ ] `ros2 topic hz` confirms approximately 5 Hz
- [ ] You understand how `--ros-args -p` sets parameters

<details>
<summary>Solution</summary>

The `-p frequency:=5.0` parameter tells the talker to publish at 5 Hz. You should see:
- Output like "Publishing: 'Hello World: X'" appearing 5 times per second
- `ros2 topic hz /chatter` showing approximately `average rate: 5.000`

Note: Not all nodes support runtime parameter changes. The demo talker reads this parameter at startup.

</details>

---

### Exercise B1.3: Create Your Own Publisher (Challenge)

**Objective**: Run a Python version of the talker and explore differences.

**Tasks**:

1. Run the Python talker instead:
   ```bash
   ros2 run demo_nodes_py talker
   ```

2. Compare with the C++ version—do they publish to the same topic?

3. Run both talkers simultaneously and observe what happens at the listener

**Questions**:
- Can multiple publishers share the same topic?
- What happens to the message count from each publisher?

**Acceptance Criteria**:
- [ ] Both C++ and Python talkers can run
- [ ] You understand that multiple publishers can share a topic
- [ ] The listener receives messages from both

---

## From B2: Basic Sensors Overview

### Exercise B2.1: Sensor Identification (Easy)

**Objective**: Match robot tasks to appropriate sensors.

**Tasks**:
For each task below, identify which sensor would be **most useful**:

| Task | Your Answer |
|------|-------------|
| 1. Detect a door 2 meters ahead | |
| 2. Know if the robot is tilting backward | |
| 3. Pick up an egg without breaking it | |
| 4. Recognize a person's face | |
| 5. Navigate through a cluttered room | |
| 6. Detect when the robot is falling | |
| 7. Find the center of pressure under feet | |
| 8. Read text on a sign | |

**Acceptance Criteria**:
- [ ] You can justify each sensor choice
- [ ] You understand the primary use case for each sensor type

<details>
<summary>Solution</summary>

1. **Door detection** → LIDAR or Camera (LIDAR for distance, camera for recognition)
2. **Tilting detection** → IMU (gyroscope measures angular velocity, accelerometer detects tilt)
3. **Egg grasping** → Force sensor (precise grip force control)
4. **Face recognition** → RGB Camera (visual processing)
5. **Room navigation** → LIDAR (obstacle detection, SLAM)
6. **Fall detection** → IMU (sudden acceleration changes)
7. **Center of pressure** → Foot force sensors
8. **Reading text** → RGB Camera (optical character recognition)

</details>

---

### Exercise B2.2: Message Exploration (Medium)

**Objective**: Use ROS 2 CLI to explore sensor message structures.

**Tasks**:

1. View the IMU message definition:
   ```bash
   ros2 interface show sensor_msgs/msg/Imu
   ```
   **Question**: What are the three main measurement fields?

2. View the LaserScan message definition:
   ```bash
   ros2 interface show sensor_msgs/msg/LaserScan
   ```
   **Question**: What does the `ranges` array contain?

3. Find the message type for 3D point clouds:
   ```bash
   ros2 interface list | grep -i point
   ```

4. Examine the Image message:
   ```bash
   ros2 interface show sensor_msgs/msg/Image
   ```
   **Question**: What field specifies the pixel format?

**Acceptance Criteria**:
- [ ] You can describe IMU message structure (orientation, angular_velocity, linear_acceleration)
- [ ] You understand LaserScan ranges (distance measurements in meters)
- [ ] You found `sensor_msgs/msg/PointCloud2`
- [ ] You know Image encoding field specifies pixel format

<details>
<summary>Solution</summary>

1. **IMU fields**: `orientation` (quaternion), `angular_velocity` (rad/s), `linear_acceleration` (m/s²)
2. **LaserScan ranges**: Array of distance measurements in meters, one per angle increment
3. **Point cloud**: `sensor_msgs/msg/PointCloud2`
4. **Image encoding**: The `encoding` field (e.g., "rgb8", "bgr8", "mono8", "16UC1")

</details>

---

### Exercise B2.3: Sensor Data Interpretation (Challenge)

**Objective**: Interpret sensor readings and understand their meaning.

**Scenario**: A robot's IMU reports these values:
```
linear_acceleration:
  x: 0.2
  y: -0.1
  z: 9.7
angular_velocity:
  x: 0.01
  y: 0.02
  z: 0.0
```

**Questions**:

1. Why is `z` acceleration approximately 9.8 m/s²?
2. Is the robot rotating significantly? How do you know?
3. What would `z` acceleration show if the robot was in free fall?
4. If `y` angular velocity increased to 1.0 rad/s, what motion would that indicate?

**Acceptance Criteria**:
- [ ] You understand gravity's effect on accelerometer readings
- [ ] You can interpret angular velocity values
- [ ] You know what free fall would look like in IMU data

<details>
<summary>Solution</summary>

1. **Z ≈ 9.8 m/s²**: This is gravity! Accelerometers measure all forces including gravity. A stationary robot shows ~9.8 m/s² in the direction opposite to gravity.

2. **Rotation**: Angular velocities are very small (0.01-0.02 rad/s ≈ 0.5-1°/s), so the robot is nearly stationary rotationally.

3. **Free fall**: All acceleration components would be approximately 0 (the robot is accelerating with gravity, so no force is felt).

4. **Y = 1.0 rad/s**: This is about 57°/s rotation around the Y axis, which typically means the robot is pitching (tilting forward/backward) rapidly.

</details>

---

### Exercise B2.4: LIDAR Data Analysis (Challenge)

**Objective**: Process a simulated LIDAR scan.

**Given Data**:
```python
# Simulated LaserScan data
angle_min = -1.57  # -90 degrees in radians
angle_max = 1.57   # +90 degrees in radians
angle_increment = 0.0175  # ~1 degree
range_min = 0.1
range_max = 10.0
ranges = [2.5, 2.4, 2.3, 2.2, 2.1, 2.0, 1.8, 1.5, 1.2, 1.0,
          0.8, 0.7, 0.6, 0.5, 0.5, 0.5, 0.6, 0.7, 0.8, 1.0,
          1.2, 1.5, 1.8, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6]
```

**Tasks**:

1. How many readings are in this scan?
2. What is the closest obstacle distance?
3. At approximately what angle is the closest obstacle?
4. What shape might this obstacle be? (Hint: look at the pattern)

**Acceptance Criteria**:
- [ ] Correct reading count
- [ ] Identified minimum distance
- [ ] Calculated approximate angle of closest point
- [ ] Made a reasonable guess about obstacle shape

<details>
<summary>Solution</summary>

1. **Reading count**: 30 readings

2. **Closest distance**: 0.5 meters (appears 3 times)

3. **Angle calculation**:
   - Readings 13, 14, 15 show 0.5m
   - Middle point (14) is at index 14
   - Angle = angle_min + index × angle_increment
   - Angle = -1.57 + 14 × 0.0175 = -1.325 rad ≈ -76° (to the right-front)

4. **Obstacle shape**: The pattern (decreasing from sides, minimum in middle, then increasing) suggests a curved or circular obstacle like a pillar or a person standing in front of the robot.

</details>

---

## Self-Assessment Checklist

After completing all beginner exercises, verify you can:

### ROS 2 Fundamentals
- [ ] Run the talker/listener demo
- [ ] Use `ros2 node list` and `ros2 topic list`
- [ ] Echo messages from a topic
- [ ] Get information about nodes and topics
- [ ] Modify node parameters at launch

### Sensor Understanding
- [ ] Explain what each sensor type measures (IMU, LIDAR, Camera, Force)
- [ ] Identify ROS 2 message types for each sensor
- [ ] Interpret basic sensor readings
- [ ] Match sensors to appropriate robot tasks

### Ready for Intermediate?
If you checked all boxes above, you're ready to proceed to the Intermediate tier where you'll write your own ROS 2 Python nodes!

---

## Next Steps

Continue to [Intermediate Tier: Nodes, Topics, Services, and Actions](../../intermediate/01-nodes-topics.md)
