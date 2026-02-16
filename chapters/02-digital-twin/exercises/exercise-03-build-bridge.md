# Exercise 03: Build Bridge

> Implement and test a digital twin bridge node.

**Tier**: Advanced
**Time**: 60-90 minutes
**Prerequisites**: Lessons A1 and A2 completed

---

## Objective

In this exercise, you will:
1. Deploy the bridge node in a test environment
2. Configure bidirectional synchronization
3. Measure and analyze latency
4. Handle edge cases and failures

---

## The Challenge

Create a working **Digital Twin Bridge** that meets these requirements:

### Performance Targets

| Metric | Target |
|--------|--------|
| Average Latency | < 30ms |
| P95 Latency | < 50ms |
| Violation Rate | < 5% |
| Connection Recovery | < 2s |

### Functional Requirements

- [ ] Mirror mode: Physical state appears in simulation
- [ ] Sim mode: Commands route to simulation only
- [ ] Live mode: Commands route to hardware with safety checks
- [ ] Latency monitoring with threshold alerts

---

## Part 1: Environment Setup

### Task 1.1: Launch Simulation

Open **Terminal 1**:

```bash
source /opt/ros/humble/setup.bash

# Launch Gazebo with your world
gazebo --verbose simple_lab.world
```

### Task 1.2: Start Mock Hardware Publisher

Open **Terminal 2** - create a mock hardware publisher to simulate physical robot data:

```bash
source /opt/ros/humble/setup.bash

# Publish fake joint states at 100Hz
ros2 topic pub /hw/joint_states sensor_msgs/msg/JointState \
  "{header: {stamp: {sec: 0, nanosec: 0}}, name: ['left_knee', 'right_knee'], position: [0.5, 0.5]}" \
  --rate 100
```

### Task 1.3: Verify Topics

Open **Terminal 3**:

```bash
source /opt/ros/humble/setup.bash

# Check topics exist
ros2 topic list | grep -E "(hw|sim)"
```

**Checkpoint**: See both `/hw/joint_states` and simulation topics.

---

## Part 2: Deploy the Bridge

### Task 2.1: Create Package Structure

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python digital_twin_bridge

# Copy bridge files
cp /path/to/book/chapters/02-digital-twin/advanced/src/bridge_node.py \
   digital_twin_bridge/digital_twin_bridge/

cp /path/to/book/chapters/02-digital-twin/advanced/src/latency_monitor.py \
   digital_twin_bridge/digital_twin_bridge/
```

### Task 2.2: Update setup.py

Edit `digital_twin_bridge/setup.py`:

```python
entry_points={
    'console_scripts': [
        'bridge_node = digital_twin_bridge.bridge_node:main',
        'latency_monitor = digital_twin_bridge.latency_monitor:main',
    ],
},
```

### Task 2.3: Build and Source

```bash
cd ~/ros2_ws
colcon build --packages-select digital_twin_bridge
source install/setup.bash
```

**Checkpoint**: Package builds without errors.

---

## Part 3: Run the Bridge

### Task 3.1: Start Bridge in Mirror Mode

Open **Terminal 4**:

```bash
source ~/ros2_ws/install/setup.bash

ros2 run digital_twin_bridge bridge_node --ros-args \
  -p mode:=mirror \
  -p latency_threshold_ms:=50.0
```

**Checkpoint**: Bridge starts and reports "initialized in mirror mode".

### Task 3.2: Start Latency Monitor

Open **Terminal 5**:

```bash
source ~/ros2_ws/install/setup.bash

ros2 run digital_twin_bridge latency_monitor --ros-args \
  -p threshold_ms:=50.0 \
  -p report_interval:=5.0
```

**Checkpoint**: Monitor starts reporting statistics.

### Task 3.3: Verify Mirroring

```bash
# Check mirrored topic
ros2 topic echo /sim/joint_states_mirrored --once
```

**Checkpoint**: Joint states from hardware appear on mirrored topic.

---

## Part 4: Latency Analysis

### Task 4.1: Baseline Measurement

Let the system run for 60 seconds and record:

| Metric | Your Value | Target |
|--------|------------|--------|
| Average Latency | _______ ms | < 30ms |
| Max Latency | _______ ms | < 100ms |
| P95 Latency | _______ ms | < 50ms |
| Violations | _______ | < 5% |

### Task 4.2: Echo Latency Topic

```bash
ros2 topic echo /bridge/latency
```

Record 10 consecutive readings:
1. _______ ms
2. _______ ms
3. _______ ms
4. _______ ms
5. _______ ms
6. _______ ms
7. _______ ms
8. _______ ms
9. _______ ms
10. _______ ms

### Task 4.3: Check Bridge Status

```bash
ros2 topic echo /bridge/status --once
```

Record the status:
- Mode: _______
- HW Connected: _______
- Sim Connected: _______
- Latency: _______ ms

---

## Part 5: Mode Testing

### Task 5.1: Test Sim Mode

Change to simulation mode:

```bash
ros2 param set /bridge_node mode "sim"
```

Send a test command:

```bash
ros2 topic pub --once /cmd/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['left_knee'], points: [{positions: [0.8], time_from_start: {sec: 1}}]}"
```

Verify command reaches simulation:

```bash
ros2 topic echo /sim/joint_trajectory --once
```

**Checkpoint**: Command appears on `/sim/joint_trajectory`.

### Task 5.2: Test Safety Rejection

Send an invalid command (position exceeds limit):

```bash
ros2 topic pub --once /cmd/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['left_knee'], points: [{positions: [5.0], time_from_start: {sec: 1}}]}"
```

**Expected**: Bridge logs "Command rejected by safety check".

### Task 5.3: Test Empty Command

```bash
ros2 topic pub --once /cmd/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [], points: []}"
```

**Expected**: Bridge logs "Empty trajectory rejected" or "No joint names".

---

## Part 6: Failure Injection

### Task 6.1: Simulate Hardware Disconnect

Stop the mock hardware publisher (Terminal 2).

Wait 3 seconds, then check:

```bash
ros2 topic echo /bridge/status --once
```

**Expected**: `hw_connected: False` and log message "Hardware connection lost".

### Task 6.2: Reconnect Hardware

Restart the mock publisher (Terminal 2).

Check status after 5 seconds:

**Expected**: `hw_connected: True`.

### Task 6.3: Induce High Latency

Modify the mock publisher to add artificial delay:

```python
# Create delay_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class DelayPublisher(Node):
    def __init__(self):
        super().__init__('delay_publisher')
        self.pub = self.create_publisher(JointState, '/hw/joint_states', 10)
        self.timer = self.create_timer(0.01, self.publish)  # 100Hz

    def publish(self):
        msg = JointState()
        # Set timestamp 60ms in the past to simulate latency
        now = self.get_clock().now().to_msg()
        msg.header.stamp.sec = now.sec
        msg.header.stamp.nanosec = max(0, now.nanosec - 60_000_000)  # 60ms ago
        msg.name = ['left_knee', 'right_knee']
        msg.position = [0.5, 0.5]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = DelayPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run the delay publisher and observe:

**Expected**: Latency monitor shows violations and warnings.

---

## Part 7: Documentation

### Task 7.1: Capture Final Stats

After all tests, record the latency monitor's summary:

```
Runtime: _______ seconds
Total Samples: _______
Average Latency: _______ ms
Max Latency: _______ ms
Violations: _______ (_______%)
Max Consecutive: _______
Meets Threshold: _______
```

### Task 7.2: Reflection Questions

1. **What was your typical latency range?**

   Your answer: _______________________________________________

2. **What happened when hardware disconnected?**

   Your answer: _______________________________________________

3. **How would you reduce latency further?**

   Your answer: _______________________________________________

4. **What additional safety checks would you add for a real robot?**

   Your answer: _______________________________________________

---

## Completion Criteria

You have successfully completed this exercise when:

- [ ] Bridge runs in all three modes (sim, live, mirror)
- [ ] Average latency < 30ms achieved
- [ ] Safety checks reject invalid commands
- [ ] Disconnect detected within 2 seconds
- [ ] Reconnect detected automatically
- [ ] Latency monitor reports accurate statistics
- [ ] Reflection questions answered

---

## Bonus Challenges

### Challenge A: Custom QoS

Experiment with different QoS settings:

```python
# Try different reliability policies
reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE)
best_effort_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT)
```

Measure how QoS affects latency and reliability.

### Challenge B: Multi-Robot Bridge

Extend the bridge to handle multiple robots:
- `/robot1/hw/joint_states` → `/robot1/sim/joint_states`
- `/robot2/hw/joint_states` → `/robot2/sim/joint_states`

### Challenge C: Recording and Playback

Use `ros2 bag` to record bridge data:

```bash
ros2 bag record /hw/joint_states /sim/joint_states_mirrored /bridge/latency
```

Analyze the recording to find latency patterns.

### Challenge D: Visualization

Create a simple visualization that shows:
- Real-time latency graph
- Connection status indicators
- Mode display

---

## Troubleshooting

### Bridge Not Receiving Messages

```bash
# Check topic types match
ros2 topic info /hw/joint_states --verbose
ros2 topic info /sim/joint_states --verbose
```

### High Latency

| Cause | Solution |
|-------|----------|
| Network congestion | Use wired connection |
| CPU overload | Reduce publish rate |
| Python GC pauses | Use C++ for production |
| QoS mismatch | Align publisher/subscriber QoS |

### Mode Not Changing

```bash
# Verify parameter is set
ros2 param get /bridge_node mode

# List all parameters
ros2 param list /bridge_node
```

---

## Next Steps

After completing this exercise:
- Review your latency measurements for optimization opportunities
- Consider extending the bridge with additional sensors
- Explore the AI training integration in A2 lesson
- Apply these patterns to your own robot project

---

| Previous | Up | Next |
|----------|-----|------|
| [A2: Building Bridge](../advanced/A2-building-bridge.md) | [Exercises](../README.md#exercises) | [Chapter 3](../../03-perception/README.md) |
