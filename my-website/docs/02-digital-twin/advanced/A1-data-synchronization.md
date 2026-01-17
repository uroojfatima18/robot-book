---
id: chapter_2_lesson_a1
title: "A1: Digital Twin Architecture"
sidebar_position: 18
chapter: chapter_2_digital_twin
tier: advanced
---

# A1: Digital Twin Architecture

> Design data synchronization patterns for bidirectional robot-simulation communication.

## Learning Objectives

By the end of this lesson, you will be able to:
- Design a digital twin data architecture
- Identify synchronization patterns and their tradeoffs
- Plan topic mappings between physical and virtual robots
- Understand latency requirements for real-time sync
- Handle edge cases in bidirectional communication

---

## Prerequisites

- Completed Intermediate tier (I1, I2)
- Understanding of ROS 2 topics and services
- Experience with joint control

---

## Introduction

In the Beginner and Intermediate tiers, you worked with simulation in isolation. Now we bridge the gap to reality. A **Digital Twin Bridge** enables:

1. **Physical → Virtual**: Real robot state updates the simulation
2. **Virtual → Physical**: Simulation commands control the real robot
3. **Monitoring**: Track latency and detect synchronization issues

This is the foundation for testing algorithms in simulation before deploying to hardware.

---

## Architecture Overview

### High-Level Design

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          DIGITAL TWIN SYSTEM                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────┐                        ┌─────────────────┐        │
│  │  PHYSICAL ROBOT │                        │    GAZEBO SIM   │        │
│  │                 │                        │                 │        │
│  │  /hw/joint_states ◄─────────────────────► /sim/joint_states│        │
│  │  /hw/cmd_vel     ◄─────────────────────► /sim/cmd_vel     │        │
│  │  /hw/imu         ◄─────────────────────► /sim/imu         │        │
│  │  /hw/camera      ◄─────────────────────► /sim/camera      │        │
│  │                 │                        │                 │        │
│  └────────┬────────┘                        └────────┬────────┘        │
│           │                                          │                 │
│           └─────────────────┬────────────────────────┘                 │
│                             │                                          │
│                    ┌────────┴────────┐                                 │
│                    │   BRIDGE NODE   │                                 │
│                    │                 │                                 │
│                    │  - Topic sync   │                                 │
│                    │  - Latency mon  │                                 │
│                    │  - Mode select  │                                 │
│                    │                 │                                 │
│                    └────────┬────────┘                                 │
│                             │                                          │
│                    ┌────────┴────────┐                                 │
│                    │ LATENCY MONITOR │                                 │
│                    │  Target: &lt;50ms  │                                 │
│                    └─────────────────┘                                 │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Component Responsibilities

| Component | Responsibility | Update Rate |
|-----------|---------------|-------------|
| Physical Robot | Real sensors and actuators | Hardware-dependent |
| Gazebo Sim | Physics and sensor simulation | 1000 Hz internal |
| Bridge Node | Bidirectional topic remapping | 100-500 Hz |
| Latency Monitor | Track sync delay, alert on threshold | 10 Hz |

---

## Synchronization Patterns

### Pattern 1: State Mirror

The simulation mirrors the physical robot's state. Used for:
- Visualization
- Logging
- External monitoring

```
Physical Robot               Bridge               Gazebo
     │                         │                    │
     │──/hw/joint_states──────►│                    │
     │                         │──/sim/joint_states─►│
     │                         │                    │
     │                         │◄──/sim/pose────────│
     │                         │  (for visualization)
```

**Implementation**:
```python
# Mirror state from physical to simulation
def joint_state_callback(self, msg):
    # Remap topic namespace
    mirrored = JointState()
    mirrored.header = msg.header
    mirrored.name = msg.name
    mirrored.position = msg.position
    mirrored.velocity = msg.velocity

    self.sim_joint_pub.publish(mirrored)
```

### Pattern 2: Command Forward

Commands tested in simulation are forwarded to hardware:

```
Controller                   Bridge               Physical Robot
     │                         │                        │
     │──/cmd/joint_trajectory─►│                        │
     │                         │──/hw/joint_trajectory─►│
     │                         │                        │
     │                         │◄──/hw/joint_states─────│
     │◄──/cmd/feedback─────────│                        │
```

**Safety Gate**:
```python
def forward_command(self, msg):
    # Safety check before forwarding
    if not self.safety_check(msg):
        self.get_logger().warn('Command blocked by safety gate')
        return

    if self.mode == 'live':
        self.hw_cmd_pub.publish(msg)
    else:
        self.sim_cmd_pub.publish(msg)
```

### Pattern 3: Bidirectional Sync

Full synchronization for digital twin loop:

```
                    ┌──────────────────┐
                    │                  │
Physical ──────────►│   SYNC ENGINE    │◄────────── Simulation
 Robot   ◄──────────│                  │──────────►   Robot
                    │  Conflict Res.   │
                    │  Latency Track   │
                    └──────────────────┘
```

**Conflict Resolution**:
- Physical always wins for sensor data
- Simulation wins for predicted trajectories
- Configurable authority per topic

---

## Topic Mapping Strategy

### Namespace Convention

Use clear namespaces to avoid topic collisions:

| Namespace | Purpose | Example Topics |
|-----------|---------|----------------|
| `/hw/` | Physical robot topics | `/hw/joint_states`, `/hw/imu` |
| `/sim/` | Simulation topics | `/sim/joint_states`, `/sim/imu` |
| `/cmd/` | Command topics (shared) | `/cmd/joint_trajectory` |
| `/bridge/` | Bridge internal | `/bridge/latency`, `/bridge/status` |

### Topic Mapping Table

| Physical Topic | Simulation Topic | Direction | Rate |
|----------------|------------------|-----------|------|
| `/hw/joint_states` | `/sim/joint_states` | hw → sim | 100 Hz |
| `/hw/imu/data` | `/sim/imu/data` | hw → sim | 200 Hz |
| `/hw/camera/image` | `/sim/camera/image` | hw → sim | 30 Hz |
| `/cmd/joint_trajectory` | Both | cmd → both | On-demand |
| `/bridge/latency` | N/A | internal | 10 Hz |

### Configuration File

Create `bridge_config.yaml`:

```yaml
bridge_node:
  ros__parameters:
    # Operation mode: 'sim', 'live', 'mirror'
    mode: 'mirror'

    # Latency threshold in milliseconds
    latency_threshold_ms: 50.0

    # Topic mappings
    topic_mappings:
      - physical: '/hw/joint_states'
        simulation: '/sim/joint_states'
        direction: 'physical_to_sim'
        rate_hz: 100.0

      - physical: '/hw/imu/data'
        simulation: '/sim/imu/data'
        direction: 'physical_to_sim'
        rate_hz: 200.0

      - physical: '/hw/joint_trajectory'
        simulation: '/sim/joint_trajectory'
        direction: 'command'
        rate_hz: 0  # On-demand

    # Safety limits
    safety:
      max_joint_velocity: 2.0  # rad/s
      max_joint_acceleration: 5.0  # rad/s^2
      emergency_stop_topic: '/hw/emergency_stop'
```

---

## Latency Requirements

### Why 50ms?

The 50ms latency threshold balances:

| Factor | Consideration |
|--------|---------------|
| Control Loop | 20 Hz control needs 50ms update cycle |
| Human Perception | Delays >100ms feel laggy |
| Safety Response | Emergency stops must propagate quickly |
| Network Overhead | WiFi adds 10-20ms, allowance for jitter |

### Latency Sources

```
Total Latency = Sensor + Network + Processing + Publish

Physical Robot        Bridge Node          Simulation
     │                    │                    │
     │ ──Sensor Read──►   │                    │
     │     (5ms)          │                    │
     │                    │                    │
     │ ════Network════►   │                    │
     │    (10-20ms)       │                    │
     │                    │                    │
     │                    │──Processing──►     │
     │                    │    (1-5ms)         │
     │                    │                    │
     │                    │    ──Publish──►    │
     │                    │      (1ms)         │
     │                    │                    │
     ◄═══════════════════════════════════════►
              Total: 17-31ms typical
```

### Latency Measurement

```python
def measure_latency(self, msg):
    """Calculate one-way latency from message timestamp."""
    msg_time = Time.from_msg(msg.header.stamp)
    now = self.get_clock().now()

    latency_ns = now.nanoseconds - msg_time.nanoseconds
    latency_ms = latency_ns / 1e6

    return latency_ms
```

---

## Edge Cases

### Network Disconnection

When physical robot loses connection:

```python
class ConnectionMonitor:
    def __init__(self):
        self.last_heartbeat = time.time()
        self.timeout_sec = 1.0  # 1 second timeout

    def check_connection(self):
        if time.time() - self.last_heartbeat > self.timeout_sec:
            self.handle_disconnect()

    def handle_disconnect(self):
        # 1. Stop forwarding commands
        self.mode = 'sim_only'

        # 2. Alert operator
        self.publish_alert('Physical robot disconnected')

        # 3. Simulation continues (for logging/analysis)
```

### Clock Skew

Physical and simulation clocks may drift:

```python
def sync_clocks(self, hw_time, sim_time):
    """Handle clock differences between systems."""
    drift = abs(hw_time - sim_time)

    if drift > 0.1:  # 100ms drift
        self.get_logger().warn(f'Clock drift: {drift:.3f}s')

    # Use simulation clock as reference when in sim mode
    if self.mode == 'sim':
        return sim_time
    else:
        return hw_time
```

### Sensor Failure

When a sensor stops publishing:

```python
class SensorWatchdog:
    def __init__(self, topic, timeout):
        self.topic = topic
        self.timeout = timeout
        self.last_msg = None

    def check(self):
        if self.last_msg is None:
            return 'no_data'

        age = time.time() - self.last_msg
        if age > self.timeout:
            return 'timeout'

        return 'ok'
```

---

## Design Checklist

Before implementing your bridge, verify:

### Data Flow
- [ ] All required topics identified
- [ ] Direction (physical→sim, sim→physical, bidirectional) defined
- [ ] Update rates specified
- [ ] QoS profiles match between publishers/subscribers

### Safety
- [ ] Emergency stop propagates in &lt;10ms
- [ ] Commands validated before forwarding
- [ ] Disconnection handling defined
- [ ] Fallback mode (sim-only) available

### Performance
- [ ] Latency budget allocated
- [ ] Rate limiting configured
- [ ] Resource usage acceptable (CPU, memory, bandwidth)

### Monitoring
- [ ] Latency published to monitoring topic
- [ ] Alerts configured for threshold violations
- [ ] Logging captures sync state

---

## What's Next?

In the next lesson, you'll implement the bridge node with all these patterns.

**Next**: [A2: Building the Bridge Node](A2-building-bridge.md)

---

## AI Agent Assisted Prompts

### Architecture Review
```
Review this digital twin architecture for a 24-DOF humanoid robot:
- Physical robot publishes joint_states at 500Hz
- Simulation runs at 1000Hz internal step
- WiFi network with 15ms average latency
- Target end-to-end latency: 30ms

Identify potential bottlenecks and suggest optimizations.
```

### Topic Mapping Design
```
Design a topic mapping strategy for a humanoid digital twin with:
- 24 joints (12 per leg, 6 per arm)
- IMU (400Hz)
- 2 cameras (30fps each)
- Force/torque sensors on feet (200Hz)

Include namespace conventions, QoS profiles, and bandwidth estimates.
```

### Failure Mode Analysis
```
Analyze failure modes for a digital twin bridge node:
1. What happens if simulation runs slower than real-time?
2. How should the bridge handle message queue overflow?
3. What's the safest behavior when latency exceeds threshold?

Provide detection strategies and recovery procedures for each.
```

---

## Summary

- Digital twin architecture requires clear physical/virtual separation
- Three patterns: State Mirror, Command Forward, Bidirectional Sync
- Use namespaces to organize topics (`/hw/`, `/sim/`, `/cmd/`)
- Target latency: &lt;50ms for reliable control
- Handle edge cases: disconnect, clock skew, sensor failure
- Always implement safety gates for live robot commands

---

| Previous | Up | Next |
|----------|-----|------|
| [I2: Spawning Models](../intermediate/I2-spawning-models.md) | [Advanced Tier](../README.md#advanced-tier) | [A2: Building the Bridge](A2-building-bridge.md) |
