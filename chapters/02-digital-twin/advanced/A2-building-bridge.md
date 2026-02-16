# A2: Building the Bridge Node

> Implement a bidirectional synchronization bridge between physical and simulated robots.

## Learning Objectives

By the end of this lesson, you will be able to:
- Implement a ROS 2 bridge node for digital twin sync
- Handle bidirectional topic remapping
- Monitor and report latency
- Configure operation modes (sim, live, mirror)
- Integrate safety checks for live robot commands

---

## Prerequisites

- Completed A1 (Architecture design)
- Understanding of ROS 2 node lifecycle
- Experience with multi-threaded Python

---

## Introduction

Now we implement the architecture from A1. The bridge node is the heart of the digital twin system—it must be:

- **Reliable**: Handle network issues gracefully
- **Fast**: Minimize latency overhead
- **Safe**: Validate commands before forwarding
- **Observable**: Report status and metrics

---

## Bridge Node Implementation

### Node Structure

```
BridgeNode
├── Subscribers
│   ├── /hw/joint_states
│   ├── /sim/joint_states
│   └── /cmd/joint_trajectory
├── Publishers
│   ├── /sim/joint_states (mirrored)
│   ├── /hw/joint_trajectory (forwarded)
│   └── /bridge/latency
├── Services
│   ├── /bridge/set_mode
│   └── /bridge/get_status
└── Timers
    ├── latency_check (10 Hz)
    └── watchdog (1 Hz)
```

### Complete Implementation

Create `bridge_node.py`:

```python
#!/usr/bin/env python3
"""
Digital Twin Bridge Node - Chapter 2 Advanced

Bidirectional synchronization between physical and simulated robots.

Modes:
    - sim: Commands go to simulation only
    - live: Commands go to physical robot only
    - mirror: Physical state mirrors to simulation

Topics Subscribed:
    /hw/joint_states - Physical robot state
    /sim/joint_states - Simulation state
    /cmd/joint_trajectory - Incoming commands

Topics Published:
    /sim/joint_states - Mirrored state (in mirror mode)
    /hw/joint_trajectory - Forwarded commands (in live mode)
    /bridge/latency - Latency metrics

Services:
    /bridge/set_mode - Change operation mode
    /bridge/get_status - Get current status

Author: Robot Book Chapter 2
"""

import threading
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Dict, List
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool
from rcl_interfaces.msg import ParameterDescriptor


class BridgeMode(Enum):
    """Operation modes for the bridge."""
    SIM = 'sim'           # Commands to simulation only
    LIVE = 'live'         # Commands to physical robot
    MIRROR = 'mirror'     # Physical mirrors to simulation


@dataclass
class LatencyStats:
    """Latency measurement statistics."""
    current_ms: float = 0.0
    average_ms: float = 0.0
    max_ms: float = 0.0
    samples: int = 0
    threshold_violations: int = 0


class BridgeNode(Node):
    """
    Digital Twin Bridge Node.

    Handles bidirectional synchronization between physical robot
    and Gazebo simulation with latency monitoring.
    """

    def __init__(self):
        super().__init__('bridge_node')

        # ============================================
        # Parameters
        # ============================================

        self.declare_parameter(
            'mode', 'mirror',
            ParameterDescriptor(description='Operation mode: sim, live, mirror')
        )
        self.declare_parameter(
            'latency_threshold_ms', 50.0,
            ParameterDescriptor(description='Latency warning threshold in ms')
        )
        self.declare_parameter(
            'sync_rate_hz', 100.0,
            ParameterDescriptor(description='State synchronization rate')
        )

        mode_str = self.get_parameter('mode').value
        self.mode = BridgeMode(mode_str)
        self.latency_threshold = self.get_parameter('latency_threshold_ms').value

        # ============================================
        # State
        # ============================================

        self.latency_stats = LatencyStats()
        self.last_hw_msg: Optional[JointState] = None
        self.last_sim_msg: Optional[JointState] = None
        self.last_hw_time: float = 0.0
        self.last_sim_time: float = 0.0
        self.connected = {'hw': False, 'sim': False}

        self._lock = threading.Lock()

        # ============================================
        # QoS Profiles
        # ============================================

        # Reliable for commands
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Best effort for high-frequency sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Callback group for parallel execution
        self.cb_group = ReentrantCallbackGroup()

        # ============================================
        # Subscribers
        # ============================================

        self.hw_joint_sub = self.create_subscription(
            JointState,
            '/hw/joint_states',
            self.hw_joint_callback,
            sensor_qos,
            callback_group=self.cb_group
        )

        self.sim_joint_sub = self.create_subscription(
            JointState,
            '/sim/joint_states',
            self.sim_joint_callback,
            sensor_qos,
            callback_group=self.cb_group
        )

        self.cmd_sub = self.create_subscription(
            JointTrajectory,
            '/cmd/joint_trajectory',
            self.cmd_callback,
            reliable_qos,
            callback_group=self.cb_group
        )

        # ============================================
        # Publishers
        # ============================================

        self.sim_joint_pub = self.create_publisher(
            JointState,
            '/sim/joint_states_mirrored',
            sensor_qos
        )

        self.hw_cmd_pub = self.create_publisher(
            JointTrajectory,
            '/hw/joint_trajectory',
            reliable_qos
        )

        self.sim_cmd_pub = self.create_publisher(
            JointTrajectory,
            '/sim/joint_trajectory',
            reliable_qos
        )

        self.latency_pub = self.create_publisher(
            Float64,
            '/bridge/latency',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/bridge/status',
            10
        )

        # ============================================
        # Timers
        # ============================================

        self.latency_timer = self.create_timer(
            0.1,  # 10 Hz
            self.publish_latency,
            callback_group=self.cb_group
        )

        self.watchdog_timer = self.create_timer(
            1.0,  # 1 Hz
            self.watchdog_check,
            callback_group=self.cb_group
        )

        self.status_timer = self.create_timer(
            0.5,  # 2 Hz
            self.publish_status,
            callback_group=self.cb_group
        )

        # ============================================
        # Startup
        # ============================================

        self.get_logger().info(f'Bridge Node initialized in {self.mode.value} mode')
        self.get_logger().info(f'Latency threshold: {self.latency_threshold} ms')

    # ============================================
    # Callbacks
    # ============================================

    def hw_joint_callback(self, msg: JointState):
        """Handle physical robot joint states."""
        with self._lock:
            self.last_hw_msg = msg
            self.last_hw_time = time.time()
            self.connected['hw'] = True

        # Calculate latency
        latency = self._calculate_latency(msg)
        self._update_latency_stats(latency)

        # Mirror to simulation if in mirror mode
        if self.mode == BridgeMode.MIRROR:
            self.sim_joint_pub.publish(msg)

    def sim_joint_callback(self, msg: JointState):
        """Handle simulation joint states."""
        with self._lock:
            self.last_sim_msg = msg
            self.last_sim_time = time.time()
            self.connected['sim'] = True

    def cmd_callback(self, msg: JointTrajectory):
        """Handle incoming joint trajectory commands."""
        # Safety validation
        if not self._validate_command(msg):
            self.get_logger().warn('Command rejected by safety check')
            return

        # Route based on mode
        if self.mode == BridgeMode.SIM:
            self.sim_cmd_pub.publish(msg)
            self.get_logger().debug('Command routed to simulation')

        elif self.mode == BridgeMode.LIVE:
            if self._pre_live_check():
                self.hw_cmd_pub.publish(msg)
                self.get_logger().debug('Command routed to hardware')
            else:
                self.get_logger().warn('Live command blocked - safety check failed')

        elif self.mode == BridgeMode.MIRROR:
            # In mirror mode, commands go to simulation for testing
            self.sim_cmd_pub.publish(msg)
            self.get_logger().debug('Command routed to simulation (mirror mode)')

    # ============================================
    # Latency Monitoring
    # ============================================

    def _calculate_latency(self, msg: JointState) -> float:
        """Calculate message latency in milliseconds."""
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return 0.0

        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9

        latency_ms = (now - msg_time) * 1000
        return max(0.0, latency_ms)  # Clamp negative values

    def _update_latency_stats(self, latency_ms: float):
        """Update running latency statistics."""
        with self._lock:
            self.latency_stats.current_ms = latency_ms
            self.latency_stats.samples += 1

            # Running average
            n = self.latency_stats.samples
            self.latency_stats.average_ms = (
                (self.latency_stats.average_ms * (n - 1) + latency_ms) / n
            )

            # Track maximum
            if latency_ms > self.latency_stats.max_ms:
                self.latency_stats.max_ms = latency_ms

            # Track threshold violations
            if latency_ms > self.latency_threshold:
                self.latency_stats.threshold_violations += 1
                self.get_logger().warn(
                    f'Latency threshold exceeded: {latency_ms:.1f}ms > {self.latency_threshold}ms'
                )

    def publish_latency(self):
        """Publish current latency to monitoring topic."""
        msg = Float64()
        with self._lock:
            msg.data = self.latency_stats.current_ms
        self.latency_pub.publish(msg)

    # ============================================
    # Safety Checks
    # ============================================

    def _validate_command(self, msg: JointTrajectory) -> bool:
        """Validate command before forwarding."""
        # Check for empty trajectory
        if not msg.points:
            self.get_logger().warn('Empty trajectory rejected')
            return False

        # Check joint names present
        if not msg.joint_names:
            self.get_logger().warn('No joint names in trajectory')
            return False

        # Check position limits (example: +/- 3.14 rad)
        for point in msg.points:
            for pos in point.positions:
                if abs(pos) > 3.14:
                    self.get_logger().warn(f'Position {pos} exceeds limits')
                    return False

        return True

    def _pre_live_check(self) -> bool:
        """Additional checks before sending to live robot."""
        with self._lock:
            # Check hardware connection
            if not self.connected['hw']:
                self.get_logger().error('Hardware not connected')
                return False

            # Check latency is acceptable
            if self.latency_stats.current_ms > self.latency_threshold * 2:
                self.get_logger().error('Latency too high for live control')
                return False

        return True

    # ============================================
    # Watchdog
    # ============================================

    def watchdog_check(self):
        """Check connection status periodically."""
        now = time.time()
        timeout = 2.0  # seconds

        with self._lock:
            # Check hardware timeout
            if self.connected['hw'] and (now - self.last_hw_time) > timeout:
                self.connected['hw'] = False
                self.get_logger().warn('Hardware connection lost')

            # Check simulation timeout
            if self.connected['sim'] and (now - self.last_sim_time) > timeout:
                self.connected['sim'] = False
                self.get_logger().warn('Simulation connection lost')

    def publish_status(self):
        """Publish bridge status."""
        with self._lock:
            status = {
                'mode': self.mode.value,
                'hw_connected': self.connected['hw'],
                'sim_connected': self.connected['sim'],
                'latency_ms': round(self.latency_stats.current_ms, 2),
                'latency_avg_ms': round(self.latency_stats.average_ms, 2),
                'violations': self.latency_stats.threshold_violations
            }

        msg = String()
        msg.data = str(status)
        self.status_pub.publish(msg)

    # ============================================
    # Mode Control
    # ============================================

    def set_mode(self, mode: str) -> bool:
        """Change operation mode."""
        try:
            new_mode = BridgeMode(mode)
            self.mode = new_mode
            self.get_logger().info(f'Mode changed to: {mode}')
            return True
        except ValueError:
            self.get_logger().error(f'Invalid mode: {mode}')
            return False


def main(args=None):
    """Entry point for the bridge node."""
    rclpy.init(args=args)

    node = BridgeNode()

    # Use multi-threaded executor for parallel callbacks
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Latency Monitor

Create `latency_monitor.py` for dedicated latency tracking:

```python
#!/usr/bin/env python3
"""
Latency Monitor - Track and visualize bridge latency.

Subscribes to /bridge/latency and provides:
- Console output with warnings
- Latency history for analysis
- Threshold alerts
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from collections import deque
import statistics


class LatencyMonitor(Node):
    """Monitor and analyze bridge latency."""

    def __init__(self):
        super().__init__('latency_monitor')

        # Parameters
        self.declare_parameter('threshold_ms', 50.0)
        self.declare_parameter('window_size', 100)

        self.threshold = self.get_parameter('threshold_ms').value
        window_size = self.get_parameter('window_size').value

        # History buffer
        self.history = deque(maxlen=window_size)
        self.violation_count = 0

        # Subscriber
        self.sub = self.create_subscription(
            Float64,
            '/bridge/latency',
            self.latency_callback,
            10
        )

        # Periodic stats report
        self.timer = self.create_timer(5.0, self.report_stats)

        self.get_logger().info(f'Latency Monitor started (threshold: {self.threshold}ms)')

    def latency_callback(self, msg: Float64):
        """Process latency measurement."""
        latency = msg.data
        self.history.append(latency)

        if latency > self.threshold:
            self.violation_count += 1
            self.get_logger().warn(
                f'LATENCY ALERT: {latency:.1f}ms > {self.threshold}ms '
                f'(violations: {self.violation_count})'
            )

    def report_stats(self):
        """Report latency statistics."""
        if not self.history:
            return

        avg = statistics.mean(self.history)
        std = statistics.stdev(self.history) if len(self.history) > 1 else 0
        min_val = min(self.history)
        max_val = max(self.history)

        self.get_logger().info(
            f'Latency Stats: avg={avg:.1f}ms, std={std:.1f}ms, '
            f'min={min_val:.1f}ms, max={max_val:.1f}ms, '
            f'violations={self.violation_count}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LatencyMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Running the Bridge

### Launch Sequence

```bash
# Terminal 1: Start Gazebo simulation
ros2 launch gazebo_ros gazebo.launch.py world:=simple_lab.world

# Terminal 2: Start bridge node
ros2 run your_package bridge_node.py --ros-args \
  -p mode:=mirror \
  -p latency_threshold_ms:=50.0

# Terminal 3: Start latency monitor
ros2 run your_package latency_monitor.py

# Terminal 4: Monitor status
ros2 topic echo /bridge/status
```

### Testing the Bridge

```bash
# Check latency
ros2 topic echo /bridge/latency

# Send a test command
ros2 topic pub --once /cmd/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['left_knee'], points: [{positions: [0.5], time_from_start: {sec: 1}}]}"

# Change mode (if service implemented)
ros2 service call /bridge/set_mode std_srvs/srv/SetBool "{data: true}"
```

---

## Configuration

### Launch File with Parameters

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='bridge_node.py',
            name='bridge_node',
            parameters=[{
                'mode': 'mirror',
                'latency_threshold_ms': 50.0,
                'sync_rate_hz': 100.0,
            }],
            output='screen',
        ),
        Node(
            package='your_package',
            executable='latency_monitor.py',
            name='latency_monitor',
            parameters=[{
                'threshold_ms': 50.0,
                'window_size': 100,
            }],
            output='screen',
        ),
    ])
```

---

## Troubleshooting

### High Latency

| Symptom | Cause | Solution |
|---------|-------|----------|
| Latency > 100ms | Network congestion | Use wired connection |
| Latency spikes | GC pauses | Increase Python heap |
| Gradual increase | Queue buildup | Reduce publish rate |

### Connection Issues

```bash
# Check if topics exist
ros2 topic list | grep -E "(hw|sim)"

# Check message flow
ros2 topic hz /hw/joint_states
ros2 topic hz /sim/joint_states

# Check for errors
ros2 topic echo /rosout --field msg
```

### Mode Not Changing

Verify mode parameter is being set:
```bash
ros2 param get /bridge_node mode
ros2 param set /bridge_node mode "live"
```

---

## What's Next?

You now have a working digital twin bridge. The next section extends the bridge for AI training integration.

---

## AI Training Preparation

This section prepares your digital twin for integration with machine learning pipelines.

### Architecture Overview

The AI training pipeline connects simulation to external ML frameworks:

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   GAZEBO    │────►│   BRIDGE    │────►│  STREAMER   │────►│  ML MODEL   │
│ Simulation  │     │    Node     │     │   (Batch)   │     │ (Training)  │
└─────────────┘     └─────────────┘     └─────────────┘     └─────────────┘
      ▲                                                           │
      │                                                           │
      └───────────────────── Actions ─────────────────────────────┘
```

See the full diagram: [AI Training Architecture](assets/diagrams/ai-training-architecture.md)

### Sensor Streamer Node

The `sensor_streamer.py` node collects and batches sensor data for ML training:

```python
# Launch the sensor streamer
ros2 run your_package sensor_streamer --ros-args \
  -p batch_size:=32 \
  -p stream_rate_hz:=30.0 \
  -p api_endpoint:='http://localhost:8000/ingest'
```

Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `batch_size` | 32 | Samples per batch |
| `stream_rate_hz` | 30.0 | Max streaming rate |
| `include_images` | False | Include camera data |
| `api_endpoint` | localhost:8000 | ML server endpoint |

### Data Pipeline

1. **Collection**: Sensor data collected at 100Hz
2. **Batching**: Grouped into batches of 32 samples
3. **Streaming**: Sent to external API at 30Hz
4. **Training**: ML model processes batches

```bash
# Monitor streaming status
ros2 topic echo /ai/stream_status

# View batched data
ros2 topic echo /ai/sensor_batch
```

---

## Reinforcement Learning Integration Guide

This section explains how to connect your digital twin to RL training frameworks.

### Gym Environment Wrapper

Create an OpenAI Gym-compatible environment:

```python
import gymnasium as gym
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HumanoidEnv(gym.Env):
    """
    Gym environment wrapping the Gazebo digital twin.

    Observation: Joint positions, velocities, IMU data
    Action: Target joint positions
    Reward: Forward velocity + upright bonus - energy penalty
    """

    def __init__(self, node: Node):
        super().__init__()
        self.node = node

        # Define spaces
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(128,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(24,), dtype=np.float32
        )

        # ROS 2 setup
        self.joint_sub = node.create_subscription(
            JointState, '/sim/joint_states',
            self._joint_callback, 10
        )
        self.cmd_pub = node.create_publisher(
            JointTrajectory, '/cmd/joint_trajectory', 10
        )

        self.latest_state = None

    def _joint_callback(self, msg):
        self.latest_state = msg

    def reset(self, seed=None):
        # Reset simulation
        # Return initial observation
        obs = self._get_observation()
        return obs, {}

    def step(self, action):
        # Send action to simulation
        self._send_action(action)

        # Wait for next state
        rclpy.spin_once(self.node, timeout_sec=0.05)

        # Get observation
        obs = self._get_observation()

        # Calculate reward
        reward = self._compute_reward(obs, action)

        # Check termination
        terminated = self._check_terminated(obs)

        return obs, reward, terminated, False, {}

    def _get_observation(self):
        if self.latest_state is None:
            return np.zeros(128, dtype=np.float32)

        obs = np.zeros(128, dtype=np.float32)
        obs[:len(self.latest_state.position)] = self.latest_state.position
        return obs

    def _send_action(self, action):
        msg = JointTrajectory()
        msg.joint_names = ['joint_' + str(i) for i in range(24)]

        point = JointTrajectoryPoint()
        point.positions = action.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50_000_000  # 50ms

        msg.points = [point]
        self.cmd_pub.publish(msg)

    def _compute_reward(self, obs, action):
        # Example reward function
        alive_bonus = 1.0
        forward_reward = obs[0] * 0.5  # Assuming first obs is forward velocity
        energy_penalty = -0.01 * np.sum(action ** 2)
        return alive_bonus + forward_reward + energy_penalty

    def _check_terminated(self, obs):
        # Example: terminate if fallen
        return False  # Implement fall detection
```

### Training Loop Example

```python
import stable_baselines3 as sb3
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv

def make_env(rank):
    def _init():
        rclpy.init()
        node = rclpy.create_node(f'rl_env_{rank}')
        env = HumanoidEnv(node)
        return env
    return _init

# Create parallel environments
num_envs = 8
env = SubprocVecEnv([make_env(i) for i in range(num_envs)])

# Train with PPO
model = PPO(
    "MlpPolicy",
    env,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    n_epochs=10,
    gamma=0.99,
    verbose=1
)

model.learn(total_timesteps=1_000_000)
model.save("humanoid_walker")
```

### Sim-to-Real Considerations

When transferring policies from simulation to real hardware:

1. **Domain Randomization**: Vary physics parameters during training
   ```python
   # Randomize friction, mass, latency
   friction = np.random.uniform(0.5, 1.5)
   mass_scale = np.random.uniform(0.9, 1.1)
   latency_ms = np.random.uniform(0, 20)
   ```

2. **Action Smoothing**: Limit action rate of change
   ```python
   action = 0.8 * prev_action + 0.2 * new_action
   ```

3. **Observation Noise**: Add realistic sensor noise
   ```python
   obs += np.random.normal(0, 0.01, obs.shape)
   ```

4. **Safety Constraints**: Enforce joint limits and velocities
   ```python
   action = np.clip(action, joint_min, joint_max)
   ```

### Performance Targets

| Metric | Target | How to Measure |
|--------|--------|----------------|
| Env Steps/sec | > 1000 | With headless Gazebo |
| Training Throughput | 1M steps/hour | 8 parallel envs |
| Policy Latency | < 10ms | Model inference time |
| Sim-to-Real Gap | < 20% | Performance difference |

---

## AI Agent Assisted Prompts

### Implementation Review
```
Review this bridge_node.py implementation for potential issues:
1. Thread safety concerns
2. Memory leaks in the latency history
3. QoS mismatches that could cause message drops
4. Edge cases in clock synchronization

Suggest specific fixes for each issue found.
```

### Performance Optimization
```
My bridge node shows 45ms average latency but occasional spikes to 200ms.
The hardware publishes at 500Hz and simulation at 1000Hz.
Analyze potential causes and provide optimization strategies focusing on:
- Python-specific optimizations
- ROS 2 executor tuning
- Message serialization overhead
```

### Testing Strategy
```
Create a comprehensive test plan for the digital twin bridge including:
1. Unit tests for each method
2. Integration tests with mock publishers
3. Performance benchmarks
4. Failure injection tests
5. End-to-end validation with real hardware simulation
```

---

## Summary

- Bridge node handles bidirectional sync between physical and virtual
- Three modes: sim, live, mirror
- Latency monitoring with configurable threshold (<50ms target)
- Safety gates validate commands before live forwarding
- Watchdog detects connection timeouts
- Use MultiThreadedExecutor for parallel callback handling

---

| Previous | Up | Next |
|----------|-----|------|
| [A1: Data Synchronization](A1-data-synchronization.md) | [Advanced Tier](../README.md#advanced-tier) | [Exercise 03: Build Bridge](../exercises/exercise-03-build-bridge.md) |
