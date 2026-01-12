# Research: Chapter 2 - Digital Twin & Simulation

**Generated**: 2025-12-25
**Phase**: 0 (Outline & Research)
**Status**: Complete

## Research Tasks Completed

### 1. Gazebo Classic vs Ignition for ROS 2 Humble

**Decision**: Gazebo Classic (gazebo11)

**Rationale**:
- Gazebo Classic has mature, stable integration with ROS 2 Humble via `gazebo_ros_pkgs`
- Extensive documentation and community examples specifically for humanoid robotics
- Lower installation complexity for beginners (single apt install)
- Most existing URDF models and tutorials target Classic
- Ignition (now Gazebo Sim) has newer architecture but less ROS 2 Humble coverage

**Alternatives Considered**:
- Ignition Fortress: Better architecture but steeper learning curve, less tutorial coverage
- Unity + ROS 2: Higher fidelity visuals but requires Unity license knowledge
- Isaac Sim: NVIDIA-specific, requires GPU, deferred to Chapter 4

### 2. Physics Engine Selection

**Decision**: ODE (Open Dynamics Engine) - Gazebo default

**Rationale**:
- Default physics engine in Gazebo Classic, zero configuration required
- Stable for humanoid simulation with proper inertia values
- Well-documented parameter tuning for robotics applications
- Sufficient for educational purposes without specialized physics needs

**Alternatives Considered**:
- Bullet: Better for soft-body, but adds complexity
- DART: Good for articulated bodies but less Gazebo integration docs
- Simbody: Specialized for biomechanics, overkill for intro chapter

### 3. ROS 2 Control Framework

**Decision**: Use `ros2_control` with `gazebo_ros2_control` plugin

**Rationale**:
- Industry-standard approach for joint control in ROS 2
- Clean abstraction between simulation and hardware
- Consistent API students will use on real robots
- Excellent documentation in ROS 2 Humble

**Alternatives Considered**:
- Direct topic publishing: Simpler but not portable to real hardware
- Custom controllers: Educational overhead without benefit
- MoveIt2 integration: Deferred to later chapters (motion planning focus)

### 4. Digital Twin Data Synchronization Pattern

**Decision**: ROS 2 Topic-based bidirectional bridge with QoS configuration

**Rationale**:
- Native ROS 2 pattern using publishers/subscribers
- QoS profiles handle reliability vs latency tradeoffs
- Easy to visualize data flow for educational purposes
- Scales to multiple data streams (joints, sensors, commands)

**Alternatives Considered**:
- ROS 2 Services: Request-response not suitable for continuous sync
- ROS 2 Actions: Overkill for simple data mirroring
- Custom middleware: Not ROS 2 native, loses educational value

### 5. Performance Monitoring Approach

**Decision**: Built-in Gazebo RTF + custom latency tracking node

**Rationale**:
- RTF (Real-Time Factor) is standard Gazebo metric, visible in GUI
- Custom Python node demonstrates practical monitoring skills
- 50ms latency threshold aligns with typical 20Hz control loops
- 0.8 RTF threshold is industry-accepted for reliable physics

**Alternatives Considered**:
- External profiling tools: Too complex for educational context
- Gazebo logging only: Misses ROS 2 side latency
- No monitoring: Students need to understand performance constraints

### 6. URDF Validation Strategy

**Decision**: Use `check_urdf` tool + custom Python validator script

**Rationale**:
- `check_urdf` catches syntax errors before Gazebo launch
- Custom script validates inertia matrices (common failure point)
- Provides actionable error messages for students
- Integrates with troubleshooting section of edge cases

**Alternatives Considered**:
- Gazebo-only validation: Error messages less helpful
- Manual inspection: Not scalable for students
- xacro --check: Only covers xacro syntax, not physics validity

## Best Practices Identified

### Gazebo World File Structure
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_lab">
    <!-- Physics configuration -->
    <physics type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Robot spawned via ROS 2 -->
  </world>
</sdf>
```

### ROS 2 Bridge Node Pattern
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

class DigitalTwinBridge(Node):
    def __init__(self):
        super().__init__('digital_twin_bridge')

        # QoS for real-time data
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Bidirectional subscriptions
        self.real_sub = self.create_subscription(...)
        self.sim_sub = self.create_subscription(...)

        # Publishers
        self.to_sim_pub = self.create_publisher(...)
        self.to_real_pub = self.create_publisher(...)
```

### Latency Measurement Pattern
```python
import time

class LatencyMonitor:
    def __init__(self, threshold_ms=50):
        self.threshold = threshold_ms / 1000.0
        self.last_timestamp = None

    def check(self, msg_timestamp):
        latency = time.time() - msg_timestamp
        if latency > self.threshold:
            self.get_logger().warn(f'Latency {latency*1000:.1f}ms exceeds threshold')
        return latency
```

## Unresolved Items

None - all NEEDS CLARIFICATION items from Technical Context resolved.

## References

1. ROS 2 Humble Documentation: https://docs.ros.org/en/humble/
2. Gazebo Classic Tutorials: https://classic.gazebosim.org/tutorials
3. ros2_control Documentation: https://control.ros.org/humble/
4. gazebo_ros_pkgs: https://github.com/ros-simulation/gazebo_ros_pkgs
