# Data Model: Chapter 2 - Digital Twin & Simulation

**Generated**: 2025-12-25
**Phase**: 1 (Design & Contracts)

## Entity Definitions

### 1. Digital Twin

A virtual robot replica maintaining real-time synchronization with its physical counterpart.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| `robot_name` | string | Unique identifier for the robot | Required, matches URDF robot name |
| `urdf_path` | string | Path to URDF description file | Valid file path, Chapter 1 dependency |
| `sync_mode` | enum | Synchronization direction | `SIM_TO_REAL`, `REAL_TO_SIM`, `BIDIRECTIONAL` |
| `sync_rate_hz` | float | Data synchronization frequency | 10-100 Hz, default 20 Hz |
| `latency_threshold_ms` | float | Maximum acceptable latency | Default 50ms |

**State Transitions**:
```
INITIALIZED → CONNECTING → SYNCHRONIZED → DEGRADED → DISCONNECTED
     ↑              ↓            ↓           ↓
     └──────────────┴────────────┴───────────┘
                    (on error recovery)
```

### 2. Simulation World

A 3D environment file describing physics properties and spatial configuration.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| `world_name` | string | Unique world identifier | Required |
| `physics_engine` | enum | Physics solver type | `ODE`, `BULLET`, `DART` |
| `real_time_update_rate` | int | Physics updates per second | 100-1000, default 1000 |
| `max_step_size` | float | Maximum physics step | 0.0001-0.01, default 0.001 |
| `gravity` | vector3 | Gravity vector (m/s²) | Default [0, 0, -9.81] |

**Relationships**:
- Contains 0..n `Model` instances
- Contains 0..n `Light` instances
- Contains 1 `GroundPlane`

### 3. Physics Engine Configuration

Runtime parameters for physics simulation.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| `solver_type` | enum | Constraint solver | `QUICK`, `WORLD` |
| `solver_iterations` | int | Solver iteration count | 10-100, default 50 |
| `sor_parameter` | float | Successive over-relaxation | 0.0-2.0, default 1.3 |
| `contact_surface_layer` | float | Contact tolerance | 0.0001-0.01 |

### 4. Bridge Node

ROS 2 component synchronizing data between physical and simulated robots.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| `node_name` | string | ROS 2 node name | Default `digital_twin_bridge` |
| `real_topic_prefix` | string | Physical robot topic namespace | Default `/robot` |
| `sim_topic_prefix` | string | Simulation topic namespace | Default `/sim` |
| `qos_reliability` | enum | QoS reliability policy | `BEST_EFFORT`, `RELIABLE` |
| `qos_depth` | int | Message queue depth | 1-100, default 10 |

**Subscribed Topics**:
- `{real_topic_prefix}/joint_states` (sensor_msgs/JointState)
- `{sim_topic_prefix}/joint_states` (sensor_msgs/JointState)

**Published Topics**:
- `{real_topic_prefix}/joint_commands` (trajectory_msgs/JointTrajectory)
- `{sim_topic_prefix}/joint_commands` (trajectory_msgs/JointTrajectory)

### 5. Model Spawner

Service/tool for inserting URDF descriptions into simulation.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| `model_name` | string | Spawned model identifier | Required, unique in world |
| `urdf_content` | string | URDF XML or file path | Valid URDF required |
| `initial_pose` | Pose | Spawn position/orientation | Default origin |
| `reference_frame` | string | Parent frame for pose | Default `world` |

### 6. Real-Time Factor (RTF) Monitor

Performance metric tracking simulation speed.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| `current_rtf` | float | Current real-time factor | 0.0-∞, target >= 0.8 |
| `average_rtf` | float | Rolling average RTF | 10-second window |
| `min_threshold` | float | Warning threshold | Default 0.8 |
| `sample_interval_ms` | int | Measurement interval | Default 100ms |

**Events**:
- `RTF_DEGRADED`: RTF drops below threshold
- `RTF_RECOVERED`: RTF returns above threshold

## Relationship Diagram

```
┌─────────────────┐     contains      ┌──────────────────┐
│ Simulation World │◄─────────────────│ Physics Engine   │
└────────┬────────┘                   └──────────────────┘
         │
         │ spawns
         ▼
┌─────────────────┐     describes     ┌──────────────────┐
│  Model Spawner  │◄─────────────────│   URDF Model     │
└────────┬────────┘                   │  (from Ch. 1)    │
         │                            └──────────────────┘
         │ creates
         ▼
┌─────────────────┐     syncs with    ┌──────────────────┐
│  Digital Twin   │◄────────────────►│  Physical Robot  │
└────────┬────────┘                   │  (future ch.)    │
         │                            └──────────────────┘
         │ uses
         ▼
┌─────────────────┐     monitors      ┌──────────────────┐
│  Bridge Node    │◄─────────────────│   RTF Monitor    │
└─────────────────┘                   └──────────────────┘
```

## Validation Rules

1. **URDF Inertia Validation**: All links must have valid inertia matrices (positive definite)
2. **Topic Naming**: All topics must follow ROS 2 naming conventions (`/namespace/topic_name`)
3. **RTF Threshold**: System must warn when RTF < 0.8 for > 5 seconds
4. **Latency Check**: Bridge must log warning when message latency > 50ms
5. **World File Syntax**: Must be valid SDF 1.6 format
