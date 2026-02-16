# AI Training Architecture Diagram

This document provides the architectural diagram for integrating digital twin simulation with AI/ML training pipelines.

## Control Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                          AI TRAINING PIPELINE                                    │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐      │
│  │   GAZEBO    │    │   BRIDGE    │    │   SENSOR    │    │    ML       │      │
│  │ SIMULATION  │───►│    NODE     │───►│  STREAMER   │───►│  TRAINING   │      │
│  │             │    │             │    │             │    │             │      │
│  │ /sim/joints │    │ Latency Mon │    │ Batching    │    │ PyTorch/TF  │      │
│  │ /sim/imu    │    │ Safety Gate │    │ HTTP/gRPC   │    │ Gym Env     │      │
│  │ /sim/camera │    │ Mode Switch │    │ Buffering   │    │ RL/IL Algo  │      │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘    └──────┬──────┘      │
│         │                  │                  │                  │              │
│         │                  │                  │                  │              │
│         ▼                  ▼                  ▼                  ▼              │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                         ROS 2 MESSAGE BUS                                │   │
│  │                                                                          │   │
│  │  Topics: /sim/joint_states, /ai/sensor_batch, /cmd/joint_trajectory     │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│         │                  │                  │                  │              │
│         │                  │                  │                  │              │
│         ▼                  ▼                  ▼                  ▼              │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐   ┌──────────────┐     │
│  │   PHYSICS    │   │   DIGITAL    │   │   EXTERNAL   │   │   ACTION     │     │
│  │   ENGINE     │   │    TWIN      │   │     API      │   │   EXECUTOR   │     │
│  │              │   │              │   │              │   │              │     │
│  │  ODE/DART    │◄──│  Sync Loop   │◄──│  ML Server   │◄──│  Policy Net  │     │
│  │  1000 Hz     │   │  50ms target │   │  Inference   │   │  Actor/Critic│     │
│  └──────────────┘   └──────────────┘   └──────────────┘   └──────────────┘     │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Data Flow Diagram

```
                    SENSOR DATA FLOW (Observation)
                    ═══════════════════════════════

┌─────────────┐     ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   SENSORS   │────►│   BRIDGE    │────►│  STREAMER   │────►│  ML MODEL   │
│             │     │             │     │             │     │             │
│ Joint State │     │ Timestamp   │     │ Batch (32)  │     │ Observation │
│ IMU Data    │     │ Validation  │     │ JSON/Proto  │     │ Vector      │
│ Camera RGB  │     │ QoS Match   │     │ HTTP POST   │     │ [dim: 128]  │
│ Force/Torq  │     │             │     │             │     │             │
└─────────────┘     └─────────────┘     └─────────────┘     └─────────────┘
      │                   │                   │                   │
      │                   │                   │                   │
      ▼                   ▼                   ▼                   ▼
    100 Hz             100 Hz              30 Hz              30 Hz


                    ACTION DATA FLOW (Control)
                    ═══════════════════════════

┌─────────────┐     ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│  ML MODEL   │────►│   BRIDGE    │────►│  GAZEBO     │────►│   ROBOT     │
│             │     │             │     │             │     │             │
│ Action Vec  │     │ Safety Clamp│     │ Apply Force │     │ Joint Move  │
│ [dim: 24]   │     │ Rate Limit  │     │ Set Velocity│     │ Actuator    │
│ Trajectory  │     │ Mode Check  │     │ Joint Ctrl  │     │             │
│             │     │             │     │             │     │             │
└─────────────┘     └─────────────┘     └─────────────┘     └─────────────┘
      │                   │                   │                   │
      │                   │                   │                   │
      ▼                   ▼                   ▼                   ▼
    30 Hz              100 Hz             1000 Hz             50 Hz
```

## Component Responsibilities

| Component | Primary Role | Update Rate | Latency Budget |
|-----------|--------------|-------------|----------------|
| Gazebo Simulation | Physics, Rendering | 1000 Hz internal | N/A |
| Bridge Node | Sync, Safety, Routing | 100 Hz | 10ms |
| Sensor Streamer | Batching, HTTP | 30 Hz | 20ms |
| ML Training | Policy Learning | 30 Hz | N/A |
| Action Executor | Command Publishing | 30 Hz | 10ms |

## RL Training Loop

```
┌───────────────────────────────────────────────────────────────┐
│                    REINFORCEMENT LEARNING LOOP                 │
├───────────────────────────────────────────────────────────────┤
│                                                               │
│    ┌─────────┐                              ┌─────────┐       │
│    │  ENV    │                              │  AGENT  │       │
│    │(Gazebo) │                              │ (Policy)│       │
│    └────┬────┘                              └────┬────┘       │
│         │                                        │            │
│         │    state (s_t)                         │            │
│         ├───────────────────────────────────────►│            │
│         │                                        │            │
│         │                                        │ π(s_t)     │
│         │                                        │            │
│         │    action (a_t)                        │            │
│         │◄───────────────────────────────────────┤            │
│         │                                        │            │
│         │    step()                              │            │
│         │                                        │            │
│         │    (s_{t+1}, reward, done)             │            │
│         ├───────────────────────────────────────►│            │
│         │                                        │            │
│         │                                        │ update()   │
│         │                                        │            │
│                                                               │
│    Episode repeats until done or max_steps                    │
│                                                               │
└───────────────────────────────────────────────────────────────┘
```

## State Space (Observation)

For a 24-DOF humanoid robot:

```
Observation Vector [dim=128]:
├── Joint Positions    [24 floats]   - Current joint angles (rad)
├── Joint Velocities   [24 floats]   - Current joint velocities (rad/s)
├── Joint Torques      [24 floats]   - Current joint efforts (Nm)
├── IMU Orientation    [4 floats]    - Quaternion (x, y, z, w)
├── IMU Angular Vel    [3 floats]    - Angular velocity (rad/s)
├── IMU Linear Acc     [3 floats]    - Linear acceleration (m/s²)
├── Foot Contact       [4 floats]    - Contact forces (normalized)
├── Base Velocity      [6 floats]    - Linear + angular velocity
├── Target Position    [3 floats]    - Goal position (m)
├── Target Velocity    [3 floats]    - Desired velocity (m/s)
├── Phase Variable     [2 floats]    - Gait phase (sin, cos)
└── Time Step          [1 float]     - Episode progress
    ─────────────────────────────────
    Total: 101 floats (padded to 128)
```

## Action Space

```
Action Vector [dim=24]:
├── Hip Joints         [6 floats]    - 3 DOF per leg × 2 legs
├── Knee Joints        [2 floats]    - 1 DOF per leg × 2 legs
├── Ankle Joints       [4 floats]    - 2 DOF per leg × 2 legs
├── Shoulder Joints    [6 floats]    - 3 DOF per arm × 2 arms
├── Elbow Joints       [2 floats]    - 1 DOF per arm × 2 arms
├── Wrist Joints       [4 floats]    - 2 DOF per arm × 2 arms
    ─────────────────────────────────
    Total: 24 joint position targets

Action Range: [-1, 1] normalized
Mapped to: Joint limits per URDF specification
```

## Reward Function Design

```
Reward = w1 × R_alive
       + w2 × R_forward
       + w3 × R_upright
       + w4 × R_energy
       + w5 × R_smooth

Where:
├── R_alive    = 1.0 if not fallen, else 0.0
├── R_forward  = forward_velocity / target_velocity
├── R_upright  = cos(torso_tilt)
├── R_energy   = -sum(joint_torques²) / max_torque
├── R_smooth   = -sum(delta_actions²)

Typical weights: [1.0, 0.5, 0.3, 0.01, 0.01]
```

## Implementation Notes

### Prerequisites for AI Training

1. **Gym Environment Wrapper**: Create OpenAI Gym-compatible environment
2. **Observation Normalization**: Scale inputs to [-1, 1] range
3. **Action Clipping**: Enforce joint limits
4. **Reward Shaping**: Balance exploration vs exploitation
5. **Parallel Environments**: Use SubprocVecEnv for throughput

### Performance Targets

| Metric | Target | Notes |
|--------|--------|-------|
| Environment Steps/sec | > 1000 | With headless Gazebo |
| Observation Latency | < 10ms | From sim to ML model |
| Action Latency | < 10ms | From ML model to sim |
| Training Throughput | 1M steps/hour | With 8 parallel envs |

### Recommended Algorithms

| Task | Algorithm | Reference |
|------|-----------|-----------|
| Walking | PPO | Schulman et al., 2017 |
| Manipulation | SAC | Haarnoja et al., 2018 |
| Imitation | GAIL | Ho & Ermon, 2016 |
| Sim-to-Real | Domain Randomization | Tobin et al., 2017 |
