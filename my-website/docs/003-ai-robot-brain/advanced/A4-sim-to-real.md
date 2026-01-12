# A4: Sim-to-Real Transfer

**Deploying Learned Policies on Real Robots**

---

## Learning Objectives

By the end of this lesson, you will be able to:
- Explain the sim-to-real gap and its causes
- Describe domain randomization techniques
- Load and run pre-trained policies using ONNX
- Understand safety considerations for real robot deployment

---

## Prerequisites

- Completed [A3: Reinforcement Learning Fundamentals](A3-reinforcement-learning.md)
- Understanding of neural network inference
- Basic knowledge of robot safety

---

## Theory: The Sim-to-Real Gap

### What is the Sim-to-Real Gap?

The **sim-to-real gap** refers to differences between simulation and reality that cause trained policies to fail on real robots.

![Sim-to-Real Gap](diagrams/sim-to-real-gap.svg)

*Alt-text: Diagram illustrating the sim-to-real gap. On the left, a simulated robot in a simplified environment. On the right, a real robot with complex physics, sensor noise, and real-world variations. Arrows show the gap between simulation (perfect physics, no noise) and reality (complex physics, sensor noise, actuator delays).*

### Sources of the Gap

| Category | Simulation | Reality |
|----------|-----------|---------|
| **Physics** | Simplified models | Complex dynamics |
| **Sensors** | Perfect readings | Noise, latency |
| **Actuators** | Instant response | Delays, backlash |
| **Environment** | Static, known | Dynamic, variable |
| **Contacts** | Simplified friction | Complex surface interactions |

### Consequences

- Policy works perfectly in simulation
- Policy fails catastrophically on real robot
- Robot may fall, collide, or behave erratically

---

## Techniques to Bridge the Gap

### 1. Domain Randomization

**Randomize simulation parameters** during training so the policy learns to be robust:

```python
# Domain randomization example
class RandomizedEnv:
    def reset(self):
        # Randomize physics
        self.friction = np.random.uniform(0.5, 1.5)
        self.mass_scale = np.random.uniform(0.8, 1.2)

        # Randomize sensors
        self.sensor_noise_std = np.random.uniform(0.0, 0.1)
        self.sensor_delay = np.random.randint(0, 3)

        # Randomize actuators
        self.motor_strength = np.random.uniform(0.9, 1.1)
        self.action_delay = np.random.randint(0, 2)
```

**Common Randomization Targets**:
- Friction coefficients
- Mass and inertia
- Motor strength
- Sensor noise
- Control delays
- Link lengths
- External forces

### 2. System Identification

**Measure real robot parameters** and match simulation:

```python
# System identification measurements
real_robot_params = {
    'motor_kp': 80.0,    # Measured from step response
    'motor_kd': 5.0,
    'friction': 0.8,     # Measured from sliding test
    'latency_ms': 15,    # Measured from command-response
}

# Update simulation
sim.set_motor_gains(real_robot_params['motor_kp'], real_robot_params['motor_kd'])
```

### 3. Progressive Training

**Train in stages** from simple to realistic:

1. **Stage 1**: Perfect simulation, no noise
2. **Stage 2**: Add sensor noise
3. **Stage 3**: Add actuator delays
4. **Stage 4**: Add physics randomization
5. **Stage 5**: Deploy to real robot

---

## Loading Pre-Trained Policies

### ONNX Format

**ONNX** (Open Neural Network Exchange) is a standard format for trained models:

- Works across frameworks (PyTorch → ONNX → TensorRT)
- Optimized for inference
- Portable to edge devices

### Code Example: Policy Loader

```python
#!/usr/bin/env python3
"""
policy_loader.py

Loads a pre-trained ONNX policy and runs inference.
Demonstrates sim-to-real policy deployment pattern.

Dependencies: onnxruntime, numpy

Usage:
  python3 policy_loader.py
"""

import numpy as np

try:
    import onnxruntime as ort
except ImportError:
    print("Install onnxruntime: pip install onnxruntime")
    raise


class PolicyLoader:
    """Loads and runs inference on an ONNX policy."""

    def __init__(self, model_path: str):
        """Initialize the policy loader.

        Args:
            model_path: Path to .onnx policy file
        """
        # Create inference session
        self.session = ort.InferenceSession(
            model_path,
            providers=['CPUExecutionProvider']  # or CUDAExecutionProvider
        )

        # Get input/output info
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.output_name = self.session.get_outputs()[0].name

        print(f"Loaded policy: {model_path}")
        print(f"Input: {self.input_name} {self.input_shape}")
        print(f"Output: {self.output_name}")

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        """Run policy inference to get action.

        Args:
            observation: Current state observation

        Returns:
            Action to execute
        """
        # Ensure correct shape and type
        obs = observation.astype(np.float32)
        if len(obs.shape) == 1:
            obs = obs.reshape(1, -1)  # Add batch dimension

        # Run inference
        outputs = self.session.run(
            [self.output_name],
            {self.input_name: obs}
        )

        action = outputs[0][0]  # Remove batch dimension
        return action


def main():
    """Example usage with dummy observations."""

    # Load the pre-trained policy
    policy = PolicyLoader('pretrained/locomotion_policy.onnx')

    # Simulate observations (replace with real sensor data)
    obs_dim = 48  # Typical for quadruped: joints + velocities + commands
    observation = np.random.randn(obs_dim).astype(np.float32)

    # Get action
    action = policy.get_action(observation)
    print(f"Observation shape: {observation.shape}")
    print(f"Action shape: {action.shape}")
    print(f"Action values: {action[:5]}...")  # First 5 values

    # In real deployment:
    # while True:
    #     observation = get_robot_state()
    #     action = policy.get_action(observation)
    #     send_to_robot(action)


if __name__ == '__main__':
    main()
```

---

## Safety Considerations

### Pre-Deployment Checklist

> **WARNING**: Real robot deployment requires careful safety protocols. Failure to follow safety procedures can result in damage to the robot, environment, or injury to humans.

- [ ] **Emergency stop** tested and accessible
- [ ] **Safety perimeter** established (no humans in workspace)
- [ ] **Velocity limits** configured in hardware
- [ ] **Joint limits** enforced in software
- [ ] **Collision detection** enabled
- [ ] **Initial tests** at reduced speed
- [ ] **Fallback behavior** defined

### Speed Progression

| Stage | Speed | Duration |
|-------|-------|----------|
| 1 | 10% max | Until stable |
| 2 | 25% max | 10+ minutes |
| 3 | 50% max | 30+ minutes |
| 4 | 75% max | Extended testing |
| 5 | 100% max | Production ready |

### Monitoring During Deployment

```python
def safe_execution_loop(policy, robot):
    """Safe policy execution with monitoring."""

    MAX_VELOCITY = 0.5  # m/s safety limit
    MAX_TORQUE = 10.0   # Nm safety limit

    while True:
        # Get observation
        obs = robot.get_observation()

        # Get action from policy
        action = policy.get_action(obs)

        # Safety clipping
        action = np.clip(action, -MAX_TORQUE, MAX_TORQUE)

        # Check velocity limits
        if np.any(np.abs(robot.joint_velocities) > MAX_VELOCITY):
            print("VELOCITY LIMIT EXCEEDED - stopping")
            robot.emergency_stop()
            break

        # Execute (with rate limiting)
        robot.send_action(action)
        time.sleep(0.01)  # 100 Hz control
```

---

## Key Concepts Summary

| Concept | Definition |
|---------|------------|
| **Sim-to-Real Gap** | Differences between simulation and reality |
| **Domain Randomization** | Randomize simulation during training |
| **System Identification** | Measure and match real robot parameters |
| **ONNX** | Portable neural network format |
| **Safety Limits** | Hardware and software safeguards |

---

## Hands-On Exercise

### Exercise A4.1: Domain Randomization Design

For a quadruped robot, list 10 parameters you would randomize:

| Parameter | Range | Rationale |
|-----------|-------|-----------|
| 1. | | |
| 2. | | |
| ... | | |

<details>
<summary>Click for suggested answer</summary>

1. **Friction coefficient**: 0.5-1.5 (floor surfaces vary)
2. **Base mass**: ±20% (payload variations)
3. **Motor strength**: ±10% (manufacturing tolerances)
4. **Joint damping**: ±30% (wear and temperature)
5. **Sensor noise**: 0-5% (sensor quality)
6. **Control latency**: 0-20ms (communication delays)
7. **External push force**: 0-50N (unexpected contacts)
8. **Ground height**: ±5cm (uneven terrain)
9. **Link lengths**: ±2% (calibration errors)
10. **Gravity direction**: ±5° (slopes)

</details>

### Exercise A4.2: Safety Protocol

Create a safety checklist for deploying a mobile robot policy:

1. Hardware safety measures
2. Software safety limits
3. Testing progression
4. Monitoring during operation

---

## AI Agent Assisted Prompts

### Prompt 1: Gap Analysis
```
My quadruped policy trained in Isaac Gym falls over when I deploy it on the
real robot. The simulation uses simplified contact models. What are the most
likely causes and how can I diagnose the specific problem?
```

### Prompt 2: Domain Randomization Strategy
```
I'm training a manipulation policy for a robot arm. What parameters should
I randomize and what ranges should I use? I want good real-world transfer
without making training too hard.
```

### Prompt 3: Real-Time Inference
```
My ONNX policy runs at 50ms per inference on CPU but I need 1kHz control.
What optimization strategies can I use? Should I use TensorRT, model
pruning, or simpler architectures?
```

---

## Summary

In this lesson, you learned:

1. **Sim-to-real gap** causes policies to fail on real robots
2. **Domain randomization** makes policies robust to variations
3. **System identification** matches simulation to reality
4. **ONNX** enables portable policy deployment
5. **Safety protocols** are essential for real robot deployment

---

## Chapter Complete!

**Congratulations!** You've completed Chapter 4: AI-Robot Brain!

You now understand:
- **Perception** pipelines and sensor processing
- **SLAM** and map generation
- **Navigation** with Nav2 and behavior trees
- **Reinforcement learning** fundamentals
- **Sim-to-real** transfer techniques

### What's Next?

- Complete [Advanced Exercises](../exercises/advanced-exercises.md)
- Apply these concepts to your own robot projects
- Continue to Chapter 4 for manipulation and grasping
