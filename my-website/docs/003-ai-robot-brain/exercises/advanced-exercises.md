# Advanced Tier Exercises

**Test Your Skills in Costmaps, Planners, RL, and Sim-to-Real**

---

## Overview

These exercises test your advanced knowledge from:
- A1: Costmap Configuration
- A2: Planners and Behavior Trees
- A3: Reinforcement Learning Fundamentals
- A4: Sim-to-Real Transfer

---

## Exercise 1: Costmap Layer Design

**Task**: Design a custom costmap configuration for a hospital delivery robot.

### Requirements

The robot must:
- Avoid patients and staff (high priority)
- Stay away from medical equipment
- Navigate narrow hallways (1.2m wide)
- Handle automatic doors

### Your Design

Complete this configuration:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: ___      # Hospital delivery robot size
      resolution: ___        # Balance accuracy/performance
      plugins: [___]         # What layers do you need?

      # Configure each layer
      static_layer:
        # Your config

      obstacle_layer:
        observation_sources: ___  # What sensors?
        # Your config

      inflation_layer:
        inflation_radius: ___     # How much safety margin?
        cost_scaling_factor: ___  # Steep or gradual?
```

### Bonus
How would you add a "keep-out zone" layer for restricted areas (e.g., operating rooms)?

---

## Exercise 2: Behavior Tree Modification

**Task**: Modify the navigation behavior tree for a warehouse robot.

### Requirements

The robot should:
1. Try to navigate normally
2. If blocked for >5 seconds, check for a human
3. If human detected, wait politely for 10 seconds
4. If still blocked, find alternate route
5. After 3 failures, call for help

### Starter XML

```xml
<root main_tree_to_execute="WarehouseNav">
  <BehaviorTree ID="WarehouseNav">
    <!-- Your design here -->
    <!--
      Hints:
      - Use RecoveryNode for retry logic
      - Use Timeout decorator for time limits
      - Use custom conditions for human detection
      - Use service calls for "call for help"
    -->
  </BehaviorTree>
</root>
```

### Questions

1. What BT nodes would you need for human detection?
2. How would you implement "call for help"?
3. What recovery behaviors are appropriate for a warehouse?

---

## Exercise 3: Reward Function Design

**Task**: Design a reward function for a robot arm sorting task.

### Scenario

A robot arm must:
- Pick up objects from a bin
- Sort them by color (red bin, blue bin, green bin)
- Place them gently (don't drop!)
- Avoid collisions with bin edges

### Your Reward Function

Write pseudocode for the reward:

```python
def compute_reward(state, action, next_state):
    reward = 0.0

    # Task progress
    # ...

    # Collision penalties
    # ...

    # Efficiency bonus
    # ...

    # Success bonus
    # ...

    return reward
```

### Questions

1. What makes this a "sparse" vs "dense" reward design?
2. How would you handle the multi-step nature (pick, move, place)?
3. What terminal conditions would you use?

---

## Exercise 4: Sim-to-Real Analysis

**Task**: Diagnose sim-to-real transfer failures.

### Scenario

You trained a walking policy for a quadruped robot in Isaac Gym. The policy works perfectly in simulation but has these problems on the real robot:

| Problem | Simulation | Real Robot |
|---------|-----------|------------|
| Walking speed | 1.0 m/s | 0.3 m/s |
| Stability | Perfect | Wobbles |
| Turning | Smooth | Overshoots |
| Battery impact | N/A | Slows down after 10 min |

### Analysis

For each problem, identify:
1. **Likely cause** (physics, sensors, actuators, environment)
2. **Diagnostic test** to confirm the cause
3. **Solution** (domain randomization, system ID, or other)

### Your Answers

| Problem | Cause | Test | Solution |
|---------|-------|------|----------|
| Slow walking | ? | ? | ? |
| Wobbling | ? | ? | ? |
| Turning overshoot | ? | ? | ? |
| Battery degradation | ? | ? | ? |

<details>
<summary>Click for suggested answers</summary>

| Problem | Cause | Test | Solution |
|---------|-------|------|----------|
| Slow walking | Motor strength overestimated in sim | Measure max torque | Randomize motor strength ±20% |
| Wobbling | Joint damping differs | Step response test | System ID for damping |
| Turning overshoot | Control latency | Measure cmd→response delay | Add latency to sim (10-30ms) |
| Battery degradation | Mass/inertia change | Track performance vs battery | Randomize mass; add battery-aware observation |

</details>

---

## Exercise 5: Integrated Challenge

**Task**: Design a complete learning-based navigation system.

### Scenario

You're building a robot that must:
1. Learn to navigate in simulation
2. Transfer to real hardware
3. Adapt to new environments

### System Design

Draw or describe:
1. **Training pipeline** (simulator, algorithm, observations, actions)
2. **Transfer strategy** (domain randomization parameters)
3. **Deployment architecture** (ONNX loading, safety limits)
4. **Adaptation mechanism** (how to handle new environments)

### Questions

1. What observations would you use? (sensors, proprioception, commands)
2. What action space? (velocities, positions, torques)
3. How would you ensure safety during real-world adaptation?
4. What would trigger a "fall back to classical navigation" mode?

---

## Self-Assessment Checklist

Before completing the chapter, confirm you can:

### Costmaps
- [ ] Explain the purpose of each costmap layer
- [ ] Configure global and local costmaps
- [ ] Tune inflation radius and cost scaling
- [ ] Add custom observation sources

### Planners and Behavior Trees
- [ ] Compare NavFn, Smac, and other planners
- [ ] Understand DWB and MPPI controllers
- [ ] Read and modify behavior tree XML
- [ ] Design custom recovery behaviors

### Reinforcement Learning
- [ ] Explain the RL framework (agent, environment, reward)
- [ ] Describe MDP components
- [ ] Compare PPO and SAC
- [ ] Design reward functions for robot tasks

### Sim-to-Real Transfer
- [ ] Identify sources of sim-to-real gap
- [ ] Apply domain randomization
- [ ] Load and run ONNX policies
- [ ] Implement safety limits for deployment

---

## Project Ideas

After completing these exercises, consider these projects:

1. **Custom Navigation Stack**
   - Design costmaps for your specific robot
   - Create a custom behavior tree
   - Benchmark against default Nav2

2. **RL for Mobile Robot**
   - Train a velocity controller in simulation
   - Compare to PID control
   - Attempt sim-to-real transfer

3. **Adaptive Navigation**
   - Start with classical Nav2
   - Add learning component for local planning
   - Evaluate in various environments

---

## Next Steps

**Congratulations on completing Chapter 4!**

You now have a solid foundation in:
- Robotic perception and sensors
- SLAM and map building
- Autonomous navigation with Nav2
- Reinforcement learning fundamentals
- Sim-to-real transfer techniques

Continue to the next chapter for Vision-Language-Action (VLA) integration!
