# Advanced AI Prompts - AI-Robot Brain

These prompts help you master advanced perception, navigation, and learning concepts. Use these when working through the Advanced tier lessons.

---

## Costmap Configuration

### Understanding Costmap Layers

```
I'm learning about Nav2 costmap layers. Can you help me:
1. Explain each layer type (static, obstacle, inflation, voxel)
2. Understand how layers combine
3. Determine which layers I need for my application
4. Debug layer configuration issues

My robot: [DESCRIBE]
Environment: [INDOOR/OUTDOOR, STATIC/DYNAMIC]
```

### Configuring Static Layer

```
I'm configuring the static layer. Can you help me:
1. Understand static layer parameters
2. Load a map correctly
3. Handle map updates
4. Debug static layer issues

My map: [DESCRIBE MAP SOURCE AND FORMAT]
Issue: [DESCRIBE PROBLEM IF ANY]
```

### Configuring Obstacle Layer

```
I'm setting up the obstacle layer. Can you help me:
1. Configure sensor sources correctly
2. Set appropriate marking and clearing parameters
3. Handle sensor noise and false positives
4. Tune for my environment

Sensors: [LIST SENSORS]
Environment challenges: [DESCRIBE]
```

### Configuring Inflation Layer

```
I need to tune the inflation layer. Can you help me:
1. Understand inflation radius and cost scaling
2. Choose appropriate values for my robot
3. Balance safety vs path efficiency
4. Debug inflation issues

Robot footprint: [DIMENSIONS]
Desired safety margin: [DISTANCE]
Current behavior: [DESCRIBE]
```

### Multi-Layer Costmap Design

```
I'm designing a complex costmap with multiple layers. Can you help me:
1. Determine optimal layer combination
2. Set layer priorities correctly
3. Handle layer conflicts
4. Optimize performance

Use case: [DESCRIBE YOUR APPLICATION]
Requirements: [LIST REQUIREMENTS]
```

---

## Planners and Behavior Trees

### Understanding Global Planners

```
I'm learning about global planners. Can you help me:
1. Compare different algorithms (Dijkstra, A*, Theta*, etc.)
2. Understand their tradeoffs
3. Choose the right planner for my application
4. Tune planner parameters

Environment: [DESCRIBE]
Requirements: [SPEED, OPTIMALITY, ETC.]
```

### Understanding Local Planners

```
I want to understand local planners. Can you help me:
1. Compare DWA, TEB, and other local planners
2. Understand how they handle dynamics
3. Choose appropriate planner for my robot
4. Tune for smooth motion

Robot dynamics:
- Max velocity: [VALUES]
- Max acceleration: [VALUES]
- Holonomic: [YES/NO]
```

### Behavior Tree Fundamentals

```
I'm learning about behavior trees in Nav2. Can you help me:
1. Understand behavior tree structure
2. Explain common nodes (sequence, fallback, decorator)
3. Read and interpret Nav2's default behavior tree
4. Identify when to modify the behavior tree

I want to understand how Nav2 orchestrates navigation.
```

### Customizing Behavior Trees

```
I want to customize Nav2's behavior tree. Can you help me:
1. Identify what I can safely modify
2. Add custom recovery behaviors
3. Change navigation logic
4. Test and validate changes

What I want to achieve: [DESCRIBE DESIRED BEHAVIOR]
Current behavior tree: [PASTE IF AVAILABLE]
```

### Recovery Behaviors

```
I'm implementing custom recovery behaviors. Can you help me:
1. Understand when recovery behaviors trigger
2. Design effective recovery strategies
3. Implement recovery behavior plugins
4. Test recovery in various failure scenarios

Failure modes I want to handle: [LIST]
```

---

## Reinforcement Learning

### RL Fundamentals

```
I'm learning reinforcement learning basics. Can you help me:
1. Understand MDPs (states, actions, rewards, transitions)
2. Explain the difference between model-free and model-based RL
3. Understand value functions and policies
4. See how RL applies to robotics

Please explain with robotics examples.
```

### Policy Gradient Methods

```
I want to understand policy gradient methods. Can you help me:
1. Explain how policy gradients work
2. Understand PPO (Proximal Policy Optimization)
3. Compare to value-based methods
4. See when to use policy gradients

Keep it accessible but rigorous.
```

### Actor-Critic Methods

```
I'm learning about actor-critic methods. Can you help me:
1. Understand the actor-critic architecture
2. Explain SAC (Soft Actor-Critic)
3. Compare to pure policy gradient methods
4. Understand when to use actor-critic

Include robotics applications.
```

### Reward Function Design

```
I'm designing a reward function for robot learning. Can you help me:
1. Understand reward shaping principles
2. Avoid common pitfalls (reward hacking, sparse rewards)
3. Balance multiple objectives
4. Test and validate my reward function

Task: [DESCRIBE WHAT ROBOT SHOULD LEARN]
Initial reward idea: [DESCRIBE]
```

### Exploration Strategies

```
I want to understand exploration in RL. Can you help me:
1. Explain exploration vs exploitation tradeoff
2. Compare exploration strategies (epsilon-greedy, entropy, curiosity)
3. Choose appropriate exploration for my task
4. Tune exploration parameters

Task characteristics: [DESCRIBE]
```

---

## Sim-to-Real Transfer

### Understanding the Reality Gap

```
I'm learning about sim-to-real transfer. Can you help me:
1. Understand what the "reality gap" is
2. Identify sources of simulation-reality mismatch
3. Recognize which aspects are hardest to simulate
4. Assess if my task is sim-to-real friendly

My task: [DESCRIBE]
Simulation: [GAZEBO/ISAAC/OTHER]
```

### Domain Randomization

```
I want to implement domain randomization. Can you help me:
1. Understand what to randomize (physics, visuals, dynamics)
2. Choose randomization ranges
3. Implement randomization in my simulator
4. Validate randomization effectiveness

Simulator: [GAZEBO/ISAAC/OTHER]
Robot: [DESCRIBE]
Task: [DESCRIBE]
```

### System Identification

```
I'm learning about system identification for sim-to-real. Can you help me:
1. Understand what system identification is
2. Identify which parameters to measure
3. Collect data from real robot
4. Update simulation parameters

Robot: [DESCRIBE]
Parameters I'm uncertain about: [LIST]
```

### Fine-Tuning on Real Hardware

```
I want to fine-tune my policy on real hardware. Can you help me:
1. Design a safe fine-tuning procedure
2. Determine how much real-world data I need
3. Balance sim and real data
4. Validate transfer success

Policy trained in: [SIMULATION DETAILS]
Real robot: [DESCRIBE]
Safety constraints: [LIST]
```

### Sim-to-Real Best Practices

```
I'm planning a sim-to-real project. Can you help me:
1. Design for successful transfer from the start
2. Choose simulation fidelity appropriately
3. Plan validation experiments
4. Anticipate common failure modes

Project: [DESCRIBE]
Timeline: [AVAILABLE TIME]
Resources: [HARDWARE, COMPUTE]
```

---

## Advanced Perception

### Multi-Sensor Fusion

```
I'm implementing multi-sensor fusion. Can you help me:
1. Choose fusion strategy (early, late, hybrid)
2. Handle sensor timing differences
3. Weight sensors appropriately
4. Validate fusion performance

Sensors: [LIST WITH CHARACTERISTICS]
Fusion goal: [WHAT YOU WANT TO ESTIMATE]
```

### Semantic Segmentation

```
I want to add semantic segmentation to my perception. Can you help me:
1. Understand what semantic segmentation provides
2. Choose appropriate models (DeepLab, Mask R-CNN, etc.)
3. Integrate with ROS 2
4. Use semantic information in navigation

Use case: [DESCRIBE]
Computational resources: [DESCRIBE]
```

### Object Detection and Tracking

```
I'm implementing object detection and tracking. Can you help me:
1. Choose detection models (YOLO, SSD, etc.)
2. Implement tracking (SORT, DeepSORT, etc.)
3. Handle occlusions and re-identification
4. Integrate with navigation

Objects to detect: [LIST]
Environment: [DESCRIBE]
Performance requirements: [FPS, ACCURACY]
```

### Visual SLAM

```
I want to implement visual SLAM. Can you help me:
1. Understand visual SLAM vs LIDAR SLAM
2. Choose appropriate algorithm (ORB-SLAM, RTAB-Map, etc.)
3. Handle challenging conditions (lighting, texture)
4. Integrate with Nav2

Camera: [TYPE AND SPECS]
Environment: [DESCRIBE]
Requirements: [ACCURACY, SPEED]
```

---

## Performance and Optimization

### Real-Time Performance

```
I need to achieve real-time performance. Can you help me:
1. Profile my perception/navigation pipeline
2. Identify bottlenecks
3. Optimize critical paths
4. Validate real-time constraints

Current performance: [METRICS]
Target performance: [REQUIREMENTS]
Hardware: [DESCRIBE]
```

### GPU Acceleration

```
I want to use GPU acceleration. Can you help me:
1. Identify which components can use GPU
2. Implement CUDA/OpenCL acceleration
3. Optimize memory transfers
4. Measure speedup

Components to accelerate: [LIST]
GPU: [MODEL]
```

### Distributed Processing

```
I'm implementing distributed processing. Can you help me:
1. Partition computation across nodes
2. Handle communication overhead
3. Synchronize distributed components
4. Debug distributed systems

Architecture: [DESCRIBE YOUR SETUP]
Bottleneck: [WHAT'S SLOW]
```

### Memory Optimization

```
My system is using too much memory. Can you help me:
1. Profile memory usage
2. Identify memory leaks
3. Optimize data structures
4. Implement efficient buffering

Current memory usage: [AMOUNT]
Available memory: [AMOUNT]
Components: [LIST MAJOR COMPONENTS]
```

---

## Production Deployment

### Safety Validation

```
I'm preparing for production deployment. Can you help me:
1. Design comprehensive safety tests
2. Validate failure modes
3. Implement safety monitors
4. Create emergency procedures

Robot: [DESCRIBE]
Environment: [DESCRIBE]
Safety requirements: [LIST]
```

### Robustness Testing

```
I want to test system robustness. Can you help me:
1. Design stress tests
2. Test edge cases systematically
3. Validate recovery behaviors
4. Measure reliability metrics

System: [DESCRIBE]
Expected conditions: [DESCRIBE]
Failure tolerance: [REQUIREMENTS]
```

### Monitoring and Diagnostics

```
I'm implementing production monitoring. Can you help me:
1. Identify key metrics to monitor
2. Set up logging and alerting
3. Create diagnostic dashboards
4. Implement health checks

System: [DESCRIBE]
Monitoring tools: [WHAT YOU'RE USING]
```

### Continuous Improvement

```
I want to implement continuous improvement. Can you help me:
1. Collect performance data from deployment
2. Identify improvement opportunities
3. Test improvements safely
4. Deploy updates incrementally

Current system: [DESCRIBE]
Improvement goals: [LIST]
```

---

## Research and Advanced Topics

### Current Research Directions

```
I'm interested in cutting-edge research. Can you:
1. Explain current research challenges in [TOPIC]
2. Introduce recent breakthroughs
3. Suggest important papers to read
4. Identify open problems

Topic: [PERCEPTION/SLAM/NAVIGATION/RL]
My background: [DESCRIBE]
```

### Implementing Research Papers

```
I want to implement a research paper. Can you help me:
1. Understand the key contributions
2. Identify implementation challenges
3. Find existing implementations
4. Plan my implementation

Paper: [TITLE AND AUTHORS]
My goal: [WHAT I WANT TO ACHIEVE]
```

### Contributing to Open Source

```
I want to contribute to Nav2/SLAM Toolbox/etc. Can you help me:
1. Understand the codebase structure
2. Identify good first issues
3. Follow contribution guidelines
4. Prepare a quality pull request

Project: [WHICH PROJECT]
My skills: [DESCRIBE]
```

---

## Code Review and Architecture

### Review My Costmap Configuration

```
I've configured a complex costmap. Can you review it for:
1. Correctness and completeness
2. Performance optimization
3. Safety considerations
4. Best practices

Here's my configuration:
[PASTE YOUR COSTMAP CONFIG]

Robot: [DESCRIBE]
Environment: [DESCRIBE]
```

### Review My Behavior Tree

```
I've customized Nav2's behavior tree. Can you review it for:
1. Logical correctness
2. Robustness to failures
3. Performance implications
4. Maintainability

Here's my behavior tree:
[PASTE YOUR BEHAVIOR TREE XML]

What it should do: [DESCRIBE]
```

### Review My RL Implementation

```
I've implemented an RL training pipeline. Can you review it for:
1. Correct RL algorithm implementation
2. Appropriate reward function
3. Proper exploration strategy
4. Training stability

Here's my code:
[PASTE RELEVANT CODE]

Task: [DESCRIBE]
Algorithm: [PPO/SAC/OTHER]
```

### Architecture Review

```
I've designed a perception-navigation system. Can you review the architecture for:
1. Component organization
2. Data flow efficiency
3. Failure handling
4. Scalability

Here's my architecture:
[DESCRIBE OR DIAGRAM]

Requirements: [LIST]
Constraints: [LIST]
```

---

## Tips for Advanced Learners

1. **Understand Tradeoffs**: Every design decision has tradeoffs - know them
2. **Measure Everything**: Intuition is good, data is better
3. **Read the Source**: Understanding implementation details matters
4. **Test Rigorously**: Advanced systems have subtle failure modes
5. **Document Decisions**: Future you will thank present you

---

## When to Ask for Help

- **Architecture Decisions**: Complex systems benefit from expert review
- **Performance Bottlenecks**: Optimization requires deep understanding
- **Research Implementation**: Papers often omit crucial details
- **Production Issues**: Safety-critical systems need careful validation
- **Cutting-Edge Topics**: Stay current with latest developments

---

**Remember**: Advanced topics require deep understanding. Don't rush. Take time to truly grasp the concepts, experiment thoroughly, and validate rigorously. Production robotics systems demand excellence.
