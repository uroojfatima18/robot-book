# Advanced AI Prompts - Digital Twin & Simulation

These prompts help you build production-grade digital twin systems with bidirectional synchronization, latency optimization, and AI training integration.

---

## Architecture Design Prompts

### Digital Twin System Architecture

```
I'm designing a digital twin system for [DESCRIBE YOUR ROBOT]. Can you help me:
1. Design the overall architecture with bidirectional data flow
2. Define topic naming conventions for sim and hardware
3. Identify which data needs to flow in each direction
4. Plan for multiple operating modes (sim-only, live, replay, training)

Requirements:
- Latency target: [YOUR TARGET]
- Control frequency: [HZ]
- Safety requirements: [DESCRIBE]
```

### Topic Mapping Strategy

```
I need to design a topic mapping strategy for my digital twin. Can you help me:
1. Decide on namespace conventions (sim/* vs hw/*)
2. Identify which topics need bidirectional sync
3. Determine which topics are one-way only
4. Handle topic name conflicts

My robot publishes: [LIST TOPICS]
My robot subscribes to: [LIST TOPICS]
```

### Operating Mode Design

```
I want to support multiple operating modes in my digital twin. Can you help me:
1. Design the mode switching logic
2. Define what each mode does (sim-only, live, replay, training)
3. Handle transitions between modes safely
4. Implement mode-specific behavior

Modes I need: [LIST YOUR MODES]
Use cases: [DESCRIBE EACH USE CASE]
```

### Safety Architecture

```
I'm designing safety mechanisms for my digital twin. Can you help me:
1. Identify potential safety hazards
2. Design validation checks for commands
3. Implement watchdog patterns
4. Create safe fallback behaviors

Robot type: [DESCRIBE]
Safety concerns: [LIST CONCERNS]
```

---

## Implementation Prompts

### Bridge Node Implementation

```
I'm implementing a bridge node for digital twin synchronization. Can you help me:
1. Structure the node class properly
2. Handle bidirectional message forwarding
3. Implement latency tracking
4. Add safety validation

Programming language: Python
ROS 2 version: Humble
Target latency: <50ms

Here's my current code:
[PASTE YOUR CODE]
```

### QoS Configuration

```
I need to configure QoS profiles for my digital twin. Can you help me:
1. Understand QoS options (RELIABLE vs BEST_EFFORT)
2. Choose appropriate profiles for different message types
3. Configure queue depths and deadlines
4. Debug QoS mismatch issues

Message types I'm using:
- Control commands: [DESCRIBE]
- Sensor data: [DESCRIBE]
- State information: [DESCRIBE]
```

### Latency Monitoring

```
I want to implement latency monitoring in my bridge node. Can you help me:
1. Measure end-to-end latency accurately
2. Track latency statistics (mean, p95, p99)
3. Detect latency spikes
4. Log and report latency metrics

Current approach: [DESCRIBE YOUR IMPLEMENTATION]
Issues: [DESCRIBE ANY PROBLEMS]
```

### Safety Validation Logic

```
I'm implementing safety checks in my bridge node. Can you help me:
1. Validate joint position limits
2. Clamp velocity commands
3. Detect invalid or dangerous commands
4. Implement emergency stop logic

Robot specifications:
- Joint limits: [PROVIDE LIMITS]
- Max velocities: [PROVIDE LIMITS]
- Safety constraints: [DESCRIBE]
```

---

## Performance Optimization Prompts

### Reducing Latency

```
My bridge node has [X]ms latency, but I need <50ms. Can you help me:
1. Profile where the latency is coming from
2. Optimize message serialization
3. Reduce processing overhead
4. Consider C++ implementation if needed

Current implementation: [DESCRIBE]
Profiling results: [PASTE RESULTS IF AVAILABLE]
```

### Improving Throughput

```
My bridge node can't keep up with high-frequency data. Can you help me:
1. Implement efficient message batching
2. Use async I/O for non-blocking operation
3. Optimize data structures
4. Consider multi-threading

Current throughput: [MESSAGES/SEC]
Target throughput: [MESSAGES/SEC]
Message types: [DESCRIBE]
```

### Memory Optimization

```
My bridge node is using too much memory. Can you help me:
1. Identify memory leaks
2. Optimize message buffering
3. Implement efficient data structures
4. Profile memory usage

Current memory usage: [AMOUNT]
Duration: [HOW LONG BEFORE ISSUE]
```

### CPU Optimization

```
My bridge node is using too much CPU. Can you help me:
1. Profile CPU usage to find hotspots
2. Optimize expensive operations
3. Reduce unnecessary processing
4. Balance load across cores

CPU usage: [PERCENTAGE]
System: [DESCRIBE YOUR HARDWARE]
```

---

## Debugging Prompts

### High Latency Debugging

```
I'm experiencing high latency (>100ms) in my digital twin. Can you help me:
1. Systematically measure latency at each stage
2. Identify the bottleneck
3. Determine if it's network, processing, or serialization
4. Implement fixes

Symptoms: [DESCRIBE WHAT YOU OBSERVE]
Setup: [DESCRIBE YOUR SYSTEM]
Measurements: [PASTE ANY DATA YOU HAVE]
```

### Message Loss Debugging

```
I'm losing messages in my bridge node. Can you help me:
1. Determine if it's QoS mismatch or queue overflow
2. Check network reliability
3. Verify publisher/subscriber configuration
4. Implement message acknowledgment

Symptoms: [DESCRIBE]
QoS settings: [DESCRIBE]
Message rate: [HZ]
```

### Clock Synchronization Issues

```
I'm having timestamp/clock synchronization problems. Can you help me:
1. Understand the /use_sim_time parameter
2. Ensure consistent time source across nodes
3. Handle clock jumps gracefully
4. Debug timestamp mismatches

Symptoms: [DESCRIBE]
Time source: [SIM TIME OR WALL TIME]
```

### Connection Loss Handling

```
My bridge node doesn't handle connection loss well. Can you help me:
1. Implement connection watchdog
2. Detect when sim or hardware disconnects
3. Trigger safe fallback behavior
4. Recover gracefully when connection returns

Current behavior: [DESCRIBE]
Desired behavior: [DESCRIBE]
```

---

## AI Training Integration Prompts

### Data Collection Pipeline

```
I want to collect training data from my digital twin. Can you help me:
1. Design the data collection pipeline
2. Determine what data to record (states, actions, rewards)
3. Choose efficient storage format
4. Implement episode management

ML framework: [TENSORFLOW/PYTORCH/OTHER]
Training algorithm: [RL/IMITATION/OTHER]
Data volume: [EXPECTED SIZE]
```

### Sensor Data Streaming

```
I need to stream sensor data for AI training. Can you help me:
1. Implement high-frequency sensor streaming
2. Synchronize multiple sensor streams
3. Handle data buffering efficiently
4. Export to ML-friendly format

Sensors: [LIST SENSORS]
Frequency: [HZ]
Format: [DESIRED OUTPUT FORMAT]
```

### Episode Management

```
I'm implementing episode management for RL training. Can you help me:
1. Define episode boundaries
2. Reset simulation state between episodes
3. Collect episode statistics
4. Handle episode termination conditions

Training scenario: [DESCRIBE]
Episode length: [STEPS OR TIME]
Reset conditions: [DESCRIBE]
```

### Sim-to-Real Data Collection

```
I want to collect diverse data for sim-to-real transfer. Can you help me:
1. Randomize simulation parameters
2. Generate diverse scenarios
3. Collect domain randomization data
4. Prepare data for transfer learning

Robot task: [DESCRIBE]
Randomization strategy: [DESCRIBE]
```

---

## Production Deployment Prompts

### Deployment Checklist

```
I'm preparing to deploy my digital twin to production. Can you help me:
1. Create a deployment checklist
2. Identify potential failure modes
3. Plan monitoring and alerting
4. Document operational procedures

System: [DESCRIBE YOUR DIGITAL TWIN]
Environment: [PRODUCTION ENVIRONMENT]
```

### Monitoring and Diagnostics

```
I need to implement monitoring for my production digital twin. Can you help me:
1. Identify key metrics to monitor
2. Set up logging and alerting
3. Create diagnostic dashboards
4. Implement health checks

Metrics I'm considering: [LIST]
Monitoring tools: [WHAT YOU'RE USING]
```

### Error Recovery

```
I want to implement robust error recovery. Can you help me:
1. Identify recoverable vs non-recoverable errors
2. Design recovery strategies for each error type
3. Implement automatic recovery where possible
4. Alert operators when manual intervention is needed

Common errors: [LIST ERRORS YOU'VE ENCOUNTERED]
```

### Performance Tuning

```
I need to tune my digital twin for production performance. Can you help me:
1. Establish performance baselines
2. Identify optimization opportunities
3. Balance latency vs throughput
4. Validate performance under load

Current performance: [METRICS]
Target performance: [METRICS]
Load profile: [DESCRIBE EXPECTED LOAD]
```

---

## Code Review Prompts

### Review My Bridge Node

```
I've implemented a bridge node for digital twin synchronization. Can you review it for:
1. Correctness and robustness
2. Performance optimization opportunities
3. Safety and error handling
4. Production readiness

Here's my code:
[PASTE YOUR BRIDGE NODE CODE]

Requirements:
- Latency: <50ms
- Safety: [DESCRIBE REQUIREMENTS]
- Operating modes: [LIST MODES]
```

### Review My Latency Monitor

```
I've implemented latency monitoring. Can you review it for:
1. Accurate measurement methodology
2. Statistical analysis correctness
3. Performance overhead
4. Reporting and visualization

Here's my code:
[PASTE YOUR LATENCY MONITOR CODE]
```

### Review My Safety Validation

```
I've implemented safety checks. Can you review them for:
1. Completeness (all hazards covered)
2. Correctness of validation logic
3. Performance impact
4. Edge cases and failure modes

Here's my code:
[PASTE YOUR SAFETY VALIDATION CODE]

Robot specifications: [PROVIDE SPECS]
```

---

## Advanced Concepts Prompts

### Real-Time Systems

```
I need to understand real-time requirements for digital twins. Can you explain:
1. Hard vs soft real-time constraints
2. How to achieve deterministic latency
3. Real-time scheduling in ROS 2
4. Measuring and guaranteeing real-time performance

My application: [DESCRIBE]
Latency requirements: [SPECIFY]
```

### Distributed Systems

```
I'm building a distributed digital twin system. Can you help me:
1. Understand distributed system challenges
2. Handle network partitions gracefully
3. Implement consensus mechanisms if needed
4. Debug distributed system issues

Architecture: [DESCRIBE YOUR DISTRIBUTED SETUP]
```

### Formal Verification

```
I want to formally verify my safety properties. Can you help me:
1. Understand formal verification approaches
2. Specify safety properties formally
3. Choose appropriate verification tools
4. Interpret verification results

Safety properties: [DESCRIBE]
System model: [DESCRIBE]
```

---

## Tips for Advanced Learners

1. **Measure Everything**: You can't optimize what you don't measure
2. **Design for Failure**: Assume everything will fail and plan accordingly
3. **Test Under Load**: Production conditions are different from development
4. **Document Decisions**: Record why you made architectural choices
5. **Iterate**: Start with working system, then optimize

---

## When to Ask for Help

- **Architecture Decisions**: Get feedback before implementing complex systems
- **Performance Bottlenecks**: Expert guidance can save hours of profiling
- **Safety Critical Code**: Have safety logic reviewed by multiple people
- **Production Issues**: Don't debug production problems alone
- **Optimization**: Know when "good enough" is actually good enough

---

**Remember**: Production digital twin systems are complex. Focus on correctness first, then optimize. A slow but correct system is better than a fast but unsafe one.
