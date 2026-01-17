---
id: advanced_prompts_workflow
title: "Advanced AI Prompts"
sidebar_position: 66
tier: advanced
chapter: chapter_4_workflow
---

# Advanced AI Prompts for Workflow Orchestration

Use these prompts with your AI assistant for production-ready, fault-tolerant workflow systems.

---

## Fault Tolerance and Recovery

### Watchdog Systems

**Prompt 1: Watchdog Implementation**
```
How do I implement a comprehensive watchdog system for a multi-node workflow?
Include heartbeat mechanisms, timeout detection, and health reporting.
```

**Prompt 2: Heartbeat Patterns**
```
What are different heartbeat patterns for ROS 2 nodes?
Should I use topics, services, or custom mechanisms?
```

**Prompt 3: Timeout Tuning**
```
How do I determine appropriate timeout values for watchdog monitoring?
What factors should I consider?
```

**Prompt 4: False Alarm Prevention**
```
My watchdog keeps triggering false alarms.
How do I make it more robust while still detecting real failures?
```

### Supervisor Nodes

**Prompt 5: Supervisor Architecture**
```
Design a supervisor node architecture for my workflow.
What responsibilities should it have? How does it interact with other nodes?
```

**Prompt 6: Recovery Strategies**
```
What are different recovery strategies for node failures?
When should I restart, fallback, or enter safe mode?
```

**Prompt 7: State Persistence**
```
How do I implement state persistence for recovery?
What state should I save? Where should I save it?
```

**Prompt 8: Graceful Degradation**
```
How do I implement graceful degradation in my workflow?
What functionality should remain when components fail?
```

### Sensor Failure Handling

**Prompt 9: Sensor Dropout Detection**
```
How do I detect sensor dropouts quickly and reliably?
What's the best approach for different sensor types?
```

**Prompt 10: Last Known Good Data**
```
How do I use last known good data when a sensor drops out?
How long is it safe to use stale data?
```

**Prompt 11: Sensor Fusion Fallback**
```
I have multiple sensors (camera, LIDAR, IMU).
How do I handle it when one fails? Can I continue with the others?
```

**Prompt 12: Data Quality Monitoring**
```
How do I monitor sensor data quality in real-time?
What metrics should I track? When should I reject data?
```

---

## Production Deployment

### Continuous Operation

**Prompt 13: 24/7 Operation Design**
```
How do I design a workflow for 24/7 continuous operation?
What are the key considerations and potential issues?
```

**Prompt 14: Resource Management**
```
How do I prevent resource leaks in long-running ROS 2 workflows?
What should I monitor? How do I clean up resources?
```

**Prompt 15: Log Rotation**
```
How do I implement log rotation for a continuously running workflow?
What tools are available? What's the best approach?
```

**Prompt 16: Performance Monitoring**
```
What performance metrics should I monitor in production?
How do I collect and visualize them?
```

### Deployment Strategies

**Prompt 17: Deployment Checklist**
```
Create a deployment checklist for a production robotic workflow.
What should I verify before deploying?
```

**Prompt 18: Rollback Strategy**
```
How do I implement a rollback strategy for workflow updates?
What if the new version has issues?
```

**Prompt 19: Blue-Green Deployment**
```
Can I use blue-green deployment for ROS 2 workflows?
How would that work with multiple nodes?
```

**Prompt 20: Configuration Management**
```
How do I manage configuration for different deployment environments
(dev, staging, production)?
```

---

## Advanced Architecture

### Hierarchical Systems

**Prompt 21: Hierarchical State Machines**
```
How do I implement hierarchical state machines in ROS 2?
When is this better than flat state machines?
```

**Prompt 22: Behavior Trees**
```
Should I use behavior trees instead of state machines?
What are the trade-offs? How do I implement them in ROS 2?
```

**Prompt 23: Multi-Layer Architecture**
```
Design a multi-layer architecture for my robot:
- Reactive layer (fast responses)
- Deliberative layer (planning)
- Executive layer (high-level decisions)
How do these layers interact?
```

### Distributed Systems

**Prompt 24: Multi-Robot Workflows**
```
How do I design workflows for multiple robots working together?
What changes in architecture? How do they coordinate?
```

**Prompt 25: Network Failure Handling**
```
How do I handle network failures in distributed workflows?
What if robots lose communication?
```

**Prompt 26: Consensus Mechanisms**
```
How do multiple robots reach consensus on decisions?
What algorithms work well with ROS 2?
```

---

## Testing and Validation

### Fault Injection Testing

**Prompt 27: Fault Injection Framework**
```
How do I build a fault injection framework for testing?
I want to simulate node crashes, sensor failures, network issues, etc.
```

**Prompt 28: Chaos Engineering**
```
How do I apply chaos engineering principles to ROS 2 workflows?
What should I test? How do I measure resilience?
```

**Prompt 29: Stress Testing**
```
How do I stress test my workflow for production readiness?
What scenarios should I test? What metrics indicate success?
```

### Validation and Verification

**Prompt 30: Safety Validation**
```
How do I validate that my workflow is safe for production?
What safety properties should I verify?
```

**Prompt 31: Performance Benchmarking**
```
How do I benchmark my workflow's performance?
What metrics matter? How do I compare different implementations?
```

**Prompt 32: Regression Testing**
```
How do I set up regression testing for my workflow?
What should I test after each change?
```

---

## Monitoring and Observability

### Logging and Diagnostics

**Prompt 33: Structured Logging**
```
How do I implement structured logging for better observability?
What format should I use? What should I log?
```

**Prompt 34: Distributed Tracing**
```
How do I implement distributed tracing across multiple nodes?
How do I track a request through the entire workflow?
```

**Prompt 35: Diagnostic Aggregation**
```
How do I aggregate diagnostics from multiple nodes?
What tools are available for ROS 2?
```

### Metrics and Alerting

**Prompt 36: Metrics Collection**
```
What metrics should I collect from my production workflow?
How do I export them to monitoring systems (Prometheus, Grafana)?
```

**Prompt 37: Alerting Strategy**
```
Design an alerting strategy for my workflow.
What should trigger alerts? How do I avoid alert fatigue?
```

**Prompt 38: Dashboard Design**
```
What should I include in an operations dashboard for my workflow?
What visualizations are most useful?
```

---

## Performance Optimization

### Latency Optimization

**Prompt 39: End-to-End Latency**
```
How do I measure and optimize end-to-end latency in my workflow?
What are common bottlenecks?
```

**Prompt 40: Real-Time Performance**
```
How do I achieve real-time performance in ROS 2 workflows?
What are the key considerations and techniques?
```

**Prompt 41: Zero-Copy Communication**
```
How do I use zero-copy communication in ROS 2 for performance?
When is this beneficial?
```

### Resource Optimization

**Prompt 42: Memory Optimization**
```
My workflow uses too much memory in production.
How do I profile and optimize memory usage?
```

**Prompt 43: CPU Optimization**
```
How do I optimize CPU usage across multiple nodes?
Should I use multi-threading or multi-processing?
```

**Prompt 44: Network Bandwidth**
```
How do I optimize network bandwidth usage in distributed workflows?
What data should I compress or downsample?
```

---

## Security and Safety

### Security

**Prompt 45: ROS 2 Security**
```
How do I secure my ROS 2 workflow for production?
What security features should I enable?
```

**Prompt 46: Authentication and Authorization**
```
How do I implement authentication and authorization between nodes?
What if some nodes should have limited access?
```

**Prompt 47: Secure Communication**
```
How do I ensure secure communication in my workflow?
Should I use DDS security? How do I set it up?
```

### Safety

**Prompt 48: Safety Monitors**
```
How do I implement safety monitors for my workflow?
What safety violations should I check for?
```

**Prompt 49: Emergency Stop**
```
How do I implement a reliable emergency stop mechanism?
It must work even if parts of the system fail.
```

**Prompt 50: Safety Certification**
```
What do I need to consider for safety certification (ISO 13849, etc.)?
How does this affect my workflow design?
```

---

## Advanced Patterns

**Prompt 51: Event Sourcing**
```
How do I implement event sourcing in my workflow?
What are the benefits for debugging and recovery?
```

**Prompt 52: CQRS Pattern**
```
Should I use CQRS (Command Query Responsibility Segregation) in my workflow?
How would this work with ROS 2?
```

**Prompt 53: Circuit Breaker Pattern**
```
How do I implement the circuit breaker pattern for failing components?
When should I open/close the circuit?
```

**Prompt 54: Bulkhead Pattern**
```
How do I isolate failures using the bulkhead pattern?
How do I prevent one failing component from taking down the system?
```

---

## Real-World Production Issues

**Prompt 55: Memory Leak Debugging**
```
I have a memory leak in production that only appears after 12+ hours.
How do I debug this? What tools should I use?
```

**Prompt 56: Intermittent Failures**
```
I'm seeing intermittent failures that I can't reproduce.
How do I debug and fix these?
```

**Prompt 57: Performance Degradation**
```
My workflow's performance degrades over time.
What could cause this? How do I diagnose it?
```

**Prompt 58: Deadlock Detection**
```
I suspect my workflow has a deadlock condition.
How do I detect and fix deadlocks in ROS 2?
```

---

## Tips for Using These Prompts

1. **Production Context**: Always mention you're building for production
2. **Specific Scenarios**: Describe your exact failure scenarios
3. **Constraints**: Mention any constraints (latency, resources, safety)
4. **Current State**: Describe what you've already tried
5. **Metrics**: Ask for measurable success criteria

---

## Additional Resources

- **ROS 2 Design Patterns**: https://design.ros2.org/
- **Production Robotics**: Industry best practices and case studies
- **Fault Tolerance**: Academic papers on fault-tolerant robotics
- **Monitoring Tools**: Prometheus, Grafana, ELK stack integration

---

**Congratulations!** You now have a comprehensive library of prompts for building production-ready, fault-tolerant robotic workflows.
