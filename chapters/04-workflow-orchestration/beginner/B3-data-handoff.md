---
id: b3_data_handoff
title: Data Handoff Between Components
tier: beginner
chapter: chapter_4_workflow
estimated_time: 1 hour
prerequisites: ["b1_pipelines_flows_triggers", "b2_state_machines_concepts"]
---

# B3: Data Handoff Between Components

## Learning Objectives

By the end of this lesson, you will be able to:
- Understand how data flows between robotic components
- Identify different data passing patterns (direct, queued, shared memory)
- Recognize when to use synchronous vs asynchronous communication
- Design data interfaces between workflow components

## Introduction

In multi-component robotic workflows, data must flow smoothly between different processing stages. Understanding how to pass data efficiently and reliably is crucial for building robust robotic systems. This lesson explores the patterns and best practices for data handoff in robotic pipelines.

Think of data handoff like a relay race - each runner (component) must pass the baton (data) smoothly to the next runner without dropping it or slowing down the team.

## What is Data Handoff?

Data handoff is the process of transferring information from one component to another in a robotic system. This can happen in several ways:

- **Direct passing**: Component A sends data directly to Component B
- **Queued passing**: Data is placed in a queue for Component B to retrieve
- **Shared memory**: Components access data from a common storage location
- **Message passing**: Data is sent as messages through a communication system (like ROS 2 topics)

### Code Example: Direct Data Passing

```python
# direct_data_passing.py
class SensorProcessor:
    """Component that processes sensor data"""
    def process_sensor_data(self, raw_data):
        """Process raw sensor data and return processed result"""
        # Simple processing: filter out values below threshold
        threshold = 1.0
        filtered_data = [x for x in raw_data if x >= threshold]
        return filtered_data

class PathPlanner:
    """Component that plans paths based on sensor data"""
    def plan_path(self, sensor_data):
        """Plan a path based on processed sensor data"""
        if len(sensor_data) == 0:
            return "NO_OBSTACLES_CLEAR_PATH"
        elif min(sensor_data) < 2.0:
            return "OBSTACLES_DETECTED_AVOID"
        else:
            return "SAFE_DISTANCE_PROCEED"

# Direct data handoff example
sensor_processor = SensorProcessor()
path_planner = PathPlanner()

# Simulate sensor reading
raw_sensor_data = [0.5, 1.5, 2.0, 0.8, 3.0]

# Component 1 processes data
processed_data = sensor_processor.process_sensor_data(raw_sensor_data)
print(f"Processed data: {processed_data}")

# Component 2 receives processed data directly
path_decision = path_planner.plan_path(processed_data)
print(f"Path decision: {path_decision}")
```

**Expected Output**:
```
Processed data: [1.5, 2.0, 3.0]
Path decision: OBSTACLES_DETECTED_AVOID
```

## Synchronous vs Asynchronous Communication

### Synchronous Communication
In synchronous communication, the sender waits for the receiver to acknowledge receipt before continuing. This ensures data is received but can slow down the system.

**Use when:**
- Data must be processed in order
- You need confirmation of receipt
- The next step depends on the result

### Asynchronous Communication
In asynchronous communication, the sender doesn't wait for acknowledgment. This is faster but requires careful handling of timing and ordering.

**Use when:**
- High-frequency data streams
- Components can process independently
- Some data loss is acceptable

### Code Example: Synchronous vs Asynchronous

```python
# sync_vs_async.py
import time

class SynchronousProcessor:
    """Processes data synchronously - waits for completion"""
    def process(self, data):
        """Process data and return result immediately"""
        print(f"[SYNC] Processing: {data}")
        time.sleep(0.1)  # Simulate processing time
        result = data * 2
        print(f"[SYNC] Result: {result}")
        return result

class AsynchronousProcessor:
    """Processes data asynchronously - doesn't wait"""
    def __init__(self):
        self.pending_data = []

    def submit(self, data):
        """Submit data for processing (non-blocking)"""
        print(f"[ASYNC] Submitted: {data}")
        self.pending_data.append(data)

    def process_pending(self):
        """Process all pending data"""
        results = []
        for data in self.pending_data:
            print(f"[ASYNC] Processing: {data}")
            time.sleep(0.1)  # Simulate processing time
            result = data * 2
            results.append(result)
        self.pending_data = []
        return results

# Synchronous example
print("=== Synchronous Processing ===")
sync_proc = SynchronousProcessor()
start_time = time.time()
result1 = sync_proc.process(5)
result2 = sync_proc.process(10)
result3 = sync_proc.process(15)
sync_time = time.time() - start_time
print(f"Synchronous total time: {sync_time:.2f}s\n")

# Asynchronous example
print("=== Asynchronous Processing ===")
async_proc = AsynchronousProcessor()
start_time = time.time()
async_proc.submit(5)
async_proc.submit(10)
async_proc.submit(15)
submit_time = time.time() - start_time
print(f"Submission time: {submit_time:.2f}s")

results = async_proc.process_pending()
total_time = time.time() - start_time
print(f"Results: {results}")
print(f"Asynchronous total time: {total_time:.2f}s")
```

**Expected Output**:
```
=== Synchronous Processing ===
[SYNC] Processing: 5
[SYNC] Result: 10
[SYNC] Processing: 10
[SYNC] Result: 20
[SYNC] Processing: 15
[SYNC] Result: 30
Synchronous total time: 0.30s

=== Asynchronous Processing ===
[ASYNC] Submitted: 5
[ASYNC] Submitted: 10
[ASYNC] Submitted: 15
Submission time: 0.00s
[ASYNC] Processing: 5
[ASYNC] Processing: 10
[ASYNC] Processing: 15
Results: [10, 20, 30]
Asynchronous total time: 0.30s
```

## Data Passing Patterns

### Pattern 1: Pipeline (Sequential)
Data flows through components in sequence, with each component transforming the data.

```
Sensor → Filter → Analyzer → Decision Maker → Actuator
```

**Best for:** Sequential processing where each step depends on the previous

### Pattern 2: Fan-Out (Broadcast)
One component sends data to multiple receivers simultaneously.

```
        → Component A
Sensor  → Component B
        → Component C
```

**Best for:** Multiple components need the same data (e.g., multiple algorithms analyzing the same sensor data)

### Pattern 3: Fan-In (Aggregation)
Multiple components send data to one receiver that combines them.

```
Sensor A →
Sensor B → Data Fusion → Decision
Sensor C →
```

**Best for:** Sensor fusion, combining multiple data sources

### Code Example: Fan-Out Pattern

```python
# fan_out_pattern.py
class DataBroadcaster:
    """Broadcasts data to multiple subscribers"""
    def __init__(self):
        self.subscribers = []

    def subscribe(self, subscriber):
        """Add a subscriber to receive broadcasts"""
        self.subscribers.append(subscriber)

    def broadcast(self, data):
        """Send data to all subscribers"""
        print(f"Broadcasting: {data}")
        for subscriber in self.subscribers:
            subscriber.receive(data)

class ObstacleDetector:
    """Subscriber that detects obstacles"""
    def receive(self, sensor_data):
        obstacles = [x for x in sensor_data if x < 1.0]
        print(f"  [ObstacleDetector] Found {len(obstacles)} obstacles")

class PathQualityAnalyzer:
    """Subscriber that analyzes path quality"""
    def receive(self, sensor_data):
        avg_distance = sum(sensor_data) / len(sensor_data) if sensor_data else 0
        print(f"  [PathQualityAnalyzer] Average clearance: {avg_distance:.2f}m")

class SafetyMonitor:
    """Subscriber that monitors safety"""
    def receive(self, sensor_data):
        min_distance = min(sensor_data) if sensor_data else float('inf')
        safety_status = "SAFE" if min_distance > 0.5 else "UNSAFE"
        print(f"  [SafetyMonitor] Status: {safety_status}")

# Fan-out example
broadcaster = DataBroadcaster()
broadcaster.subscribe(ObstacleDetector())
broadcaster.subscribe(PathQualityAnalyzer())
broadcaster.subscribe(SafetyMonitor())

# Broadcast sensor data to all subscribers
sensor_reading = [0.3, 1.5, 2.0, 0.8, 1.2]
broadcaster.broadcast(sensor_reading)
```

**Expected Output**:
```
Broadcasting: [0.3, 1.5, 2.0, 0.8, 1.2]
  [ObstacleDetector] Found 2 obstacles
  [PathQualityAnalyzer] Average clearance: 1.16m
  [SafetyMonitor] Status: UNSAFE
```

## Data Buffering and Queues

When components process at different speeds, buffering helps prevent data loss.

### Code Example: Data Queue

```python
# data_queue.py
from collections import deque

class DataQueue:
    """Simple queue for buffering data between components"""
    def __init__(self, max_size=10):
        self.queue = deque(maxlen=max_size)

    def enqueue(self, data):
        """Add data to queue"""
        if len(self.queue) >= self.queue.maxlen:
            print(f"  [Queue] Warning: Queue full, oldest data will be dropped")
        self.queue.append(data)
        print(f"  [Queue] Enqueued: {data}, Queue size: {len(self.queue)}")

    def dequeue(self):
        """Remove and return data from queue"""
        if len(self.queue) == 0:
            print(f"  [Queue] Warning: Queue empty")
            return None
        data = self.queue.popleft()
        print(f"  [Queue] Dequeued: {data}, Queue size: {len(self.queue)}")
        return data

    def is_empty(self):
        return len(self.queue) == 0

# Queue example
print("=== Producer-Consumer with Queue ===")
queue = DataQueue(max_size=3)

# Producer adds data
print("\nProducer adding data:")
queue.enqueue("data_1")
queue.enqueue("data_2")
queue.enqueue("data_3")

# Consumer processes data
print("\nConsumer processing data:")
while not queue.is_empty():
    data = queue.dequeue()
    print(f"  Processing: {data}")

# Producer adds more data than queue can hold
print("\nProducer overwhelming queue:")
queue.enqueue("data_4")
queue.enqueue("data_5")
queue.enqueue("data_6")
queue.enqueue("data_7")  # This will cause oldest to be dropped
```

**Expected Output**:
```
=== Producer-Consumer with Queue ===

Producer adding data:
  [Queue] Enqueued: data_1, Queue size: 1
  [Queue] Enqueued: data_2, Queue size: 2
  [Queue] Enqueued: data_3, Queue size: 3

Consumer processing data:
  [Queue] Dequeued: data_1, Queue size: 2
  Processing: data_1
  [Queue] Dequeued: data_2, Queue size: 1
  Processing: data_2
  [Queue] Dequeued: data_3, Queue size: 0
  Processing: data_3

Producer overwhelming queue:
  [Queue] Enqueued: data_4, Queue size: 1
  [Queue] Enqueued: data_5, Queue size: 2
  [Queue] Enqueued: data_6, Queue size: 3
  [Queue] Warning: Queue full, oldest data will be dropped
  [Queue] Enqueued: data_7, Queue size: 3
```

## Diagrams

![Data Handoff Patterns](../diagrams/data-handoff-patterns.svg)
*Common data passing patterns in robotic workflows*

## Hardware Notes

> **Simulation vs. Real Hardware**: In simulation, data handoff is instantaneous and reliable. In real hardware, you must account for network latency, message loss, and timing jitter. Real systems need buffering, timeout handling, and data validation to ensure robust communication.

## Summary

- Data handoff is the process of transferring information between workflow components
- Synchronous communication waits for acknowledgment; asynchronous doesn't
- Common patterns include pipeline (sequential), fan-out (broadcast), and fan-in (aggregation)
- Buffering and queues help handle components that process at different speeds
- Choose the right pattern based on your workflow requirements

## AI-Assisted Learning

<details>
<summary>Ask the AI Assistant</summary>

### Conceptual Questions
- When should you use synchronous vs asynchronous communication?
- What are the tradeoffs between different data passing patterns?

### Debugging Help
- How do you debug data loss in a pipeline?
- What happens when a queue overflows?

### Extension Ideas
- How would you implement priority queues for urgent data?
- What would a hybrid synchronous-asynchronous system look like?

</details>

## Exercises

1. **Data Flow Design** (Beginner)
   - Design a data flow for a robot that uses camera, LIDAR, and IMU sensors
   - Identify which components need which data
   - Acceptance Criteria: Clear diagram showing data flow between all components

2. **Queue Implementation** (Intermediate)
   - Implement a priority queue where safety-critical data is processed first
   - Test with mixed priority data
   - Acceptance Criteria: High-priority data is always processed before low-priority

## Next Steps

You've completed the Beginner tier! You now understand:
- Pipelines, flows, and triggers
- State machines and their components
- Data handoff between components

Continue to the [Intermediate Tier](../intermediate/README.md) to implement these concepts in ROS 2.
