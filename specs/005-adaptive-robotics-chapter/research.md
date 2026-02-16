# Research: Chapter 5 - Adaptive Robotics

**Feature**: 005-adaptive-robotics-chapter
**Date**: 2025-12-30
**Status**: Complete

---

## 1. ROS 2 Behavior Switching Patterns

### Decision
Use a state-machine approach with ROS 2 topics for behavior switching, leveraging `rclpy` lifecycle nodes.

### Rationale
- **Lifecycle nodes** provide standardized state management (configure, activate, deactivate)
- **Topic-based triggers** allow loose coupling between sensor nodes and behavior nodes
- **Parameter-based configuration** enables runtime threshold tuning without code changes

### Alternatives Considered
| Alternative | Why Rejected |
|-------------|--------------|
| BehaviorTree.CPP | Over-engineered for educational scope; adds significant learning curve |
| SMACH (ROS 1) | Deprecated; not ROS 2 native |
| Custom FSM | Reinvents standard patterns; harder to debug |

### Implementation Pattern
```python
# Simplified behavior switching pattern
class BehaviorSwitcher(Node):
    def __init__(self):
        super().__init__('behavior_switcher')
        self.current_behavior = 'idle'
        self.behaviors = {'explore': self.explore, 'avoid': self.avoid, 'idle': self.idle}

        # Hysteresis thresholds
        self.activate_threshold = self.declare_parameter('activate_threshold', 0.5).value
        self.deactivate_threshold = self.declare_parameter('deactivate_threshold', 0.7).value
```

---

## 2. TurtleBot3 Sensor Integration

### Decision
Use TurtleBot3 Burger model with LaserScan (lidar) and Odometry as primary sensor inputs.

### Rationale
- **LaserScan** provides 360-degree distance data ideal for obstacle detection
- **Odometry** enables position tracking for behavior validation
- **Standard messages** (sensor_msgs/LaserScan, nav_msgs/Odometry) ensure code portability

### Key Topics
| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `/scan` | sensor_msgs/LaserScan | Distance measurements for threshold triggers |
| `/odom` | nav_msgs/Odometry | Robot position for logging and validation |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands for behavior output |

### Gazebo Setup
```bash
# Launch TurtleBot3 in empty world
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## 3. Hysteresis Implementation

### Decision
Implement hysteresis using dual thresholds (on/off) with configurable dead band.

### Rationale
- **Prevents oscillation** when sensor values hover near threshold
- **Standard engineering pattern** teaches real-world control concepts
- **Simple implementation** suitable for educational content

### Pattern
```python
class HysteresisThreshold:
    def __init__(self, activate: float, deactivate: float):
        self.activate = activate      # Lower threshold to trigger
        self.deactivate = deactivate  # Higher threshold to release
        self.is_active = False

    def update(self, value: float) -> bool:
        if not self.is_active and value < self.activate:
            self.is_active = True
        elif self.is_active and value > self.deactivate:
            self.is_active = False
        return self.is_active
```

### Configuration (YAML)
```yaml
behavior_switcher:
  ros__parameters:
    activate_threshold: 0.5   # Trigger avoid behavior
    deactivate_threshold: 0.7 # Return to explore behavior
    dead_band: 0.2            # Difference between thresholds
```

---

## 4. JSON Decision Logging

### Decision
Use Python's `json` module with structured log entries appended to timestamped files.

### Rationale
- **Human-readable** when pretty-printed
- **Machine-parseable** for analysis tools
- **Native Python** - no additional dependencies
- **Structured** - enables filtering and replay

### Log Entry Schema
```json
{
    "timestamp": "2025-12-30T14:32:15.123456",
    "sequence": 42,
    "input": {
        "sensor": "scan",
        "value": 0.45,
        "topic": "/scan"
    },
    "decision": {
        "from_behavior": "explore",
        "to_behavior": "avoid",
        "trigger_rule": "distance_threshold",
        "threshold_used": 0.5
    },
    "outcome": {
        "command": {"linear_x": 0.0, "angular_z": 0.5},
        "success": true
    }
}
```

### File Naming Convention
```
logs/
└── decision_log_2025-12-30_143215.json
```

---

## 5. Heuristic Scoring Model

### Decision
Implement weighted scoring with configurable weights stored in YAML.

### Rationale
- **Transparent** - weights visible and tunable by learners
- **Deterministic** - predictable behavior for debugging
- **Extensible** - easy to add new behaviors

### Scoring Algorithm
```python
class HeuristicSelector:
    def __init__(self, weights: Dict[str, float]):
        self.weights = weights  # {'explore': 1.0, 'avoid': 2.0, 'idle': 0.5}

    def select_behavior(self, scores: Dict[str, float]) -> str:
        weighted = {b: scores[b] * self.weights[b] for b in scores}
        # Deterministic tie-breaking: alphabetical order
        return max(sorted(weighted.keys()), key=lambda b: weighted[b])
```

### Tie-Breaking Rule
When scores are equal, select behavior alphabetically (deterministic, reproducible).

---

## 6. Session-Scoped Adaptation Memory

### Decision
Use in-memory Python dictionary with optional JSON export for session review.

### Rationale
- **Session-scoped** - resets on restart for clean learning experiments
- **Simple** - no database setup required
- **Exportable** - learners can save and analyze memory state

### Memory Structure
```python
class AdaptationMemory:
    def __init__(self, decay_rate: float = 0.9, max_entries: int = 100):
        self.history: List[Dict] = []
        self.score_adjustments: Dict[str, float] = {}
        self.decay_rate = decay_rate
        self.max_entries = max_entries

    def record_outcome(self, behavior: str, success: bool):
        adjustment = 0.1 if success else -0.1
        self.score_adjustments[behavior] = self.score_adjustments.get(behavior, 0) + adjustment
        # Apply decay to prevent unbounded growth
        self._apply_decay()
```

### Bounds and Decay
- **Max adjustment**: +/- 1.0 per behavior
- **Decay rate**: 0.9 per cycle (prevents runaway adaptation)
- **Max history**: 100 entries (prevents memory overflow)

---

## 7. Testing Strategy

### Decision
Three-tier testing: unit tests (pytest), integration tests (launch_testing), manual validation.

### Unit Tests
| Test | Purpose |
|------|---------|
| `test_hysteresis_threshold` | Verify on/off threshold logic |
| `test_heuristic_scoring` | Verify weighted selection and tie-breaking |
| `test_decision_logger` | Verify JSON schema and file creation |
| `test_adaptation_memory` | Verify decay, bounds, and recording |

### Integration Tests
```python
# test_behavior_switching_launch.py
@pytest.mark.launch(fixture=generate_test_description)
def test_behavior_switch_on_obstacle(launch_context, proc_info):
    # Simulate obstacle approach
    # Verify behavior switches within 500ms
    pass
```

### Manual Validation
- Run TurtleBot3 in Gazebo
- Approach obstacles
- Verify behavior switching in rviz2
- Review JSON logs

---

## 8. Real-World Deployment Notes

### Simulation vs. Hardware Differences
| Aspect | Simulation | Real Hardware |
|--------|------------|---------------|
| Sensor noise | Minimal (optional) | Significant |
| Latency | Consistent | Variable |
| Battery | Unlimited | Limited |
| Safety | None required | Critical |

### Hardware Recommendations
- Add sensor noise filtering (moving average)
- Increase hysteresis dead band for real sensors
- Add emergency stop behavior
- Monitor battery level

---

## Summary

All research questions resolved. No NEEDS CLARIFICATION remaining.

| Topic | Decision | Confidence |
|-------|----------|------------|
| Behavior switching | ROS 2 topics + lifecycle nodes | High |
| Robot platform | TurtleBot3 Burger | High |
| Hysteresis | Dual thresholds with dead band | High |
| Logging | JSON files with structured schema | High |
| Scoring | Weighted selection with alphabetic tie-break | High |
| Memory | Session-scoped with decay bounds | High |
| Testing | Unit + integration + manual | High |

---

*End of Research*
