# Data Model: Chapter 5 - Adaptive Robotics

**Feature**: 005-adaptive-robotics-chapter
**Date**: 2025-12-30
**Status**: Complete

---

## Entity Relationship Diagram

```text
┌─────────────────┐       triggers        ┌─────────────────┐
│  FeedbackSource │──────────────────────▶│   TriggerRule   │
│  (Sensor Data)  │                       │  (Threshold)    │
└─────────────────┘                       └────────┬────────┘
                                                   │
                                                   │ activates
                                                   ▼
┌─────────────────┐       selects         ┌─────────────────┐
│   ScoreTable    │◀─────────────────────│  DecisionNode   │
│   (Weights)     │                       │  (Logic Hub)    │
└─────────────────┘                       └────────┬────────┘
                                                   │
        ┌──────────────────────────────────────────┼──────────────────────┐
        │                                          │                      │
        ▼                                          ▼                      ▼
┌─────────────────┐                       ┌─────────────────┐    ┌─────────────────┐
│ BehaviorModule  │                       │  DecisionLog    │    │AdaptationMemory │
│   (Actions)     │                       │   (Audit)       │    │  (Learning)     │
└─────────────────┘                       └─────────────────┘    └─────────────────┘
```

---

## Entity Definitions

### 1. FeedbackSource

**Description**: Sensor or user-triggered data that influences decisions.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| source_id | string | Unique identifier | Required, alphanumeric |
| sensor_type | enum | Type of sensor | `lidar`, `odometry`, `imu`, `user_input` |
| topic_name | string | ROS 2 topic | Must start with `/` |
| value | float | Current reading | Range depends on sensor |
| timestamp | datetime | When reading was taken | ISO 8601 format |
| is_valid | bool | Data quality flag | Default: true |

**Validation Rules**:
- `topic_name` must be a valid ROS 2 topic path
- `timestamp` must be within 1 second of current time (stale data check)

---

### 2. TriggerRule

**Description**: Condition definition for behavior switching.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| rule_id | string | Unique identifier | Required |
| source_id | string | Reference to FeedbackSource | Must exist |
| operator | enum | Comparison type | `<`, `>`, `<=`, `>=`, `==` |
| activate_threshold | float | Value to trigger activation | Required |
| deactivate_threshold | float | Value to trigger deactivation | Must differ from activate |
| target_behavior | string | Behavior to activate | Must exist in BehaviorModule |
| priority | int | Rule precedence (lower = higher) | 1-100 |
| enabled | bool | Is rule active | Default: true |

**Validation Rules**:
- `deactivate_threshold` must create valid hysteresis (typically > `activate_threshold` for `<` operator)
- `priority` determines order when multiple rules match

**State Transitions**:
```text
[inactive] ──(value crosses activate_threshold)──▶ [active]
[active] ──(value crosses deactivate_threshold)──▶ [inactive]
```

---

### 3. DecisionNode

**Description**: Central logic hub that runs conditional logic to select robot actions.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| node_name | string | ROS 2 node name | Required |
| current_behavior | string | Active behavior | Must exist in BehaviorModule |
| default_behavior | string | Fallback behavior | Required |
| active_rules | list[string] | Currently triggered rules | References TriggerRule |
| last_decision_time | datetime | When last decision made | ISO 8601 |
| decision_count | int | Total decisions made | >= 0 |

**Lifecycle States**:
```text
[unconfigured] ──configure──▶ [inactive] ──activate──▶ [active]
[active] ──deactivate──▶ [inactive] ──cleanup──▶ [unconfigured]
```

---

### 4. BehaviorModule

**Description**: Executable robot strategy (action set).

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| behavior_id | string | Unique identifier | Required, lowercase |
| display_name | string | Human-readable name | Required |
| velocity_linear | float | Forward/backward speed | -1.0 to 1.0 m/s |
| velocity_angular | float | Rotation speed | -2.0 to 2.0 rad/s |
| duration_limit | float | Max time in behavior | Optional, seconds |
| is_safe | bool | Can run on real hardware | Default: true |

**Standard Behaviors**:
| ID | Name | Linear | Angular | Purpose |
|----|------|--------|---------|---------|
| `idle` | Idle | 0.0 | 0.0 | Stopped |
| `explore` | Explore | 0.2 | 0.0 | Move forward |
| `avoid` | Avoid Obstacle | 0.0 | 0.5 | Turn away |
| `backup` | Back Up | -0.1 | 0.0 | Reverse |

---

### 5. ScoreTable

**Description**: Numeric weights for heuristic behavior selection.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| table_id | string | Unique identifier | Required |
| weights | dict[string, float] | Behavior -> weight mapping | All behaviors must exist |
| last_updated | datetime | When weights changed | ISO 8601 |
| version | int | Update count | >= 1 |

**Example**:
```yaml
weights:
  explore: 1.0
  avoid: 2.0
  idle: 0.5
  backup: 1.5
```

**Weight Constraints**:
- All weights must be > 0
- No upper limit (relative weighting)
- Tie-breaking: alphabetical order

---

### 6. AdaptationMemory

**Description**: Session-scoped store of past outcomes.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| memory_id | string | Session identifier | Auto-generated UUID |
| created_at | datetime | Session start time | ISO 8601 |
| history | list[HistoryEntry] | Past decisions | Max 100 entries |
| score_adjustments | dict[string, float] | Accumulated adjustments | -1.0 to +1.0 per behavior |
| decay_rate | float | Decay factor per cycle | 0.0 to 1.0 |

**HistoryEntry Structure**:
```python
{
    "sequence": 42,
    "behavior": "avoid",
    "success": true,
    "adjustment": 0.1,
    "timestamp": "2025-12-30T14:32:15"
}
```

**Bounds**:
- Max history entries: 100 (oldest removed first)
- Max adjustment per behavior: +/- 1.0
- Decay applied each decision cycle

---

### 7. DecisionLog

**Description**: Timestamped audit record.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| log_id | string | Unique identifier | UUID |
| session_id | string | Links to AdaptationMemory | Required |
| sequence | int | Order within session | Auto-increment |
| timestamp | datetime | When logged | ISO 8601 |
| input | InputRecord | Sensor data snapshot | Required |
| decision | DecisionRecord | What was decided | Required |
| outcome | OutcomeRecord | What happened | Optional (filled later) |

**InputRecord**:
```json
{
    "sensor": "scan",
    "value": 0.45,
    "topic": "/scan"
}
```

**DecisionRecord**:
```json
{
    "from_behavior": "explore",
    "to_behavior": "avoid",
    "trigger_rule": "distance_threshold",
    "threshold_used": 0.5
}
```

**OutcomeRecord**:
```json
{
    "command": {"linear_x": 0.0, "angular_z": 0.5},
    "success": true,
    "duration_ms": 1234
}
```

---

## Relationships Summary

| From | To | Relationship | Cardinality |
|------|-----|--------------|-------------|
| FeedbackSource | TriggerRule | triggers | 1:N |
| TriggerRule | BehaviorModule | activates | N:1 |
| DecisionNode | TriggerRule | evaluates | 1:N |
| DecisionNode | BehaviorModule | selects | 1:1 |
| DecisionNode | ScoreTable | uses | 1:1 |
| DecisionNode | DecisionLog | writes | 1:N |
| DecisionNode | AdaptationMemory | updates | 1:1 |
| AdaptationMemory | ScoreTable | adjusts | 1:1 |

---

*End of Data Model*
