# ROS 2 Interface Contracts: Chapter 4 - Workflow Orchestration

**Branch**: `004-workflow-orchestration` | **Date**: 2025-12-30

---

## Overview

This document defines the ROS 2 message, service, and action interfaces used in Chapter 4 code examples.

---

## 1. Custom Messages

### workflow_msgs/msg/PipelineStatus

Reports the status of a pipeline stage.

```
# Pipeline stage status report

# Header for timestamp
std_msgs/Header header

# Stage identification
string stage_name
string node_name

# Status enumeration
uint8 STATUS_IDLE = 0
uint8 STATUS_RUNNING = 1
uint8 STATUS_PAUSED = 2
uint8 STATUS_ERROR = 3
uint8 STATUS_SHUTDOWN = 4
uint8 status

# Performance metrics
float32 processing_time_ms
float32 messages_per_second
uint32 total_messages_processed

# Error information (populated when status == STATUS_ERROR)
string error_message
uint32 error_code
```

### workflow_msgs/msg/WatchdogAlert

Alert message from watchdog supervisor.

```
# Watchdog alert notification

# Header for timestamp
std_msgs/Header header

# Alert severity
uint8 SEVERITY_INFO = 0
uint8 SEVERITY_WARNING = 1
uint8 SEVERITY_ERROR = 2
uint8 SEVERITY_CRITICAL = 3
uint8 severity

# Alert details
string node_name
string alert_type          # "timeout", "crash", "dropout", "recovery"
string message

# Recovery status
uint8 recovery_attempt     # Current attempt number (0 = no recovery in progress)
uint8 max_recovery_attempts
bool recovery_successful
```

### workflow_msgs/msg/SensorHealth

Health status for sensor monitoring.

```
# Sensor health report

# Header for timestamp
std_msgs/Header header

# Sensor identification
string sensor_name
string topic_name

# Health metrics
bool is_healthy
float32 expected_rate_hz
float32 actual_rate_hz
uint32 consecutive_dropouts
uint32 total_dropouts

# Last message timing
builtin_interfaces/Time last_message_time
float32 time_since_last_message_sec
```

---

## 2. Custom Services

### workflow_srvs/srv/SetPipelineState

Control pipeline execution state.

```
# Request: Set the pipeline state
uint8 STATE_IDLE = 0
uint8 STATE_RUNNING = 1
uint8 STATE_PAUSED = 2
uint8 STATE_SHUTDOWN = 3
uint8 requested_state

# Optional: specific stage to affect (empty = all stages)
string stage_name
---
# Response
bool success
string message
uint8 current_state
```

### workflow_srvs/srv/GetPipelineStatus

Query current pipeline status.

```
# Request: Get pipeline status
string stage_name    # Empty = get all stages
---
# Response
bool success
workflow_msgs/PipelineStatus[] stage_statuses
string message
```

### workflow_srvs/srv/SelectPath

Switch between pipeline execution paths.

```
# Request: Select execution path
string path_name     # "default", "fallback", "emergency"
string reason        # Why switching (for logging)
---
# Response
bool success
string previous_path
string current_path
string message
```

### workflow_srvs/srv/TriggerRecovery

Manually trigger recovery for a failed node.

```
# Request: Trigger recovery
string node_name
bool force_restart   # true = skip graceful recovery, go straight to restart
---
# Response
bool success
uint8 recovery_attempt
string message
```

---

## 3. Custom Actions

### workflow_actions/action/ExecutePipeline

Execute a multi-stage pipeline to completion.

```
# Goal: Execute pipeline
string pipeline_name
string[] stages_to_execute    # Empty = all stages
float32 timeout_sec           # 0 = no timeout

---
# Result
bool success
string[] completed_stages
string[] failed_stages
float32 total_execution_time_sec
string result_message

---
# Feedback
string current_stage
uint8 stages_completed
uint8 total_stages
float32 progress_percent
string status_message
```

### workflow_actions/action/NavigateToWaypoint

Navigate robot to a waypoint (example domain action).

```
# Goal: Navigate to waypoint
geometry_msgs/PoseStamped target_pose
float32 tolerance_position_m   # Default: 0.1
float32 tolerance_orientation_rad  # Default: 0.1
bool allow_replanning          # Default: true

---
# Result
bool success
geometry_msgs/PoseStamped final_pose
float32 distance_traveled_m
float32 time_elapsed_sec
string result_message

---
# Feedback
geometry_msgs/PoseStamped current_pose
float32 distance_remaining_m
float32 estimated_time_remaining_sec
string current_status    # "planning", "executing", "recovery"
```

---

## 4. Standard Interfaces Used

### Topics

| Topic | Message Type | Direction | Description |
|-------|--------------|-----------|-------------|
| `/robot/scan` | `sensor_msgs/LaserScan` | Pub | Lidar scan data |
| `/robot/path` | `nav_msgs/Path` | Pub | Planned path |
| `/robot/cmd_vel` | `geometry_msgs/Twist` | Pub | Velocity commands |
| `/robot/odom` | `nav_msgs/Odometry` | Pub | Robot odometry |
| `/supervisor/status` | `workflow_msgs/PipelineStatus` | Pub | Pipeline status |
| `/supervisor/alerts` | `workflow_msgs/WatchdogAlert` | Pub | Watchdog alerts |
| `/*/health` | `workflow_msgs/SensorHealth` | Pub | Per-node health |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/pipeline/set_state` | `workflow_srvs/SetPipelineState` | Control pipeline state |
| `/pipeline/get_status` | `workflow_srvs/GetPipelineStatus` | Query pipeline status |
| `/pipeline/select_path` | `workflow_srvs/SelectPath` | Switch execution path |
| `/supervisor/trigger_recovery` | `workflow_srvs/TriggerRecovery` | Manual recovery trigger |

### Actions

| Action | Type | Description |
|--------|------|-------------|
| `/pipeline/execute` | `workflow_actions/ExecutePipeline` | Execute pipeline |
| `/navigation/waypoint` | `workflow_actions/NavigateToWaypoint` | Navigate to waypoint |

---

## 5. QoS Profiles

### Sensor Data (High Frequency)

```yaml
sensor_qos:
  reliability: BEST_EFFORT
  durability: VOLATILE
  history: KEEP_LAST
  depth: 5
  lifespan: 100ms
```

### Control Commands (Reliable)

```yaml
control_qos:
  reliability: RELIABLE
  durability: VOLATILE
  history: KEEP_LAST
  depth: 10
  deadline: 50ms
```

### Status/Alerts (Persistent)

```yaml
status_qos:
  reliability: RELIABLE
  durability: TRANSIENT_LOCAL
  history: KEEP_LAST
  depth: 100
```

---

## 6. Error Codes

### Pipeline Errors

| Code | Name | Description |
|------|------|-------------|
| 0 | SUCCESS | Operation completed successfully |
| 1 | TIMEOUT | Operation timed out |
| 2 | INVALID_STATE | Invalid state transition requested |
| 3 | NODE_NOT_FOUND | Referenced node does not exist |
| 4 | DEPENDENCY_FAILED | Upstream dependency failed |
| 5 | RECOVERY_FAILED | Recovery attempts exhausted |
| 6 | HARDWARE_ERROR | Hardware communication error |
| 7 | CONFIGURATION_ERROR | Invalid configuration |

### Watchdog Errors

| Code | Name | Description |
|------|------|-------------|
| 100 | HEARTBEAT_TIMEOUT | No heartbeat received within timeout |
| 101 | NODE_CRASH | Node process terminated unexpectedly |
| 102 | SENSOR_DROPOUT | Sensor data frequency below threshold |
| 103 | RECOVERY_INITIATED | Recovery process started |
| 104 | RECOVERY_COMPLETE | Recovery successful |
| 105 | ESCALATION_REQUIRED | Manual intervention needed |

---

## 7. Package Dependencies

### workflow_msgs (message definitions)

```xml
<!-- package.xml -->
<package format="3">
  <name>workflow_msgs</name>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
</package>
```

### workflow_srvs (service definitions)

```xml
<!-- package.xml -->
<package format="3">
  <name>workflow_srvs</name>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>workflow_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
</package>
```

### workflow_actions (action definitions)

```xml
<!-- package.xml -->
<package format="3">
  <name>workflow_actions</name>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>geometry_msgs</depend>
  <depend>workflow_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
</package>
```

---

*Contracts completed: 2025-12-30*
