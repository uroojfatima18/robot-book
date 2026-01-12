# ROS 2 Message Contracts: Chapter 2

**Generated**: 2025-12-25
**Phase**: 1 (Design & Contracts)

## Standard Messages Used

This chapter uses standard ROS 2 message types. No custom messages required.

### sensor_msgs/JointState

Used for publishing and subscribing to joint positions, velocities, and efforts.

```
# Standard ROS 2 message
std_msgs/Header header
string[] name           # Joint names
float64[] position      # Joint positions (rad or m)
float64[] velocity      # Joint velocities (rad/s or m/s)
float64[] effort        # Joint efforts (Nm or N)
```

**Usage in Chapter**:
- Topic: `/joint_states` - Physical robot joint state
- Topic: `/sim/joint_states` - Simulated robot joint state
- QoS: BEST_EFFORT, depth=10

### trajectory_msgs/JointTrajectory

Used for commanding joint movements.

```
# Standard ROS 2 message
std_msgs/Header header
string[] joint_names
trajectory_msgs/JointTrajectoryPoint[] points
```

**JointTrajectoryPoint**:
```
float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
builtin_interfaces/Duration time_from_start
```

**Usage in Chapter**:
- Topic: `/joint_commands` - Commands to physical robot
- Topic: `/sim/joint_commands` - Commands to simulation
- QoS: RELIABLE, depth=10

### geometry_msgs/Pose

Used for model spawning position.

```
geometry_msgs/Point position
  float64 x
  float64 y
  float64 z
geometry_msgs/Quaternion orientation
  float64 x
  float64 y
  float64 z
  float64 w
```

### sensor_msgs/Imu

Used for IMU sensor simulation (Advanced tier).

```
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
```

**Usage in Chapter**:
- Topic: `/sim/imu` - Simulated IMU data
- QoS: BEST_EFFORT, depth=10

### sensor_msgs/Image

Used for camera simulation stub (Advanced tier).

```
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
```

**Usage in Chapter**:
- Topic: `/sim/camera/image_raw` - Simulated camera image
- QoS: BEST_EFFORT, depth=1

## Topic Architecture

```
Physical Robot Side          Bridge Node           Simulation Side
==================          ===========           ===============

/joint_states ────────────►┌─────────────┐◄────── /sim/joint_states
                           │             │
/joint_commands ◄──────────│   Bridge    │──────► /sim/joint_commands
                           │    Node     │
/imu (future) ◄────────────│             │──────► /sim/imu
                           │             │
/camera/image_raw ◄────────└─────────────┘──────► /sim/camera/image_raw
(future)
```

## QoS Profiles

### Real-Time Data (Joint States, IMU)
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

realtime_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

### Command Data (Joint Commands)
```python
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

### Image Data (Camera)
```python
image_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1  # Only latest frame
)
```

## Service Contracts

### /spawn_entity (gazebo_msgs/srv/SpawnEntity)

Used to spawn URDF models into Gazebo.

**Request**:
```
string name              # Model name
string xml               # URDF/SDF content
string robot_namespace   # Namespace for topics
geometry_msgs/Pose initial_pose
string reference_frame   # Parent frame
```

**Response**:
```
bool success
string status_message
```

### /delete_entity (gazebo_msgs/srv/DeleteEntity)

Used to remove models from Gazebo.

**Request**:
```
string name              # Model to delete
```

**Response**:
```
bool success
string status_message
```
