---
id: advanced_exercises
title: "Advanced Tier Exercises"
sidebar_position: 4
tier: advanced
chapter: chapter_1_ros2
---

# Advanced Tier Exercises

**Chapter**: Chapter 1 - The Robotic Nervous System (ROS 2)
**Tier**: Advanced
**Prerequisites**: Complete Lessons A1 (URDF) and A2 (Actions)

---

## Overview

These exercises challenge you to apply advanced ROS 2 concepts to realistic robotics scenarios. Each exercise builds on the foundational skills from earlier tiers.

---

## Exercise A1: Complete Humanoid URDF

**Difficulty**: Medium
**Estimated Time**: 1-2 hours
**Related Lesson**: A1 - URDF & Humanoid Robot Description

### Description

Extend the basic humanoid URDF to include complete legs with proper joint constraints.

### Tasks

1. **Add leg links**:
   - Create `left_thigh`, `left_shin`, `left_foot` links
   - Create corresponding right leg links
   - Use appropriate cylinder and box geometries

2. **Create hip joints**:
   - `left_hip_pitch`: forward/backward leg swing (revolute)
   - `left_hip_roll`: sideways leg movement (revolute)
   - Mirror for right side

3. **Create knee and ankle joints**:
   - `left_knee`: flexion/extension (revolute, 0 to ~2.5 rad)
   - `left_ankle`: dorsiflexion/plantarflexion (revolute)

4. **Add inertial properties** for simulation compatibility

### Acceptance Criteria

- [ ] URDF passes `check_urdf` validation
- [ ] Robot visualizes correctly in RViz2
- [ ] All joints move within realistic limits
- [ ] `urdf_to_graphviz` shows correct tree structure

### Hints

<details>
<summary>Hint 1: Hip Joint Orientation</summary>

For a humanoid standing upright:
- Hip pitch axis: `<axis xyz="0 1 0"/>` (Y-axis, forward/back)
- Hip roll axis: `<axis xyz="1 0 0"/>` (X-axis, side-to-side)
</details>

<details>
<summary>Hint 2: Leg Dimensions</summary>

Realistic proportions (adjust scale as needed):
- Thigh: cylinder radius=0.05, length=0.4
- Shin: cylinder radius=0.04, length=0.35
- Foot: box size="0.2 0.08 0.03"
</details>

---

## Exercise A2: Navigation Action Server

**Difficulty**: Hard
**Estimated Time**: 2-3 hours
**Related Lesson**: A2 - Advanced ROS 2 Patterns

### Description

Implement an action server that simulates robot navigation to a target position.

### Tasks

1. **Define the action structure** (conceptual):
   ```
   # Goal
   geometry_msgs/Point target_position
   float64 max_velocity

   # Result
   bool success
   float64 total_distance
   float64 total_time

   # Feedback
   geometry_msgs/Point current_position
   float64 distance_remaining
   float64 percent_complete
   ```

2. **Implement the server**:
   - Accept goals with position validation
   - Simulate movement at the specified velocity
   - Publish feedback with current position and progress
   - Handle cancellation gracefully

3. **Implement a client**:
   - Send navigation goals
   - Display feedback in real-time
   - Handle success, failure, and cancellation

### Acceptance Criteria

- [ ] Server accepts valid goals, rejects invalid positions
- [ ] Feedback updates at least 10Hz during movement
- [ ] Cancellation stops movement immediately
- [ ] Result includes accurate total distance and time
- [ ] Multiple goals can be sent in sequence

### Starter Code

```python
# Use the standard Fibonacci example as a template
# Replace Fibonacci with a custom Point-based action

import math
from geometry_msgs.msg import Point

class NavigateServer(Node):
    def __init__(self):
        super().__init__('navigate_server')
        self.current_position = Point(x=0.0, y=0.0, z=0.0)
        # TODO: Create action server

    def distance_to(self, target: Point) -> float:
        """Calculate Euclidean distance to target."""
        return math.sqrt(
            (target.x - self.current_position.x) ** 2 +
            (target.y - self.current_position.y) ** 2
        )
```

---

## Exercise A3: Multi-Robot Coordination

**Difficulty**: Challenge
**Estimated Time**: 3-4 hours
**Related Lessons**: A1, A2, I2 (Launch Files)

### Description

Create a system where multiple robots coordinate using actions and topics.

### Scenario

Three robots need to move to positions forming a triangle. Each robot uses an action server for movement, and a coordinator node orchestrates the overall behavior.

### Tasks

1. **Create a launch file** that starts:
   - 3 robot namespaces: `/robot1`, `/robot2`, `/robot3`
   - A navigation action server for each robot
   - A coordinator node

2. **Implement the coordinator**:
   - Subscribe to each robot's position topic
   - Send navigation goals to form a triangle
   - Monitor progress and report when all robots are in position

3. **Handle failures**:
   - If one robot fails, cancel all movements
   - Report which robot failed and why

### Acceptance Criteria

- [ ] All three robots can be controlled independently
- [ ] Coordinator successfully moves all robots to triangle formation
- [ ] Failure of one robot triggers graceful shutdown of others
- [ ] Progress is logged for monitoring

### Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                        Coordinator                          │
│                                                              │
│   Sends NavigateTo goals    Monitors position topics        │
│          ↓                         ↑                        │
├─────────────┬─────────────┬─────────────────────────────────┤
│   /robot1   │   /robot2   │   /robot3                       │
│   └─action  │   └─action  │   └─action                      │
│   └─/pose   │   └─/pose   │   └─/pose                       │
└─────────────┴─────────────┴─────────────────────────────────┘
```

---

## Exercise A4: AI Command Interface

**Difficulty**: Challenge
**Estimated Time**: 2-3 hours
**Related Lesson**: A2 - AI Integration Concepts

### Description

Create a simple command interface that parses text commands and executes robot actions.

### Tasks

1. **Create a command parser node**:
   - Subscribe to a `/commands` topic (String)
   - Parse commands like "move forward 1 meter" or "turn left 90 degrees"
   - Send appropriate action goals

2. **Implement command types**:
   - `move <direction> <distance>`: forward, backward, left, right
   - `turn <direction> <angle>`: left, right with degrees
   - `stop`: Cancel current action

3. **Provide feedback**:
   - Publish execution status to `/command_status`
   - Log progress and errors

### Acceptance Criteria

- [ ] Parser handles all command types
- [ ] Invalid commands produce helpful error messages
- [ ] Actions are executed correctly
- [ ] Status updates are published for each command phase

### Example Session

```bash
# Terminal 1: Start the command interface
ros2 run my_pkg command_interface

# Terminal 2: Send commands
ros2 topic pub -1 /commands std_msgs/String "data: 'move forward 2'"
ros2 topic pub -1 /commands std_msgs/String "data: 'turn left 90'"
ros2 topic pub -1 /commands std_msgs/String "data: 'stop'"

# Terminal 3: Monitor status
ros2 topic echo /command_status
```

---

## Solutions

Solutions for these exercises are available in the course repository under `solutions/chapter-01/advanced/`.

Each solution includes:
- Complete working code
- Comments explaining key decisions
- Test scripts to verify functionality

---

## Next Steps

After completing these exercises, you're ready for:
- **Chapter 2**: Simulation with Gazebo and Unity
- **Chapter 3**: Motion Planning with MoveIt2
- **Chapter 4**: AI Integration with NVIDIA Isaac

Congratulations on completing the Advanced tier!
