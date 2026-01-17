---
id: a_lesson2_planners
title: "Planners and Behavior Trees"
sidebar_position: 37
tier: advanced
chapter: chapter_3_ai_brain
estimated_time: "60-90 minutes"
prerequisites: ["a_lesson1_costmap"]
---

# A2: Planners and Behavior Trees

**Customizing Path Planning and Navigation Behaviors**

---

## Learning Objectives

By the end of this lesson, you will be able to:
- Explain the difference between global and local planners
- Configure Nav2 planner plugins (NavFn, Smac, MPPI)
- Understand behavior tree structure and nodes
- Create simple custom navigation behaviors

---

## Prerequisites

- Completed [A1: Costmap Configuration](A1-costmap-configuration.md)
- Nav2 running with custom costmaps
- Understanding of navigation flow

---

## Theory: Global Planners

### Purpose

**Global planners** compute a path from the robot's current position to the goal using the costmap.

### Available Planners

| Planner | Algorithm | Best For |
|---------|-----------|----------|
| **NavFn** | Wavefront/Dijkstra | General purpose, smooth paths |
| **Smac 2D** | A* variants | Large maps, fast planning |
| **Smac Hybrid** | Hybrid A* | Non-holonomic robots (cars) |
| **Smac Lattice** | State lattice | High-quality paths, complex constraints |
| **Theta*** | Any-angle A* | Smooth paths, open spaces |

### NavFn Configuration

```yaml
planner_server:
  ros__parameters:
    plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5                    # Goal tolerance (m)
      use_astar: false                  # true = A*, false = Dijkstra
      allow_unknown: true
```

### Smac Planner Configuration

```yaml
planner_server:
  ros__parameters:
    plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.25
      downsample_costmap: false
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 2.0            # seconds
      motion_model_for_search: "MOORE"  # 8-connected
      cost_travel_multiplier: 2.0
```

---

## Theory: Local Planners (Controllers)

### Purpose

**Local planners** (controllers) generate velocity commands to follow the global path while avoiding obstacles.

### Available Controllers

| Controller | Approach | Best For |
|------------|----------|----------|
| **DWB** | Dynamic Window | General purpose, differential drive |
| **TEB** | Timed Elastic Band | Time-optimal, smooth trajectories |
| **MPPI** | Model Predictive | Complex dynamics, GPU acceleration |
| **RPP** | Regulated Pure Pursuit | Simple, reliable path following |

### DWB Configuration

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.26
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_theta: -3.2
```

---

## Theory: Behavior Trees

### What are Behavior Trees?

**Behavior Trees (BTs)** are a way to structure complex robot behaviors as a tree of actions and conditions.

### BT Node Types

| Type | Symbol | Function |
|------|--------|----------|
| **Sequence** | → | Execute children in order, fail if any fails |
| **Fallback** | ? | Try children until one succeeds |
| **Action** | ▢ | Execute a robot action |
| **Condition** | ◇ | Check a condition |
| **Decorator** | ◆ | Modify child behavior |

### Nav2 Default BT Structure

```
NavigateWithReplanning (root)
├── RecoveryFallback
│   ├── MainSequence
│   │   ├── RateController (1Hz)
│   │   │   └── ComputePathToPose
│   │   └── FollowPath
│   └── RecoverySequence
│       ├── ClearCostmaps
│       ├── Spin
│       └── BackUp
```

---

## Code Example: Custom Behavior Tree

```xml
<!-- behavior_tree_example.xml -->
<!-- Custom navigation behavior tree with waypoint following -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="3" name="NavigateRecovery">

      <!-- Main navigation sequence -->
      <PipelineSequence name="NavigateWithReplanning">

        <!-- Compute path at 1Hz -->
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}"
            planner_id="GridBased"/>
        </RateController>

        <!-- Follow the computed path -->
        <FollowPath path="{path}" controller_id="FollowPath"/>

      </PipelineSequence>

      <!-- Recovery behaviors when navigation fails -->
      <SequenceStar name="RecoverySequence">

        <!-- First: clear costmaps -->
        <ClearEntireCostmap name="ClearGlobalCostmap"
          service_name="global_costmap/clear_entirely_global_costmap"/>
        <ClearEntireCostmap name="ClearLocalCostmap"
          service_name="local_costmap/clear_entirely_local_costmap"/>

        <!-- Second: wait briefly -->
        <Wait wait_duration="2"/>

        <!-- Third: spin to scan environment -->
        <Spin spin_dist="1.57"/>

        <!-- Fourth: back up -->
        <BackUp backup_dist="0.3" backup_speed="0.1"/>

      </SequenceStar>

    </RecoveryNode>
  </BehaviorTree>
</root>
```

---

## Using Custom Behavior Trees

### Step 1: Save the BT XML

Save to: `my_package/behavior_trees/my_nav_bt.xml`

### Step 2: Configure Nav2 to Use It

```yaml
bt_navigator:
  ros__parameters:
    default_bt_xml_filename: "/path/to/my_nav_bt.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
```

### Step 3: Visualize with Groot

```bash
# Install Groot (BT visualization tool)
sudo apt install ros-${ROS_DISTRO}-groot

# Launch Groot
ros2 run groot Groot
```

---

## Key Concepts Summary

| Concept | Description |
|---------|-------------|
| **Global Planner** | Computes path using full costmap |
| **Local Planner** | Follows path, avoids dynamic obstacles |
| **Behavior Tree** | Structures navigation logic |
| **Recovery** | Actions when navigation fails |
| **Groot** | BT visualization tool |

---

## Hands-On Exercise

### Exercise A2.1: Compare Planners

1. Configure Nav2 with NavFn planner
2. Send a navigation goal through a complex environment
3. Note the path quality and planning time
4. Switch to Smac 2D planner
5. Send the same goal and compare results

### Exercise A2.2: Custom Recovery Behavior

Modify the behavior tree to:
1. Add a "Wait" action before spinning
2. Increase backup distance to 0.5m
3. Add a second retry with longer spin (3.14 radians)

---

## AI Agent Assisted Prompts

### Prompt 1: Planner Selection
```
I have a large warehouse (100m x 100m) with narrow aisles. Path planning
currently takes 2-3 seconds. Which planner should I use and how should I
configure it for faster planning?
```

### Prompt 2: Behavior Tree Design
```
I want my robot to pause and play a sound when it's been stuck for more than
10 seconds. How do I add this behavior to the Nav2 behavior tree? What BT
nodes do I need?
```

### Prompt 3: Controller Tuning
```
My robot oscillates when following paths and sometimes overshoots corners.
What DWB parameters should I tune to get smoother path following?
```

---

## Summary

In this lesson, you learned:

1. **Global planners** (NavFn, Smac) compute paths on the costmap
2. **Local planners** (DWB, MPPI) follow paths and avoid obstacles
3. **Behavior trees** structure navigation logic
4. **Recovery behaviors** handle navigation failures
5. How to **create custom BTs** for specific behaviors

---

## Next Steps

**Congratulations!** You've completed the Advanced Navigation tier!

- **Next Lesson**: [A3: Reinforcement Learning Fundamentals](A3-reinforcement-learning.md)
- **Exercises**: Experiment with different planners and BT configurations
