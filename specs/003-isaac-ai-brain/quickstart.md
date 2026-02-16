# Quickstart: Chapter 3 - AI-Robot Brain (NVIDIA Isaac)

**Date**: 2025-12-27
**Feature**: 003-isaac-ai-brain

## Overview

This quickstart guide provides the essential information to begin implementing Chapter 3 content.

---

## Prerequisites Check

Before starting Chapter 3 implementation:

- [ ] Chapter 1 (ROS 2 Fundamentals) content exists for reference
- [ ] Chapter 2 (Digital Twin) content exists for reference
- [ ] ROS 2 Humble or Iron installed and tested
- [ ] Gazebo Classic or Ignition installed
- [ ] Nav2 and SLAM Toolbox packages available

---

## Quick Setup

### 1. Create Chapter Directory Structure

```bash
mkdir -p chapters/003-ai-robot-brain/{beginner,intermediate,advanced}/{diagrams,code}
mkdir -p chapters/003-ai-robot-brain/exercises
```

### 2. Install Required ROS 2 Packages

```bash
# Nav2 and SLAM Toolbox
sudo apt install ros-${ROS_DISTRO}-navigation2
sudo apt install ros-${ROS_DISTRO}-nav2-bringup
sudo apt install ros-${ROS_DISTRO}-slam-toolbox

# Vision packages
sudo apt install ros-${ROS_DISTRO}-cv-bridge
sudo apt install ros-${ROS_DISTRO}-image-transport

# TF2
sudo apt install ros-${ROS_DISTRO}-tf2-ros
sudo apt install ros-${ROS_DISTRO}-tf2-tools
```

### 3. Verify Installation

```bash
# Check Nav2
ros2 pkg list | grep nav2

# Check SLAM Toolbox
ros2 pkg list | grep slam_toolbox

# Check cv_bridge
python3 -c "from cv_bridge import CvBridge; print('cv_bridge OK')"
```

---

## Implementation Order

### Phase 1: Beginner Tier (MVP)

| Order | Lesson | Key Deliverable |
|-------|--------|-----------------|
| 1 | B1-introduction-perception.md | Perception pipeline diagram |
| 2 | B2-sensor-types.md | Sensor comparison diagram |
| 3 | B3-slam-navigation-intro.md | Navigation architecture diagram |
| 4 | beginner-exercises.md | 3-5 concept exercises |

**MVP Checkpoint**: Readers can explain perception, identify sensors, and describe SLAM/navigation.

### Phase 2: Intermediate Tier

| Order | Lesson | Key Deliverable |
|-------|--------|-----------------|
| 5 | I1-camera-depth-processing.md | camera_subscriber.py |
| 6 | I2-tf2-coordinate-frames.md | tf2_broadcaster.py |
| 7 | I3-slam-toolbox.md | slam_launch.py |
| 8 | I4-nav2-basics.md | nav2_goal_sender.py |
| 9 | intermediate-exercises.md | 3-5 coding exercises |

**Tier Checkpoint**: Readers can create sensor nodes, run SLAM, and send navigation goals.

### Phase 3: Advanced Tier

| Order | Lesson | Key Deliverable |
|-------|--------|-----------------|
| 10 | A1-costmap-configuration.md | costmap_config.yaml |
| 11 | A2-planners-behavior-trees.md | behavior_tree_example.xml |
| 12 | A3-reinforcement-learning.md | RL loop diagram |
| 13 | A4-sim-to-real.md | policy_loader.py + pre-trained policy |
| 14 | advanced-exercises.md | 3-5 advanced exercises |

**Chapter Checkpoint**: Readers can configure Nav2, understand RL, and load pre-trained policies.

---

## Key Files Reference

### Diagrams Needed

| File | Format | Lesson |
|------|--------|--------|
| perception-pipeline.svg | SVG | B1 |
| sensor-comparison.svg | SVG | B2 |
| navigation-architecture.svg | SVG | B3 |
| tf-tree-example.svg | SVG | I2 |
| slam-process.svg | SVG | I3 |
| costmap-layers.svg | SVG | A1 |
| rl-loop.svg | SVG | A3 |
| sim-to-real-gap.svg | SVG | A4 |

### Code Examples Needed

| File | Language | Lesson |
|------|----------|--------|
| camera_subscriber.py | Python | I1 |
| depth_processor.py | Python | I1 |
| tf2_broadcaster.py | Python | I2 |
| slam_launch.py | Python | I3 |
| nav2_goal_sender.py | Python | I4 |
| navigation_launch.py | Python | I4 |
| costmap_config.yaml | YAML | A1 |
| behavior_tree_example.xml | XML | A2 |
| policy_loader.py | Python | A4 |

---

## Validation Commands

### Test Code Examples

```bash
# Check Python syntax
python3 -m py_compile chapters/003-ai-robot-brain/intermediate/code/*.py

# Verify YAML
python3 -c "import yaml; yaml.safe_load(open('chapters/003-ai-robot-brain/advanced/code/costmap_config.yaml'))"
```

### Test Diagrams

```bash
# List all diagrams
find chapters/003-ai-robot-brain -name "*.svg" -o -name "*.png"

# Verify SVG validity (requires xmllint)
xmllint --noout chapters/003-ai-robot-brain/*/diagrams/*.svg 2>&1
```

---

## Success Criteria Quick Check

| SC | Metric | How to Verify |
|----|--------|---------------|
| SC-001 | 90% comprehension (Beginner) | Review quiz |
| SC-004 | Working sensor node | Run camera_subscriber.py |
| SC-005 | Generated map | Run SLAM, save map |
| SC-006 | Nav2 running | Send goal, observe robot |
| SC-010 | All code runs | Execute all .py files |

---

## Next Steps

1. Run `/sp.tasks` to generate detailed task list
2. Begin with Beginner tier content (B1, B2, B3)
3. Create diagrams in parallel with content
4. Validate each lesson against contracts before moving on
