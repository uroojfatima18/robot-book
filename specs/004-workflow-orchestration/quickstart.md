# Quickstart Guide: Chapter 4 - Workflow Orchestration

**Branch**: `004-workflow-orchestration` | **Date**: 2025-12-30

---

## Prerequisites

Before starting Chapter 4, ensure you have:

1. **Completed Chapters 1-3** or equivalent ROS 2 knowledge
2. **ROS 2 Humble or Iron** installed and sourced
3. **Python 3.10+** available
4. **colcon** build tools installed
5. (Optional) **Gazebo Sim** for full simulation examples

### Quick Environment Check

```bash
# Verify ROS 2 installation
ros2 --version
# Expected: ros2 0.x.x (Humble or Iron)

# Verify Python version
python3 --version
# Expected: Python 3.10.x or higher

# Verify colcon
colcon --version
# Expected: colcon 0.x.x
```

---

## Workspace Setup

### Option A: With Full Repository

```bash
# Clone the repository (if not already done)
git clone <repository-url> robot-book
cd robot-book

# Navigate to chapter 4 workspace
cd chapters/04-workflow-orchestration/code/ros2_ws

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### Option B: Standalone Chapter Setup

```bash
# Create workspace
mkdir -p ~/ch4_ws/src
cd ~/ch4_ws/src

# Copy packages from chapter
cp -r /path/to/robot-book/chapters/04-workflow-orchestration/code/ros2_ws/src/* .

# Build
cd ~/ch4_ws
colcon build --symlink-install

# Source
source install/setup.bash
```

---

## Running Examples

### Beginner: Simple Pipeline Demo

Run a basic three-node pipeline that demonstrates data flow:

```bash
# Terminal 1: Launch the pipeline
ros2 launch workflow_examples pipeline_demo.launch.py

# Terminal 2: Monitor output
ros2 topic echo /robot/cmd_vel
```

**What to observe:**
- Nodes start in order: lidar → planner → controller
- Log timestamps show sequential processing
- Velocity commands appear on `/robot/cmd_vel`

### Intermediate: Service-Based Control

Control pipeline execution via services:

```bash
# Terminal 1: Launch pipeline
ros2 launch workflow_examples pipeline_demo.launch.py

# Terminal 2: Check status
ros2 service call /pipeline/get_status workflow_srvs/srv/GetPipelineStatus

# Terminal 3: Pause pipeline
ros2 service call /pipeline/set_state workflow_srvs/srv/SetPipelineState \
  "{requested_state: 2}"  # 2 = PAUSED

# Resume pipeline
ros2 service call /pipeline/set_state workflow_srvs/srv/SetPipelineState \
  "{requested_state: 1}"  # 1 = RUNNING
```

### Advanced: Watchdog Supervisor

Run the watchdog-monitored pipeline:

```bash
# Terminal 1: Launch with watchdog
ros2 launch workflow_examples watchdog_demo.launch.py

# Terminal 2: Monitor alerts
ros2 topic echo /supervisor/alerts

# Terminal 3: Simulate failure (kill a node)
ros2 lifecycle set /lidar shutdown

# Observe: Watchdog detects failure and initiates recovery
```

---

## Mock Nodes (No Gazebo Required)

All examples can run without Gazebo using mock nodes:

```bash
# Launch with mock sensors
ros2 launch workflow_examples pipeline_demo.launch.py use_sim:=false

# The mock nodes publish realistic simulated data
ros2 topic echo /robot/scan --once
```

---

## Running Tests

### Unit Tests

```bash
cd ~/ch4_ws
colcon test --packages-select workflow_tests --pytest-args -v
colcon test-result --verbose
```

### Tier-Specific Assessments

```bash
# Beginner tier assessment
colcon test --packages-select workflow_tests \
  --pytest-args -k "test_beginner" -v

# Intermediate tier assessment
colcon test --packages-select workflow_tests \
  --pytest-args -k "test_intermediate" -v

# Advanced tier assessment
colcon test --packages-select workflow_tests \
  --pytest-args -k "test_advanced" -v
```

---

## Troubleshooting

### Common Issues

| Problem | Solution |
|---------|----------|
| "Package not found" | Run `source install/setup.bash` |
| Build fails | Check ROS 2 environment: `printenv | grep ROS` |
| Nodes don't communicate | Check topic names: `ros2 topic list` |
| Service timeout | Verify node is running: `ros2 node list` |

### Getting Help

Use these RAG prompts with your AI assistant:

**Debugging:**
> "My pipeline nodes start but no data flows between them. How do I debug topic connections?"

**Concepts:**
> "Explain how ROS 2 callback groups affect state machine implementations."

**Code:**
> "Generate a watchdog timer node that monitors /robot/scan at 10 Hz."

---

## File Structure Reference

```
chapters/04-workflow-orchestration/
├── content/                    # Lesson content
│   ├── beginner/
│   ├── intermediate/
│   └── advanced/
├── code/
│   └── ros2_ws/
│       └── src/
│           ├── workflow_examples/   # Example nodes
│           ├── workflow_mocks/      # Mock sensors
│           └── workflow_tests/      # Assessments
├── exercises/                  # Hands-on exercises
├── diagrams/                   # Mermaid diagrams
└── assessments/                # Tier criteria
```

---

## Next Steps

1. **Beginner**: Start with `content/beginner/B1-pipelines-flows-triggers.md`
2. **Intermediate**: Proceed to `content/intermediate/I1-launch-files.md`
3. **Advanced**: Challenge yourself with `content/advanced/A1-watchdogs-supervisors.md`

Each lesson builds on the previous. Complete exercises before moving to the next tier.

---

*Quickstart guide completed: 2025-12-30*
