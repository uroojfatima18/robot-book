# Quickstart: Chapter 2 - Digital Twin & Simulation

**Generated**: 2025-12-25
**Estimated Setup Time**: 30-45 minutes

## Prerequisites

Before starting Chapter 2, ensure you have:

- [x] Completed Chapter 1 (ROS 2 Fundamentals)
- [x] Valid URDF humanoid model from Chapter 1
- [x] Ubuntu 22.04 LTS installed
- [x] Basic command-line proficiency

## Environment Setup

### 1. Install ROS 2 Humble (if not already installed)

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Gazebo Classic

```bash
# Install Gazebo 11 (Classic)
sudo apt install gazebo -y

# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-gazebo-ros2-control -y

# Verify installation
gazebo --version
# Expected: Gazebo multi-robot simulator, version 11.x.x
```

### 3. Install Additional Dependencies

```bash
# ros2_control for joint control
sudo apt install ros-humble-ros2-control -y
sudo apt install ros-humble-ros2-controllers -y

# URDF tools
sudo apt install ros-humble-xacro -y
sudo apt install ros-humble-urdf -y

# Python dependencies
pip3 install numpy matplotlib
```

### 4. Create Chapter 2 Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src/chapter2_simulation
cd ~/ros2_ws/src/chapter2_simulation

# Copy URDF from Chapter 1
cp -r ~/ros2_ws/src/chapter1_urdf/urdf ./

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Verification Steps

### Test 1: Gazebo Launches

```bash
# Launch empty Gazebo world
gazebo --verbose

# Expected: Gazebo window opens with empty world
# RTF should show ~1.0 in bottom left
# Close with Ctrl+C
```

### Test 2: ROS 2 + Gazebo Integration

```bash
# In terminal 1: Launch Gazebo with ROS 2
ros2 launch gazebo_ros gazebo.launch.py

# In terminal 2: Check ROS 2 topics
ros2 topic list

# Expected topics include:
# /clock
# /parameter_events
# /rosout
```

### Test 3: URDF Tools Work

```bash
# Validate URDF from Chapter 1
check_urdf ~/ros2_ws/src/chapter2_simulation/urdf/humanoid.urdf

# Expected: "robot name is: humanoid" with no errors
```

## Troubleshooting

### Issue: Gazebo crashes on launch

**Symptom**: Segmentation fault or black screen

**Solution**:
```bash
# Use software rendering
export LIBGL_ALWAYS_SOFTWARE=1
gazebo --verbose
```

### Issue: ROS 2 topics not appearing

**Symptom**: `ros2 topic list` shows only `/parameter_events` and `/rosout`

**Solution**:
```bash
# Ensure ROS 2 is sourced
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID
```

### Issue: URDF validation fails

**Symptom**: `check_urdf` reports missing inertia

**Solution**: Add inertia to all links in URDF:
```xml
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
</inertial>
```

### Issue: Low RTF (< 0.8)

**Symptom**: Simulation runs slowly

**Solutions**:
1. Run headless: `gzserver --verbose world.world`
2. Reduce physics rate in world file
3. Simplify robot mesh geometry

## Docker Alternative

If installation issues persist, use Docker:

```bash
# Pull ROS 2 Humble + Gazebo image
docker pull osrf/ros:humble-desktop-full

# Run with GUI support
docker run -it --rm \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  osrf/ros:humble-desktop-full

# Inside container, install Gazebo
apt update && apt install gazebo ros-humble-gazebo-ros-pkgs -y
```

## Ready to Start

Once all verification steps pass, proceed to:

**Lesson B1: What is a Digital Twin?**

```bash
cd ~/ros2_ws/src/chapter2_simulation
# Follow instructions in chapters/02-digital-twin/beginner/B1-digital-twin-concepts.md
```
