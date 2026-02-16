#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Physical AI & Humanoid Robotics Textbook
#
# Chapter 1: The Robotic Nervous System (ROS 2)
# Lesson: B1 - Introduction to ROS 2
# Example: Demo commands for verifying ROS 2 installation

# =============================================================================
# ROS 2 DEMO COMMANDS
# =============================================================================
# This script demonstrates basic ROS 2 commands for the talker/listener demo.
# Run each section in separate terminals as indicated.
#
# Prerequisites:
#   - ROS 2 Humble installed
#   - Environment sourced: source /opt/ros/humble/setup.bash
# =============================================================================

echo "=========================================="
echo "ROS 2 Humble Demo Commands"
echo "=========================================="

# Verify ROS 2 installation
echo ""
echo "1. Verifying ROS 2 installation..."
echo "   Command: ros2 --version"
ros2 --version

# Check available packages
echo ""
echo "2. Checking demo packages..."
echo "   Command: ros2 pkg list | grep demo"
ros2 pkg list | grep demo

# Instructions for running the demo
echo ""
echo "=========================================="
echo "TALKER/LISTENER DEMO"
echo "=========================================="
echo ""
echo "Open THREE terminals and run these commands:"
echo ""
echo "TERMINAL 1 - Start the Talker:"
echo "  source /opt/ros/humble/setup.bash"
echo "  ros2 run demo_nodes_cpp talker"
echo ""
echo "TERMINAL 2 - Start the Listener:"
echo "  source /opt/ros/humble/setup.bash"
echo "  ros2 run demo_nodes_cpp listener"
echo ""
echo "TERMINAL 3 - Explore with CLI:"
echo "  source /opt/ros/humble/setup.bash"
echo "  ros2 node list          # List running nodes"
echo "  ros2 topic list         # List active topics"
echo "  ros2 topic echo /chatter  # See messages"
echo "  ros2 topic info /chatter  # Topic details"
echo ""
echo "=========================================="
echo "USEFUL ROS 2 CLI COMMANDS"
echo "=========================================="
echo ""
echo "Node Commands:"
echo "  ros2 node list              # List all nodes"
echo "  ros2 node info <node_name>  # Info about a node"
echo ""
echo "Topic Commands:"
echo "  ros2 topic list             # List all topics"
echo "  ros2 topic echo <topic>     # Print topic messages"
echo "  ros2 topic info <topic>     # Info about a topic"
echo "  ros2 topic hz <topic>       # Measure publish rate"
echo ""
echo "Service Commands:"
echo "  ros2 service list           # List all services"
echo "  ros2 service call <service> <type> <args>  # Call a service"
echo ""
echo "Action Commands:"
echo "  ros2 action list            # List all actions"
echo "  ros2 action info <action>   # Info about an action"
echo ""
echo "Package Commands:"
echo "  ros2 pkg list               # List all packages"
echo "  ros2 pkg executables <pkg>  # List executables in package"
echo ""
echo "=========================================="
echo "Press Enter to exit..."
read
