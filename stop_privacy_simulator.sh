#!/bin/bash

# ============================================================
# Care-O-Bot Privacy Environment Simulation - Shutdown Script
# ============================================================
#
# This script stops all processes that were started by the
# privacy environment simulation startup script.
#
# The simulation consists of several ROS components:
#
#   1. Keyboard teleoperation
#   2. YOLO privacy/object detectors
#   3. Object position estimation nodes
#   4. Privacy marker nodes
#   5. SLAM using GMapping
#   6. RViz visualization
#   7. Care-O-Bot ROS/Gazebo launch process
#   8. Gazebo server and client
#
# The processes are stopped in a controlled order. Components
# that depend on the simulator are stopped before the simulator
# and Gazebo are terminated.
#
# NOTE:
# This script uses `pkill -f`, which searches the complete
# command line of running processes. Therefore, the matching
# strings should be chosen carefully to avoid terminating
# unrelated processes with similar names.
# ============================================================

echo "=========================================="
echo " Stopping privacy environment simulation"
echo "=========================================="

# ------------------------------------------------------------
# 1. Stop teleoperation
# ------------------------------------------------------------

echo "Stopping keyboard teleoperation..."

# Terminate the keyboard teleoperation process.
pkill -f "teleop_twist_keyboard.py"


# ------------------------------------------------------------
# 2. Stop privacy environment nodes
# ------------------------------------------------------------

echo "Stopping privacy environment nodes..."

# Stop all running instances of the privacy detector.
pkill -f "privacy_detector.py"

# Stop all object-position processing nodes.
pkill -f "object_position.py"

# Stop all privacy marker nodes.
pkill -f "privacy_marker.py"


# ------------------------------------------------------------
# 3. Stop SLAM
# ------------------------------------------------------------

echo "Stopping SLAM..."

# Stop the SLAM node.
pkill -f "slam_gmapping"


# ------------------------------------------------------------
# 4. Stop RViz
# ------------------------------------------------------------

echo "Stopping RViz..."

# Terminate running RViz processes.
pkill -f "rviz"


# ------------------------------------------------------------
# 5. Stop Care-O-Bot simulator
# ------------------------------------------------------------

echo "Stopping Care-O-Bot simulator..."

# Terminate the Care-O-Bot simulation launch process.
pkill -f "roslaunch cob_bringup_sim robot.launch"


# ------------------------------------------------------------
# 6. Stop Gazebo
# ------------------------------------------------------------

sleep 2

echo "Stopping Gazebo..."

# Stop Gazebo server
pkill -f "gzserver"

# Stop Gazebo client
pkill -f "gzclient"


# ------------------------------------------------------------
# 7. Final cleanup
# ------------------------------------------------------------

sleep 2

echo ""
echo "=========================================="
echo " All simulation processes stopped."
echo "=========================================="

