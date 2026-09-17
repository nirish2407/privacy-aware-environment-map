#!/bin/bash

echo "=========================================="
echo " Stopping privacy environment simulation"
echo "=========================================="

# ------------------------------------------------------------
# 1. Stop teleoperation
# ------------------------------------------------------------

echo "Stopping keyboard teleoperation..."

pkill -f "teleop_twist_keyboard.py"


# ------------------------------------------------------------
# 2. Stop privacy environment nodes
# ------------------------------------------------------------

echo "Stopping privacy environment nodes..."

pkill -f "privacy_detector.py"
pkill -f "object_position.py"
pkill -f "privacy_marker.py"


# ------------------------------------------------------------
# 3. Stop SLAM
# ------------------------------------------------------------

echo "Stopping SLAM..."

pkill -f "slam_gmapping"


# ------------------------------------------------------------
# 4. Stop RViz
# ------------------------------------------------------------

echo "Stopping RViz..."

pkill -f "rviz"


# ------------------------------------------------------------
# 5. Stop Care-O-Bot simulator
# ------------------------------------------------------------

echo "Stopping Care-O-Bot simulator..."

pkill -f "roslaunch cob_bringup_sim robot.launch"


# ------------------------------------------------------------
# 6. Stop Gazebo
# ------------------------------------------------------------

sleep 2

echo "Stopping Gazebo..."

pkill -f "gzserver"
pkill -f "gzclient"


# ------------------------------------------------------------
# 7. Final cleanup
# ------------------------------------------------------------

sleep 2

echo ""
echo "=========================================="
echo " All simulation processes stopped."
echo "=========================================="

