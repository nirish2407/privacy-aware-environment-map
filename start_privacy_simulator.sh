#!/bin/bash

# ============================================================
# Care-O-Bot + SLAM + YOLO + Object Position + Privacy Markers
# ============================================================

# ROS environment
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Robot configuration
export ROBOT=cob4-5
export ROBOT_ENV=ipa-Nirish

# Create log directory
LOG_DIR="$HOME/privacy_sim_logs"
mkdir -p "$LOG_DIR"

echo "=========================================="
echo " Starting Care-O-Bot privacy simulation"
echo "=========================================="

# ------------------------------------------------------------
# 1. Start Care-O-Bot simulator
# ------------------------------------------------------------

echo "[1/5] Starting Care-O-Bot simulator..."

roslaunch cob_bringup_sim robot.launch \
    > "$LOG_DIR/simulator.log" 2>&1 &

SIM_PID=$!

echo "Simulator PID: $SIM_PID"

# Give Gazebo/ROS time to initialize
echo "Waiting for simulator..."
sleep 120


# ------------------------------------------------------------
# 2. Start SLAM
# ------------------------------------------------------------

echo "[2/5] Starting SLAM..."

rosrun gmapping slam_gmapping \
    scan:=/scan_unified \
    _base_frame:=base_link \
    _odom_frame:=odom_combined \
    > "$LOG_DIR/slam.log" 2>&1 &

SLAM_PID=$!

echo "SLAM PID: $SLAM_PID"

sleep 3


# ------------------------------------------------------------
# 3. Start YOLO detectors
# ------------------------------------------------------------

echo "[3/5] Starting YOLO detectors..."

# LEFT camera
rosrun privacy_environment_map privacy_detector.py \
    _camera_id:=left \
    _node_name:=privacy_detector_left \
    > "$LOG_DIR/yolo_left.log" 2>&1 &

YOLO_LEFT_PID=$!

# RIGHT camera
rosrun privacy_environment_map privacy_detector.py \
    _camera_id:=right \
    _node_name:=privacy_detector_right \
    > "$LOG_DIR/yolo_right.log" 2>&1 &

YOLO_RIGHT_PID=$!

echo "YOLO left PID:  $YOLO_LEFT_PID"
echo "YOLO right PID: $YOLO_RIGHT_PID"

sleep 3


# ------------------------------------------------------------
# 4. Start object position nodes
# ------------------------------------------------------------

echo "[4/5] Starting object position nodes..."

# LEFT camera
rosrun privacy_environment_map object_position.py \
    _camera_id:=left \
    __name:=object_position_left \
    > "$LOG_DIR/object_position_left.log" 2>&1 &

OBJECT_LEFT_PID=$!

# RIGHT camera
rosrun privacy_environment_map object_position.py \
    _camera_id:=right \
    __name:=object_position_right \
    > "$LOG_DIR/object_position_right.log" 2>&1 &

OBJECT_RIGHT_PID=$!

echo "Object position left PID:  $OBJECT_LEFT_PID"
echo "Object position right PID: $OBJECT_RIGHT_PID"

sleep 3


# ------------------------------------------------------------
# 5. Start privacy marker nodes
# ------------------------------------------------------------

echo "[5/5] Starting privacy markers..."

# LEFT camera
rosrun privacy_environment_map privacy_marker.py \
    _camera_id:=left \
    __name:=privacy_marker_left \
    > "$LOG_DIR/privacy_marker_left.log" 2>&1 &

MARKER_LEFT_PID=$!

# RIGHT camera
rosrun privacy_environment_map privacy_marker.py \
    _camera_id:=right \
    __name:=privacy_marker_right \
    > "$LOG_DIR/privacy_marker_right.log" 2>&1 &

MARKER_RIGHT_PID=$!

echo "Privacy marker left PID:  $MARKER_LEFT_PID"
echo "Privacy marker right PID: $MARKER_RIGHT_PID"


# ------------------------------------------------------------
# 6. Start RViz
# ------------------------------------------------------------

echo "[6/7] Starting RViz..."

rviz \
    > "$LOG_DIR/rviz.log" 2>&1 &

RVIZ_PID=$!

echo "RViz PID: $RVIZ_PID"

sleep 3


# ------------------------------------------------------------
# 7. Start keyboard teleoperation
# ------------------------------------------------------------

echo "[7/7] Starting keyboard teleoperation..."

gnome-terminal -- bash -c "
    source /opt/ros/noetic/setup.bash
    source ~/catkin_ws/devel/setup.bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py \
        cmd_vel:=/base/twist_mux/command_navigation
"

echo "Teleoperation started in a new terminal."


# ------------------------------------------------------------
# Summary
# ------------------------------------------------------------

echo ""
echo "=========================================="
echo " All processes started"
echo "=========================================="
echo ""
echo "Simulator:       $SIM_PID"
echo "SLAM:            $SLAM_PID"
echo "YOLO left:       $YOLO_LEFT_PID"
echo "YOLO right:      $YOLO_RIGHT_PID"
echo "Object left:     $OBJECT_LEFT_PID"
echo "Object right:    $OBJECT_RIGHT_PID"
echo "Marker left:     $MARKER_LEFT_PID"
echo "Marker right:    $MARKER_RIGHT_PID"
echo "RViz:            $RVIZ_PID"
echo ""
echo "Logs:"
echo "$LOG_DIR"
echo ""
echo "Use 'ps aux | grep privacy_environment_map' to check nodes."
echo "Use 'rosnode list' to check ROS nodes."
echo ""
echo "=========================================="
echo " Use the new teleop terminal to move robot"
echo "=========================================="

