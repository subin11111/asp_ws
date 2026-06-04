#!/usr/bin/env bash
set -u

LOG_DIR="/home/subin/ros2_ws/tools/asp_final_diagnostics/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/final_tf_audit_$(date +%Y%m%d_%H%M%S).log"

{
  echo "=== ROS nodes ==="
  ros2 node list | grep -Ei "asp_final|tf|bridge" || true

  echo
  echo "=== ROS topics related to tf/pose ==="
  ros2 topic list | grep -Ei "tf|pose|X1_asp|x500|model" || true

  echo
  echo "=== Gazebo topics related to pose ==="
  gz topic -l | grep -Ei "X1_asp|x500|pose" || true

  echo
  echo "=== TF echo map -> X1_asp/base_link ==="
  timeout 5 ros2 run tf2_ros tf2_echo map X1_asp/base_link || true

  echo
  echo "=== TF echo map -> x500_gimbal_0/base_link ==="
  timeout 5 ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link || true

  echo
  echo "=== /tf once ==="
  timeout 5 ros2 topic echo /tf --once || true

  echo
  echo "=== /tf_static once ==="
  timeout 5 ros2 topic echo /tf_static --once || true
} | tee "$LOG_FILE"

echo "Saved: $LOG_FILE"
