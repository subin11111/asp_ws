#!/usr/bin/env bash
set -u

LOG_DIR="/home/subin/ros2_ws/tools/asp_final_diagnostics/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/final_pipeline_audit_$(date +%Y%m%d_%H%M%S).log"

echo_block() {
  echo
  echo "=== $1 ==="
}

echo_topic_once() {
  local topic="$1"
  echo_block "$topic"
  timeout 4 ros2 topic echo "$topic" --once || true
}

{
  echo_block "ROS nodes"
  ros2 node list | sort | grep -Ei "asp_final|micro|xrce|agent|bridge" || true

  echo_block "PX4 /fmu topics"
  ros2 topic list | sort | grep '^/fmu/' || true

  echo_block "Mission topics"
  ros2 topic list | sort | grep '^/asp_final/' || true

  echo_topic_once "/asp_final/mission/state"
  echo_topic_once "/asp_final/mission/status"
  echo_topic_once "/asp_final/ugv/state"
  echo_topic_once "/asp_final/uav/exploration_state"
  echo_topic_once "/asp_final/uav/exploration_event"
  echo_topic_once "/asp_final/uav/cmd_pose"
  echo_topic_once "/asp_final/uav/mission2_takeoff_origin"
  echo_topic_once "/asp_final/px4/status"
  echo_topic_once "/asp_final/perception/uav/marker_detections"
  echo_topic_once "/asp_final/perception/uav/marker_id"
  echo_topic_once "/asp_final/perception/landing/marker_detections"
  echo_topic_once "/asp_final/perception/landing/marker_id"

  echo_topic_once "/fmu/out/vehicle_local_position"
  echo_topic_once "/fmu/out/vehicle_control_mode"
  echo_topic_once "/fmu/out/vehicle_status"
  echo_topic_once "/fmu/out/vehicle_status_v1"
  echo_topic_once "/fmu/out/vehicle_command_ack"
  echo_topic_once "/fmu/in/offboard_control_mode"
  echo_topic_once "/fmu/in/trajectory_setpoint"
  echo_topic_once "/fmu/in/vehicle_command"

  echo_block "TF map -> X1_asp/base_link"
  timeout 4 ros2 run tf2_ros tf2_echo map X1_asp/base_link || true

  echo_block "TF map -> x500_gimbal_0/base_link"
  timeout 4 ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link || true

  echo_block "Path audit"
  ros2 run asp_final_tools final_path_audit || python3 /home/subin/ros2_ws/src/asp_final_tools/asp_final_tools/final_path_audit.py || true
} | tee "$LOG_FILE"

echo "Saved: $LOG_FILE"
