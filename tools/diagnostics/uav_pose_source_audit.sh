#!/usr/bin/env bash
set -uo pipefail

WORKSPACE="/home/subin/ros2_ws"
LOG_DIR="${WORKSPACE}/tools/diagnostics/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/uav_pose_source_audit_${STAMP}.log"

mkdir -p "${LOG_DIR}"

run_section() {
  local title="$1"
  shift
  {
    echo
    echo "=== ${title} ==="
    "$@"
  } 2>&1 | tee -a "${LOG_FILE}"
}

run_topic_once() {
  local topic="$1"
  run_section "topic echo --once ${topic}" timeout 5s ros2 topic echo --once "${topic}"
}

{
  echo "UAV pose source audit"
  echo "workspace=${WORKSPACE}"
  echo "timestamp=${STAMP}"
} | tee "${LOG_FILE}"

run_section "/command/pose topic info" ros2 topic info -v /command/pose

run_topic_once /mission/state
run_topic_once /mission/status
run_topic_once /ugv/mission_event
run_topic_once /uav/mission2_takeoff_origin
run_topic_once /uav/exploration_event
run_topic_once /uav/exploration_state
run_topic_once /ugv/rendezvous_start
run_topic_once /ugv/rendezvous_reached
run_topic_once /mission/uav_exploration_complete
run_topic_once /mission/precision_landing_start
run_topic_once /command/pose
run_topic_once /debug/offboard/input_pose_enu
run_topic_once /debug/offboard/local_pose_enu
run_topic_once /debug/offboard/setpoint_pose_ned
run_topic_once /debug/offboard/frame_report
run_topic_once /fmu/out/vehicle_local_position
run_topic_once /fmu/in/trajectory_setpoint

run_section "static /command/pose references" \
  grep -Rni "/command/pose" "${WORKSPACE}/src" "${WORKSPACE}/tools"

run_section "static spawn coordinate references" \
  grep -Rni -- "-55\\|80.000000\\|-55.000000\\|-55 80" \
    "${WORKSPACE}/src/asp_uav_control" "${WORKSPACE}/tools"

run_section "static safe path and prefix references" \
  grep -Rni "uav_path_mission2_senior\\|uav_path_generated\\|uav_path_safe\\|takeoff_climb\\|safe_altitude\\|transition_\\|allow_generated_path_runtime\\|runtime_path_must_contain" \
    "${WORKSPACE}/src/asp_uav_control" "${WORKSPACE}/tools"

run_section "static forbidden return zone references" \
  grep -Rni "forbidden_return_zone\\|POSE_BLOCKED_FORBIDDEN_RETURN_ZONE\\|WAYPOINT_FORBIDDEN_RETURN_ZONE_SKIP" \
    "${WORKSPACE}/src/asp_uav_control" "${WORKSPACE}/tools" "${WORKSPACE}/docs"

run_section "static Mission2/Mission3 parallel start references" \
  grep -Rni "start_rendezvous_on_mission2_start\\|PARALLEL_MISSION2_3_STARTED\\|UGV_RENDEZVOUS_START_PUBLISHED" \
    "${WORKSPACE}/src/asp_mission_manager" "${WORKSPACE}/docs"

run_section "static external origin one-shot latch references" \
  grep -Rni "duplicate_origin_ignored\\|allow_external_origin_reanchor\\|mission2_px4_anchor" \
    "${WORKSPACE}/src/utilities_pkg/px4_ros_com/src/examples/offboard" "${WORKSPACE}/docs"

run_section "static Mission2 takeoff origin publish references" \
  grep -Rni "republish_mission2_origin\\|MISSION2_TAKEOFF_ORIGIN_PUBLISHED_ONCE" \
    "${WORKSPACE}/src/asp_uav_control" "${WORKSPACE}/docs"

echo "log_file=${LOG_FILE}" | tee -a "${LOG_FILE}"
