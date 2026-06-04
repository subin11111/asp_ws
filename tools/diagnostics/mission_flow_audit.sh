#!/usr/bin/env bash
set -uo pipefail

WORKSPACE="/home/subin/ros2_ws"
LOG_DIR="${WORKSPACE}/tools/diagnostics/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/mission_flow_audit_${STAMP}.log"

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
  echo "Mission flow audit"
  echo "workspace=${WORKSPACE}"
  echo "timestamp=${STAMP}"
} | tee "${LOG_FILE}"

run_section "mission topic graph" bash -lc \
  "ros2 topic list | grep -E 'mission|uav|ugv|command/pose|command/ugv_cmd_vel|landing' | sort"

run_section "/command/pose publishers" ros2 topic info -v /command/pose
run_section "/command/ugv_cmd_vel publishers" ros2 topic info -v /command/ugv_cmd_vel

run_topic_once /mission/state
run_topic_once /mission/status
run_topic_once /ugv/mission_event
run_topic_once /uav/exploration_event
run_topic_once /uav/exploration_state
run_topic_once /uav/mission2_takeoff_origin
run_topic_once /ugv/rendezvous_start
run_topic_once /ugv/rendezvous_state
run_topic_once /ugv/rendezvous_reached
run_topic_once /mission/uav_exploration_complete
run_topic_once /mission/precision_landing_start
run_topic_once /status/landing_complete

run_section "static Mission2/Mission3 parallel start" \
  grep -Rni "MISSION2_START_REACHED\\|PARALLEL_MISSION2_3_STARTED\\|UGV_RENDEZVOUS_START_PUBLISHED" \
    "${WORKSPACE}/src/asp_mission_manager" "${WORKSPACE}/docs"

run_section "static Mission2 UAV senior path guards" \
  grep -Rni "uav_path_mission2_senior\\|allow_generated_path_runtime\\|runtime_path_must_contain\\|forbidden_return_zone\\|POSE_BLOCKED_FORBIDDEN_RETURN_ZONE" \
    "${WORKSPACE}/src/asp_uav_control" "${WORKSPACE}/tools" "${WORKSPACE}/docs"

run_section "static Mission3 UGV senior path and release events" \
  grep -Rni "mission3_rendezvous_senior\\|start_delay_sec\\|RENDEZVOUS_START_DELAY\\|MISSION1_CMD_RELEASED\\|RENDEZVOUS_REACHED" \
    "${WORKSPACE}/src/asp_ugv_control" "${WORKSPACE}/src/asp_mission_manager" "${WORKSPACE}/tools" "${WORKSPACE}/docs"

run_section "static Mission4 dual-condition landing gate" \
  grep -Rni "uav_exploration_complete\\|rendezvous_reached\\|precision_landing_start\\|PRECISION_LANDING_READY" \
    "${WORKSPACE}/src/asp_mission_manager" "${WORKSPACE}/docs"

echo "log_file=${LOG_FILE}" | tee -a "${LOG_FILE}"
