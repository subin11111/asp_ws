#!/usr/bin/env bash
set -uo pipefail

WORKSPACE="/home/subin/ros2_ws"
LOG_DIR="${WORKSPACE}/tools/diagnostics/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/ugv_cmd_source_audit_${STAMP}.log"

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
  echo "UGV command source audit"
  echo "workspace=${WORKSPACE}"
  echo "timestamp=${STAMP}"
} | tee "${LOG_FILE}"

run_section "/command/ugv_cmd_vel topic info" ros2 topic info -v /command/ugv_cmd_vel
run_topic_once /command/ugv_cmd_vel
run_topic_once /ugv/state
run_topic_once /ugv/mission_event
run_topic_once /ugv/rendezvous_start
run_topic_once /ugv/rendezvous_reached

run_section "static UGV cmd topic references" \
  grep -Rni "/command/ugv_cmd_vel" "${WORKSPACE}/src" "${WORKSPACE}/tools"

run_section "static UGV stop/release references" \
  grep -Rni "publish_zero_twist\\|stopped_\\|zero_publish_after_stop\\|disable_cmd_after_stop" \
    "${WORKSPACE}/src/asp_ugv_control"

run_section "static UGV rendezvous speed references" \
  grep -Rni "cruise_speed\\|lookahead_distance\\|max_linear_speed\\|RENDEZVOUS_LOOKAHEAD_SKIP" \
    "${WORKSPACE}/src/asp_ugv_control"

echo "log_file=${LOG_FILE}" | tee -a "${LOG_FILE}"
