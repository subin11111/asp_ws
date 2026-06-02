#!/usr/bin/env bash

WORKSPACE="/home/subin/ros2_ws"
DIAG_DIR="${WORKSPACE}/tools/diagnostics"
LOG_DIR="${DIAG_DIR}/logs"
LOG_FILE="${LOG_DIR}/uav_runtime_$(date +%Y%m%d_%H%M%S).log"

mkdir -p "${LOG_DIR}"

{
  echo "UAV Offboard Runtime Check"
  echo "Log file: ${LOG_FILE}"
  echo
} | tee "${LOG_FILE}"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck source=/opt/ros/humble/setup.bash
  source /opt/ros/humble/setup.bash
else
  echo "WARN: /opt/ros/humble/setup.bash not found" | tee -a "${LOG_FILE}"
fi

if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  # shellcheck source=/home/subin/ros2_ws/install/setup.bash
  source "${WORKSPACE}/install/setup.bash"
else
  echo "WARN: ${WORKSPACE}/install/setup.bash not found" | tee -a "${LOG_FILE}"
fi

run_section() {
  local title="$1"

  {
    echo
    echo "================================================================================"
    echo "${title}"
    echo "================================================================================"
  } | tee -a "${LOG_FILE}"
}

run_cmd() {
  local seconds="$1"
  shift

  {
    echo
    echo "\$ $*"
  } | tee -a "${LOG_FILE}"

  timeout "${seconds}" "$@" 2>&1 | tee -a "${LOG_FILE}"
  local status=${PIPESTATUS[0]}

  if [ "${status}" -eq 124 ]; then
    echo "[timeout after ${seconds}s]" | tee -a "${LOG_FILE}"
  elif [ "${status}" -ne 0 ]; then
    echo "[command exited with status ${status}]" | tee -a "${LOG_FILE}"
  fi
}

run_shell() {
  local seconds="$1"
  local command="$2"

  {
    echo
    echo "\$ ${command}"
  } | tee -a "${LOG_FILE}"

  timeout "${seconds}" bash -lc "${command}" 2>&1 | tee -a "${LOG_FILE}"
  local status=${PIPESTATUS[0]}

  if [ "${status}" -eq 124 ]; then
    echo "[timeout after ${seconds}s]" | tee -a "${LOG_FILE}"
  elif [ "${status}" -ne 0 ]; then
    echo "[command exited with status ${status}]" | tee -a "${LOG_FILE}"
  fi
}

run_section "1) Basic Environment"
run_cmd 5 date
run_shell 5 'echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"'
run_cmd 10 ros2 node list
run_shell 10 'ros2 topic list | grep -Ei "command|fmu|clock|twist|trajectory|offboard"'

run_section "2) Topic Connection Info"
run_cmd 8 ros2 topic info -v /command/twist
run_cmd 8 ros2 topic info -v /fmu/in/trajectory_setpoint
run_cmd 8 ros2 topic info -v /fmu/in/offboard_control_mode
run_cmd 8 ros2 topic info -v /fmu/out/vehicle_control_mode
run_cmd 8 ros2 topic info -v /fmu/out/vehicle_command_ack
run_cmd 8 ros2 topic info -v /clock

run_section "3) Topic Rate Measurements"
run_cmd 8 ros2 topic hz /command/twist
run_cmd 8 ros2 topic hz /fmu/in/trajectory_setpoint
run_cmd 8 ros2 topic hz /fmu/in/offboard_control_mode
run_cmd 8 ros2 topic hz /fmu/out/vehicle_control_mode
run_cmd 8 ros2 topic hz /fmu/out/vehicle_status_v1
run_cmd 8 ros2 topic hz /clock

run_section "4) One-Shot Echo"
run_cmd 5 ros2 topic echo --once /command/twist
run_cmd 5 ros2 topic echo --once /fmu/in/trajectory_setpoint
run_cmd 5 ros2 topic echo --once /fmu/in/offboard_control_mode
run_cmd 5 ros2 topic echo --once /fmu/out/vehicle_control_mode
run_cmd 5 ros2 topic echo --once /fmu/out/vehicle_status_v1
run_cmd 5 ros2 topic echo --once /fmu/out/vehicle_command_ack
run_cmd 5 ros2 topic echo --once /clock

run_section "5) /offboard_control Parameters"
if timeout 5 ros2 node list 2>/dev/null | grep -Fxq "/offboard_control"; then
  run_cmd 8 ros2 param get /offboard_control use_sim_time
  run_cmd 8 ros2 param list /offboard_control
else
  echo "/offboard_control node not found. Skipping parameter checks." | tee -a "${LOG_FILE}"
fi

run_section "6) Judgement Hints"
cat <<'EOF' | tee -a "${LOG_FILE}"
* If /command/twist changes but /fmu/in/trajectory_setpoint does not change, suspect offboard_control callback/runtime.
* If /fmu/in/offboard_control_mode rate is below 2Hz or missing, PX4 can exit Offboard mode.
* If /clock is stopped while use_sim_time is true, timestamps may stop updating.
* If vehicle_command_ack reports rejection, inspect PX4 arming/offboard conditions.
* If relaunching turn_interfaces.launch.py fixes the issue, offboard_control internal state may be stale.
EOF

echo | tee -a "${LOG_FILE}"
echo "Done. Saved log: ${LOG_FILE}" | tee -a "${LOG_FILE}"
