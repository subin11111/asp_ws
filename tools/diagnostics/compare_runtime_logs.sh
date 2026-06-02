#!/usr/bin/env bash

WORKSPACE="/home/subin/ros2_ws"
LOG_DIR="${WORKSPACE}/tools/diagnostics/logs"

if [ ! -d "${LOG_DIR}" ]; then
  echo "No logs directory found: ${LOG_DIR}"
  echo "Run: bash tools/diagnostics/uav_offboard_runtime_check.sh"
  exit 0
fi

mapfile -t LOGS < <(find "${LOG_DIR}" -maxdepth 1 -type f -name 'uav_runtime_*.log' -printf '%T@ %p\n' | sort -nr | awk '{print $2}' | head -2)

if [ "${#LOGS[@]}" -lt 2 ]; then
  echo "Need at least two runtime logs in ${LOG_DIR}."
  echo "Run the diagnostic script once during normal behavior and once after the keyboard issue appears."
  exit 0
fi

NEW_LOG="${LOGS[0]}"
OLD_LOG="${LOGS[1]}"

echo "Latest log:   ${NEW_LOG}"
echo "Previous log: ${OLD_LOG}"
echo

KEYWORDS=(
  "topic hz"
  "average rate"
  "vehicle_command_ack"
  "vehicle_control_mode"
  "use_sim_time"
  "clock"
  "trajectory_setpoint"
  "offboard_control_mode"
)

for keyword in "${KEYWORDS[@]}"; do
  echo "================================================================================"
  echo "Keyword: ${keyword}"
  echo "================================================================================"
  echo "--- Previous log ---"
  grep -inC 3 -- "${keyword}" "${OLD_LOG}" || echo "(no matches)"
  echo
  echo "--- Latest log ---"
  grep -inC 3 -- "${keyword}" "${NEW_LOG}" || echo "(no matches)"
  echo
done
