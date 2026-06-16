#!/usr/bin/env bash
set -euo pipefail

root="${1:-/home/desktop1/ros2_ws/src}"
matches="$(grep -R --line-number --include='*.py' --include='*.xml' --include='*.yaml' --include='*.launch.py' 'asp_\\(uav_control\\|ugv_control\\|mission_manager\\|perception\\|precision_landing\\)' "$root"/asp_final_* || true)"
if [[ -n "$matches" ]]; then
  echo "$matches"
  exit 1
fi
echo "OK: no legacy asp_* package references found in asp_final_*"
