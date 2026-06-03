#!/usr/bin/env python3
import csv
import math
from datetime import datetime
from pathlib import Path


WORKSPACE = Path('/home/subin/ros2_ws')
LOG_DIR = WORKSPACE / 'tools' / 'diagnostics' / 'logs'
UAV_PATH = WORKSPACE / 'src' / 'asp_uav_control' / 'path' / 'uav_path_mission2.csv'
UGV_PATH = WORKSPACE / 'src' / 'asp_ugv_control' / 'path' / 'rendezvous.csv'
UAV_CONFIG = WORKSPACE / 'src' / 'asp_uav_control' / 'config' / 'uav_exploration_params.yaml'
MISSION2_LAUNCH = WORKSPACE / 'src' / 'asp_uav_control' / 'launch' / 'uav_exploration_mission2.launch.py'


def read_csv(path: Path):
    with path.open(newline='') as csvfile:
        return list(csv.DictReader(csvfile))


def as_float(row, key, default=0.0):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def line(message: str, output: list):
    print(message)
    output.append(message)


def audit_uav_path(output: list):
    line(f'=== UAV path audit: {UAV_PATH} ===', output)
    rows = read_csv(UAV_PATH)
    line(f'waypoints={len(rows)}', output)
    previous = None
    warnings = 0
    for index, row in enumerate(rows):
        x = as_float(row, 'x')
        y = as_float(row, 'y')
        z = as_float(row, 'z')
        tag = row.get('tag', '')
        if previous is not None:
            distance = math.hypot(x - previous[0], y - previous[1])
            if distance > 18.0 and z < 18.0:
                warnings += 1
                line(
                    f'WARN long low UAV transition index={index} tag={tag} '
                    f'distance={distance:.2f} z={z:.2f}',
                    output)
            if distance > 35.0:
                warnings += 1
                line(
                    f'WARN large UAV jump index={index} tag={tag} distance={distance:.2f}',
                    output)
        if z < 12.0 and 'scan' not in tag.lower():
            warnings += 1
            line(f'WARN low non-scan UAV waypoint index={index} tag={tag} z={z:.2f}', output)
        previous = (x, y, z)
    line(f'uav_warnings={warnings}', output)


def audit_ugv_path(output: list):
    line(f'=== UGV rendezvous path audit: {UGV_PATH} ===', output)
    rows = read_csv(UGV_PATH)
    line(f'waypoints={len(rows)}', output)
    warnings = 0
    for index, row in enumerate(rows):
        speed = as_float(row, 'target_speed')
        if speed > 0.8:
            warnings += 1
            line(f'WARN UGV speed > 0.8 index={index} speed={speed:.2f}', output)
    line(f'ugv_warnings={warnings}', output)


def audit_config(output: list):
    line(f'=== Mission2 config audit: {UAV_CONFIG} ===', output)
    text = UAV_CONFIG.read_text()
    required = {
        'continue_on_marker_timeout': 'true',
        'MARKER_TIMEOUT_CONTINUE': None,
        'waypoint_stuck_timeout_sec': None,
        'complete_on_path_done': 'true',
        'minimum_unique_markers': '0',
    }
    for key, expected in required.items():
        if key == 'MARKER_TIMEOUT_CONTINUE':
            source = (WORKSPACE / 'src' / 'asp_uav_control' / 'asp_uav_control' /
                      'uav_exploration_node.py').read_text()
            ok = key in source
        elif expected is None:
            ok = key in text
        else:
            ok = f'{key}: {expected}' in text
        line(f'{key}: {"OK" if ok else "MISSING"}', output)

    launch_text = MISSION2_LAUNCH.read_text()
    static_spawn_used = 'uav_path_safe.csv' in launch_text or '-55' in launch_text or '80.000000' in launch_text
    line(f'runtime_launch_static_spawn_prefix={"WARN" if static_spawn_used else "OK"}', output)
    forced_takeoff = 'force_takeoff_before_path' in launch_text and 'True' in launch_text
    line(f'forced_takeoff_override={"OK" if forced_takeoff else "MISSING"}', output)


def main():
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    output = []
    line('Mission path audit', output)
    line(f'timestamp={datetime.now().isoformat(timespec="seconds")}', output)
    audit_uav_path(output)
    audit_ugv_path(output)
    audit_config(output)
    log_path = LOG_DIR / f'mission_path_audit_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'
    log_path.write_text('\n'.join(output) + '\n')
    line(f'log_file={log_path}', output)


if __name__ == '__main__':
    main()
