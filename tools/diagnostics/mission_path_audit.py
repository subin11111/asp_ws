#!/usr/bin/env python3
import csv
import math
from datetime import datetime
from pathlib import Path


WORKSPACE = Path('/home/subin/ros2_ws')
LOG_DIR = WORKSPACE / 'tools' / 'diagnostics' / 'logs'
UAV_PATH = WORKSPACE / 'src' / 'asp_uav_control' / 'path' / 'uav_path_mission2_senior.csv'
GENERATED_UAV_PATH = WORKSPACE / 'src' / 'asp_uav_control' / 'path' / 'uav_path_generated.csv'
UGV_PATH = WORKSPACE / 'src' / 'asp_ugv_control' / 'path' / 'mission3_rendezvous_senior.csv'
UAV_CONFIG = WORKSPACE / 'src' / 'asp_uav_control' / 'config' / 'uav_exploration_params.yaml'
MISSION2_LAUNCH = WORKSPACE / 'src' / 'asp_uav_control' / 'launch' / 'uav_exploration_mission2.launch.py'
FULL_MISSION_LAUNCH = WORKSPACE / 'src' / 'asp_mission_manager' / 'launch' / 'full_mission.launch.py'
UAV_NODE = WORKSPACE / 'src' / 'asp_uav_control' / 'asp_uav_control' / 'uav_exploration_node.py'
UGV_FOLLOWER = WORKSPACE / 'src' / 'asp_ugv_control' / 'src' / 'ugv_path_follower_node.cpp'
UGV_RENDEZVOUS = WORKSPACE / 'src' / 'asp_ugv_control' / 'src' / 'ugv_rendezvous_node.cpp'


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
    if 'senior' not in UAV_PATH.name:
        line(f'ERROR runtime UAV path is not senior-named: {UAV_PATH.name}', output)
    if UAV_PATH == GENERATED_UAV_PATH or 'generated' in UAV_PATH.name:
        line(f'ERROR generated UAV path selected for runtime: {UAV_PATH}', output)
    rows = read_csv(UAV_PATH)
    line(f'waypoints={len(rows)}', output)
    previous = None
    warnings = 0
    forbidden_hits = 0
    max_jump = 0.0
    for index, row in enumerate(rows):
        x = as_float(row, 'x')
        y = as_float(row, 'y')
        z = as_float(row, 'z')
        tag = row.get('tag', '')
        if previous is not None:
            distance = math.hypot(x - previous[0], y - previous[1])
            max_jump = max(max_jump, distance)
        if -140.0 <= x <= -115.0 and 35.0 <= y <= 68.0:
            forbidden_hits += 1
            warnings += 1
            line(
                f'WARN UAV senior waypoint in Mission1 return zone index={index} '
                f'tag={tag} x={x:.2f} y={y:.2f}',
                output)
        if z < 10.0:
            warnings += 1
            line(f'WARN low UAV waypoint index={index} tag={tag} z={z:.2f}', output)
        previous = (x, y, z)
    line(f'max_uav_xy_jump={max_jump:.2f}', output)
    line(f'forbidden_return_zone_hits={forbidden_hits}', output)
    line(f'uav_warnings={warnings}', output)


def audit_ugv_path(output: list):
    line(f'=== UGV rendezvous path audit: {UGV_PATH} ===', output)
    if 'mission3_rendezvous_senior' not in UGV_PATH.name:
        line(f'ERROR runtime UGV path is not mission3 senior path: {UGV_PATH.name}', output)
    rows = read_csv(UGV_PATH)
    line(f'waypoints={len(rows)}', output)
    warnings = 0
    if rows:
        final = rows[-1]
        final_x = as_float(final, 'x')
        final_y = as_float(final, 'y')
        endpoint_error = math.hypot(final_x - (-57.949), final_y - 101.780)
        line(f'final_endpoint=({final_x:.3f},{final_y:.3f}) error={endpoint_error:.3f}', output)
        if endpoint_error > 0.5:
            warnings += 1
            line('WARN UGV rendezvous endpoint differs from configured final point', output)
    for index, row in enumerate(rows):
        speed = as_float(row, 'target_speed')
        if speed > 1.4:
            warnings += 1
            line(f'WARN UGV speed > 1.4 index={index} speed={speed:.2f}', output)
    line(f'ugv_warnings={warnings}', output)


def audit_config(output: list):
    line(f'=== Mission2 config audit: {UAV_CONFIG} ===', output)
    text = UAV_CONFIG.read_text()
    required = {
        'allow_generated_path_runtime': 'false',
        'runtime_path_must_contain': '"senior"',
        'forbidden_return_zone_enabled': 'true',
        'continue_on_marker_timeout': 'true',
        'MARKER_TIMEOUT_CONTINUE': None,
        'waypoint_stuck_timeout_sec': None,
        'waypoint_tolerance': '1.8',
        'yaw_tolerance_deg': '45.0',
        'ignore_yaw_for_waypoint_reached': 'true',
        'skip_close_waypoints': 'true',
        'min_waypoint_separation': '4.0',
        'complete_on_path_done': 'true',
        'minimum_unique_markers': '0',
    }
    for key, expected in required.items():
        if key == 'MARKER_TIMEOUT_CONTINUE':
            source = UAV_NODE.read_text()
            ok = key in source
        elif expected is None:
            ok = key in text
        else:
            ok = f'{key}: {expected}' in text
        line(f'{key}: {"OK" if ok else "MISSING"}', output)

    launch_text = MISSION2_LAUNCH.read_text()
    full_launch_text = FULL_MISSION_LAUNCH.read_text()
    static_spawn_used = 'uav_path_safe.csv' in launch_text or '-55' in launch_text or '80.000000' in launch_text
    line(f'runtime_launch_static_spawn_prefix={"WARN" if static_spawn_used else "OK"}', output)
    old_takeoff_param = 'force' + '_takeoff' + '_before_path'
    fixed_takeoff_removed = old_takeoff_param not in launch_text
    line(f'fixed_takeoff_prefix_removed={"OK" if fixed_takeoff_removed else "WARN"}', output)
    senior_launch = 'uav_path_mission2_senior.csv' in launch_text
    line(f'uav_mission2_launch_senior_path={"OK" if senior_launch else "MISSING"}', output)
    full_senior = (
        'uav_path_mission2_senior.csv' in full_launch_text
        and 'mission3_rendezvous_senior.csv' in full_launch_text
    )
    line(f'full_mission_senior_paths={"OK" if full_senior else "MISSING"}', output)


def audit_source_guards(output: list):
    line('=== Source guard audit ===', output)
    uav_source = UAV_NODE.read_text()
    ugv_source = UGV_FOLLOWER.read_text()
    rendezvous_source = UGV_RENDEZVOUS.read_text()
    checks = {
        'POSE_BLOCKED_FORBIDDEN_RETURN_ZONE': 'POSE_BLOCKED_FORBIDDEN_RETURN_ZONE' in uav_source,
        'WAYPOINT_FORBIDDEN_RETURN_ZONE_SKIP': 'WAYPOINT_FORBIDDEN_RETURN_ZONE_SKIP' in uav_source,
        'allow_generated_path_runtime': 'allow_generated_path_runtime' in uav_source,
        'runtime_path_must_contain': 'runtime_path_must_contain' in uav_source,
        'RENDEZVOUS_START_DELAY': 'RENDEZVOUS_START_DELAY' in rendezvous_source,
        'MISSION1_CMD_RELEASED': 'MISSION1_CMD_RELEASED' in ugv_source,
    }
    for key, ok in checks.items():
        line(f'{key}: {"OK" if ok else "MISSING"}', output)


def main():
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    output = []
    line('Mission path audit', output)
    line(f'timestamp={datetime.now().isoformat(timespec="seconds")}', output)
    audit_uav_path(output)
    audit_ugv_path(output)
    audit_config(output)
    audit_source_guards(output)
    log_path = LOG_DIR / f'mission_path_audit_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'
    log_path.write_text('\n'.join(output) + '\n')
    line(f'log_file={log_path}', output)


if __name__ == '__main__':
    main()
