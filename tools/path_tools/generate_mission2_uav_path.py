#!/usr/bin/env python3
import argparse
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List


DEFAULT_MARKERS = Path('/home/subin/ros2_ws/tools/path_tools/marker_poses.csv')
DEFAULT_OUTPUT = Path('/home/subin/ros2_ws/src/asp_uav_control/path/uav_path_mission2.csv')


@dataclass
class Marker:
    marker_id: int
    x: float
    y: float
    z: float


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    yaw_deg: float
    gimbal_pitch_deg: float
    hold_sec: float
    tag: str


def marker_id_from_name(name: str, fallback: int) -> int:
    match = re.search(r'(\d+)', name)
    return int(match.group(1)) if match else fallback


def read_markers(path: Path) -> List[Marker]:
    markers: List[Marker] = []
    with path.open(newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for index, row in enumerate(reader):
            name = row.get('name') or row.get('tag') or f'marker_{index}'
            x = float(row.get('x', 'nan'))
            y = float(row.get('y', 'nan'))
            z = float(row.get('z', 'nan'))
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            markers.append(Marker(marker_id_from_name(name, index), x, y, z))
    markers.sort(key=lambda marker: (marker.x, marker.y, marker.marker_id))
    return markers


def yaw_to_target_deg(x: float, y: float, target_x: float, target_y: float) -> float:
    return math.degrees(math.atan2(target_y - y, target_x - x))


def append_transition(
    waypoints: List[Waypoint],
    prev_x: float,
    prev_y: float,
    target_x: float,
    target_y: float,
    cruise_altitude: float,
    max_transition_step: float,
    sequence: int,
) -> int:
    distance = math.hypot(target_x - prev_x, target_y - prev_y)
    if distance <= max_transition_step:
        return sequence
    count = max(1, math.ceil(distance / max_transition_step) - 1)
    for index in range(1, count + 1):
        ratio = index / (count + 1)
        x = prev_x + (target_x - prev_x) * ratio
        y = prev_y + (target_y - prev_y) * ratio
        next_ratio = min(1.0, (index + 1) / (count + 1))
        next_x = prev_x + (target_x - prev_x) * next_ratio
        next_y = prev_y + (target_y - prev_y) * next_ratio
        waypoints.append(Waypoint(
            x=x,
            y=y,
            z=cruise_altitude,
            yaw_deg=yaw_to_target_deg(x, y, next_x, next_y),
            gimbal_pitch_deg=-60.0,
            hold_sec=1.0,
            tag=f'mission2_cruise_transition_{sequence:03d}',
        ))
        sequence += 1
    return sequence


def marker_scan_waypoints(marker: Marker, cruise_altitude: float, wall_distance: float) -> Iterable[Waypoint]:
    roof_z = max(marker.z + 12.0, 18.0)
    wall_z = max(marker.z + 8.0, 14.0)
    yield Waypoint(
        x=marker.x,
        y=marker.y,
        z=max(roof_z, 18.0),
        yaw_deg=0.0,
        gimbal_pitch_deg=-90.0,
        hold_sec=4.0,
        tag=f'marker_{marker.marker_id}_roof_scan',
    )
    wall_specs = (
        ('N', marker.x, marker.y + wall_distance, -90.0),
        ('E', marker.x + wall_distance, marker.y, 180.0),
        ('S', marker.x, marker.y - wall_distance, 90.0),
        ('W', marker.x - wall_distance, marker.y, 0.0),
    )
    for direction, x, y, yaw in wall_specs:
        yield Waypoint(
            x=x,
            y=y,
            z=max(wall_z, 14.0),
            yaw_deg=yaw,
            gimbal_pitch_deg=-35.0,
            hold_sec=4.0,
            tag=f'marker_{marker.marker_id}_wall_scan_{direction}',
        )


def build_path(
    markers: List[Marker],
    cruise_altitude: float,
    max_transition_step: float,
    max_scan_waypoints: int,
    wall_distance: float,
) -> List[Waypoint]:
    waypoints: List[Waypoint] = []
    prev_x = markers[0].x
    prev_y = markers[0].y
    transition_sequence = 1
    scan_count = 0

    for marker in markers:
        transition_sequence = append_transition(
            waypoints,
            prev_x,
            prev_y,
            marker.x,
            marker.y,
            cruise_altitude,
            max_transition_step,
            transition_sequence,
        )
        for waypoint in marker_scan_waypoints(marker, cruise_altitude, wall_distance):
            if scan_count >= max_scan_waypoints:
                return waypoints
            if waypoint.z < cruise_altitude and waypoints:
                waypoints.append(Waypoint(
                    x=waypoint.x,
                    y=waypoint.y,
                    z=cruise_altitude,
                    yaw_deg=waypoint.yaw_deg,
                    gimbal_pitch_deg=-60.0,
                    hold_sec=1.0,
                    tag=f'mission2_cruise_transition_{transition_sequence:03d}',
                ))
                transition_sequence += 1
            waypoints.append(waypoint)
            scan_count += 1
            prev_x = waypoint.x
            prev_y = waypoint.y
    return waypoints


def write_path(path: Path, waypoints: List[Waypoint]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x', 'y', 'z', 'yaw_deg', 'gimbal_pitch_deg', 'hold_sec', 'tag'])
        for waypoint in waypoints:
            writer.writerow([
                f'{waypoint.x:.6f}',
                f'{waypoint.y:.6f}',
                f'{waypoint.z:.6f}',
                f'{waypoint.yaw_deg:.2f}',
                f'{waypoint.gimbal_pitch_deg:.2f}',
                f'{waypoint.hold_sec:.2f}',
                waypoint.tag,
            ])


def main():
    parser = argparse.ArgumentParser(description='Generate Mission2 UAV safe marker scan path.')
    parser.add_argument('--markers', type=Path, default=DEFAULT_MARKERS)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--cruise-altitude', type=float, default=24.0)
    parser.add_argument('--max-transition-step', type=float, default=12.0)
    parser.add_argument('--max-scan-waypoints', type=int, default=20)
    parser.add_argument('--wall-distance', type=float, default=10.0)
    args = parser.parse_args()

    markers = read_markers(args.markers)
    if not markers:
        raise SystemExit(f'No markers loaded from {args.markers}')
    waypoints = build_path(
        markers=markers,
        cruise_altitude=max(24.0, args.cruise_altitude),
        max_transition_step=max(1.0, args.max_transition_step),
        max_scan_waypoints=max(1, args.max_scan_waypoints),
        wall_distance=max(3.0, args.wall_distance),
    )
    write_path(args.output, waypoints)
    print(f'wrote {len(waypoints)} waypoints to {args.output}')


if __name__ == '__main__':
    main()
