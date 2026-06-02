#!/usr/bin/env python3
"""Generate UAV observation waypoints from marker pose CSV rows."""

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional


DEFAULT_INPUT = Path('/home/subin/ros2_ws/tools/path_tools/marker_poses.csv')
DEFAULT_OUTPUT = Path('/home/subin/ros2_ws/src/asp_uav_control/path/uav_path_generated.csv')


@dataclass(frozen=True)
class MarkerPose:
    name: str
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    yaw_deg: float
    gimbal_pitch_deg: float
    hold_sec: float
    tag: str


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def normalize_deg(value: float) -> float:
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def yaw_to_marker_deg(wp_x: float, wp_y: float, marker: MarkerPose) -> float:
    return normalize_deg(math.degrees(math.atan2(marker.y - wp_y, marker.x - wp_x)))


def marker_yaw_deg(marker: MarkerPose) -> float:
    return normalize_deg(math.degrees(marker.yaw))


def read_markers(path: Path) -> List[MarkerPose]:
    markers: List[MarkerPose] = []
    with path.open(newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            markers.append(MarkerPose(
                name=row['name'],
                x=float(row['x']),
                y=float(row['y']),
                z=float(row['z']),
                yaw=float(row.get('yaw') or 0.0),
            ))
    return markers


def deduplicate_waypoints(waypoints: Iterable[Waypoint]) -> List[Waypoint]:
    seen = set()
    unique: List[Waypoint] = []
    for waypoint in waypoints:
        key = (round(waypoint.x, 2), round(waypoint.y, 2), round(waypoint.z, 2))
        if key in seen:
            continue
        seen.add(key)
        unique.append(waypoint)
    return unique


def generate_waypoints(
    markers: Iterable[MarkerPose],
    mode: str,
    safe_altitude_m: float,
    roof_view_altitude_offset_m: float,
    wall_view_distance_m: float,
    wall_view_height_offset_m: float,
    hold_sec: float,
    start: Optional[MarkerPose],
) -> List[Waypoint]:
    safe_hold_sec = hold_sec if hold_sec >= 1.0 else 3.0
    waypoints: List[Waypoint] = []

    if start is not None:
        first_z = max(5.0, start.z + 6.0)
        second_z = max(5.0, safe_altitude_m)
        waypoints.extend([
            Waypoint(start.x, start.y, first_z, 0.0, -45.0, safe_hold_sec, 'start_climb'),
            Waypoint(start.x, start.y, second_z, 0.0, -45.0, safe_hold_sec, 'start_safe_altitude'),
        ])

    for marker in markers:
        marker_name = marker.name.replace('/', '_').replace(' ', '_')
        if mode in ('roof', 'both'):
            roof_z = max(marker.z + roof_view_altitude_offset_m, safe_altitude_m, 5.0)
            waypoints.append(Waypoint(
                x=marker.x,
                y=marker.y,
                z=roof_z,
                yaw_deg=marker_yaw_deg(marker),
                gimbal_pitch_deg=-90.0,
                hold_sec=safe_hold_sec,
                tag=f'{marker_name}_roof',
            ))

        if mode in ('wall', 'both'):
            wall_z = max(marker.z + wall_view_height_offset_m, 8.0, 5.0)
            candidates = [
                ('E', marker.x + wall_view_distance_m, marker.y),
                ('W', marker.x - wall_view_distance_m, marker.y),
                ('N', marker.x, marker.y + wall_view_distance_m),
                ('S', marker.x, marker.y - wall_view_distance_m),
            ]
            for direction, wp_x, wp_y in candidates:
                waypoints.append(Waypoint(
                    x=wp_x,
                    y=wp_y,
                    z=wall_z,
                    yaw_deg=yaw_to_marker_deg(wp_x, wp_y, marker),
                    gimbal_pitch_deg=clamp(-35.0, -90.0, 20.0),
                    hold_sec=safe_hold_sec,
                    tag=f'{marker_name}_wall_{direction}',
                ))

    return deduplicate_waypoints(waypoints)


def write_waypoints(path: Path, waypoints: List[Waypoint]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow(['x', 'y', 'z', 'yaw_deg', 'gimbal_pitch_deg', 'hold_sec', 'tag'])
        for waypoint in waypoints:
            writer.writerow([
                f'{waypoint.x:.6f}',
                f'{waypoint.y:.6f}',
                f'{clamp(waypoint.z, 5.0, 200.0):.6f}',
                f'{normalize_deg(waypoint.yaw_deg):.2f}',
                f'{clamp(waypoint.gimbal_pitch_deg, -90.0, 20.0):.2f}',
                f'{waypoint.hold_sec:.2f}',
                waypoint.tag,
            ])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Generate UAV observation path from marker poses.')
    parser.add_argument('--input', type=Path, default=DEFAULT_INPUT)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--mode', choices=('roof', 'wall', 'both'), default='both')
    parser.add_argument('--safe-altitude-m', type=float, default=18.0)
    parser.add_argument('--roof-view-altitude-offset-m', type=float, default=12.0)
    parser.add_argument('--wall-view-distance-m', type=float, default=10.0)
    parser.add_argument('--wall-view-height-offset-m', type=float, default=2.0)
    parser.add_argument('--hold-sec', type=float, default=4.0)
    parser.add_argument('--start-x', type=float)
    parser.add_argument('--start-y', type=float)
    parser.add_argument('--start-z', type=float)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    markers = read_markers(args.input)
    start = None
    start_values = (args.start_x, args.start_y, args.start_z)
    if any(value is not None for value in start_values):
        if not all(value is not None for value in start_values):
            raise SystemExit('--start-x, --start-y, and --start-z must be provided together')
        start = MarkerPose('start', args.start_x, args.start_y, args.start_z, 0.0)

    waypoints = generate_waypoints(
        markers=markers,
        mode=args.mode,
        safe_altitude_m=args.safe_altitude_m,
        roof_view_altitude_offset_m=args.roof_view_altitude_offset_m,
        wall_view_distance_m=args.wall_view_distance_m,
        wall_view_height_offset_m=args.wall_view_height_offset_m,
        hold_sec=args.hold_sec,
        start=start,
    )
    write_waypoints(args.output, waypoints)
    print(f'Read {len(markers)} marker rows from {args.input}')
    print(f'Wrote {len(waypoints)} UAV waypoints to {args.output}')
    return 0 if waypoints else 1


if __name__ == '__main__':
    raise SystemExit(main())
