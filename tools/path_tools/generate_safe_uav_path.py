#!/usr/bin/env python3
"""Add safe takeoff and transition waypoints before an existing UAV path."""

import argparse
import csv
import math
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional


DEFAULT_INPUT = Path('/home/subin/ros2_ws/src/asp_uav_control/path/uav_path_generated.csv')
DEFAULT_OUTPUT = Path('/home/subin/ros2_ws/src/asp_uav_control/path/uav_path_safe.csv')
DEFAULT_APPLY_TARGET = Path('/home/subin/ros2_ws/src/asp_uav_control/path/uav_path.csv')


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    yaw_deg: float
    gimbal_pitch_deg: float
    hold_sec: float
    tag: str


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_deg(value: float) -> float:
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def yaw_between_deg(x1: float, y1: float, x2: float, y2: float) -> float:
    return normalize_deg(math.degrees(math.atan2(y2 - y1, x2 - x1)))


def read_waypoints(path: Path) -> List[Waypoint]:
    waypoints: List[Waypoint] = []
    with path.open(newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            waypoints.append(Waypoint(
                x=float(row['x']),
                y=float(row['y']),
                z=clamp(float(row['z']), 5.0, 200.0),
                yaw_deg=normalize_deg(float(row['yaw_deg'])),
                gimbal_pitch_deg=clamp(float(row['gimbal_pitch_deg']), -90.0, 20.0),
                hold_sec=max(1.0, float(row.get('hold_sec') or 3.0)),
                tag=row.get('tag') or f'wp_{len(waypoints)}',
            ))
    return waypoints


def build_safe_prefix(
    original: List[Waypoint],
    start_x: Optional[float],
    start_y: Optional[float],
    start_z: Optional[float],
    safe_altitude: float,
    transition_altitude: float,
    max_initial_step: float,
) -> List[Waypoint]:
    if not original:
        return []

    first = original[0]
    if start_x is None or start_y is None or start_z is None:
        print(
            'Warning: --start-x/--start-y/--start-z were not provided. '
            'Using the first waypoint x/y as a temporary start reference.')
        start_x = first.x
        start_y = first.y
        start_z = 0.0

    takeoff_yaw = yaw_between_deg(start_x, start_y, first.x, first.y)
    prefix = [
        Waypoint(start_x, start_y, clamp(start_z + 5.0, 5.0, 200.0),
                 takeoff_yaw, -60.0, 2.0, 'takeoff_climb'),
        Waypoint(start_x, start_y, clamp(safe_altitude, 5.0, 200.0),
                 takeoff_yaw, -70.0, 2.0, 'safe_altitude'),
    ]

    distance = math.hypot(first.x - start_x, first.y - start_y)
    if max_initial_step <= 0.0 or distance <= max_initial_step:
        return prefix

    transition_count = max(0, math.ceil(distance / max_initial_step) - 1)
    for index in range(1, transition_count + 1):
        ratio = index / (transition_count + 1)
        x = start_x + (first.x - start_x) * ratio
        y = start_y + (first.y - start_y) * ratio
        next_ratio = min(1.0, (index + 1) / (transition_count + 1))
        next_x = start_x + (first.x - start_x) * next_ratio
        next_y = start_y + (first.y - start_y) * next_ratio
        prefix.append(Waypoint(
            x=x,
            y=y,
            z=clamp(transition_altitude, 5.0, 200.0),
            yaw_deg=yaw_between_deg(x, y, next_x, next_y),
            gimbal_pitch_deg=-70.0,
            hold_sec=3.0,
            tag=f'transition_{index:03d}',
        ))

    return prefix


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
                f'{max(1.0, waypoint.hold_sec):.2f}',
                waypoint.tag,
            ])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Generate a safer UAV path with takeoff and transition waypoints.')
    parser.add_argument('--input', type=Path, default=DEFAULT_INPUT)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--start-x', type=float)
    parser.add_argument('--start-y', type=float)
    parser.add_argument('--start-z', type=float)
    parser.add_argument('--safe-altitude', type=float, default=18.0)
    parser.add_argument('--transition-altitude', type=float, default=18.0)
    parser.add_argument('--max-initial-step', type=float, default=15.0)
    parser.add_argument('--apply', action='store_true')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    start_values = (args.start_x, args.start_y, args.start_z)
    if any(value is not None for value in start_values) and not all(value is not None for value in start_values):
        raise SystemExit('--start-x, --start-y, and --start-z must be provided together')

    original = read_waypoints(args.input)
    prefix = build_safe_prefix(
        original=original,
        start_x=args.start_x,
        start_y=args.start_y,
        start_z=args.start_z,
        safe_altitude=args.safe_altitude,
        transition_altitude=args.transition_altitude,
        max_initial_step=args.max_initial_step,
    )
    safe_path = prefix + original
    write_waypoints(args.output, safe_path)
    print(f'Read {len(original)} original waypoints from {args.input}')
    print(f'Wrote {len(safe_path)} safe-path waypoints to {args.output}')

    if args.apply:
        shutil.copyfile(args.output, DEFAULT_APPLY_TARGET)
        print(f'Applied safe path to {DEFAULT_APPLY_TARGET}')
    else:
        print('Did not overwrite uav_path.csv. Use --apply only after reviewing the safe path.')

    return 0 if safe_path else 1


if __name__ == '__main__':
    raise SystemExit(main())
