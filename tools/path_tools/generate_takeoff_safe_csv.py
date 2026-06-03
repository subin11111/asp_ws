#!/usr/bin/env python3
"""Create a takeoff-safe UAV CSV without overwriting the source path."""

import argparse
import csv
import math
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional


DEFAULT_INPUT = Path('/home/subin/ros2_ws/src/asp_uav_control/path/uav_path.csv')
DEFAULT_OUTPUT = Path('/home/subin/ros2_ws/src/asp_uav_control/path/uav_path_safe.csv')
SAFE_PREFIX_TAGS = ('takeoff_climb', 'safe_altitude', 'transition_')


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


def normalize_yaw(yaw_deg: float) -> float:
    while yaw_deg > 180.0:
        yaw_deg -= 360.0
    while yaw_deg < -180.0:
        yaw_deg += 360.0
    return yaw_deg


def yaw_to_target(current_x: float, current_y: float, target_x: float, target_y: float) -> float:
    return normalize_yaw(math.degrees(math.atan2(target_y - current_y, target_x - current_x)))


def read_waypoints(path: Path) -> List[Waypoint]:
    waypoints: List[Waypoint] = []
    with path.open(newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        required = {'x', 'y', 'z', 'yaw_deg', 'gimbal_pitch_deg', 'hold_sec', 'tag'}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise SystemExit(f'Missing CSV fields in {path}: {sorted(missing)}')
        for row in reader:
            tag = row.get('tag', '')
            if tag == 'takeoff_climb' or tag == 'safe_altitude' or tag.startswith('transition_'):
                continue
            waypoints.append(Waypoint(
                x=float(row['x']),
                y=float(row['y']),
                z=max(5.0, float(row['z'])),
                yaw_deg=normalize_yaw(float(row['yaw_deg'])),
                gimbal_pitch_deg=clamp(float(row['gimbal_pitch_deg']), -90.0, 20.0),
                hold_sec=max(1.0, float(row['hold_sec'])),
                tag=tag or f'wp_{len(waypoints)}',
            ))
    return waypoints


def infer_start(first: Waypoint, start_x: Optional[float], start_y: Optional[float], start_z: Optional[float]):
    if start_x is not None and start_y is not None and start_z is not None:
        return start_x, start_y, start_z
    if any(value is not None for value in (start_x, start_y, start_z)):
        raise SystemExit('--start-x, --start-y, and --start-z must be provided together')
    inferred_z = min(1.6, first.z) if first.z < 1.6 else 1.6
    print(
        'WARNING: start pose was not provided. '
        'Using first waypoint x/y and estimated z for a test-only safe CSV.')
    return first.x, first.y, inferred_z


def build_safe_prefix(
    first: Waypoint,
    start_x: float,
    start_y: float,
    start_z: float,
    climb_height: float,
    safe_altitude: float,
    transition_altitude: float,
    max_transition_step: float,
    hold_sec: float,
    gimbal_pitch: float,
) -> List[Waypoint]:
    safe_altitude = max(10.0, safe_altitude)
    transition_altitude = max(safe_altitude, transition_altitude)
    gimbal_pitch = clamp(gimbal_pitch, -90.0, 20.0)
    yaw_deg = yaw_to_target(start_x, start_y, first.x, first.y)
    prefix = [
        Waypoint(start_x, start_y, max(5.0, start_z + climb_height), yaw_deg, gimbal_pitch, hold_sec, 'takeoff_climb'),
        Waypoint(start_x, start_y, safe_altitude, yaw_deg, gimbal_pitch, hold_sec, 'safe_altitude'),
    ]

    distance = math.hypot(first.x - start_x, first.y - start_y)
    if max_transition_step > 0.0 and distance > max_transition_step:
        transition_count = max(0, math.ceil(distance / max_transition_step) - 1)
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
                z=transition_altitude,
                yaw_deg=yaw_to_target(x, y, next_x, next_y),
                gimbal_pitch_deg=gimbal_pitch,
                hold_sec=hold_sec,
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
                f'{max(5.0, waypoint.z):.6f}',
                f'{normalize_yaw(waypoint.yaw_deg):.2f}',
                f'{clamp(waypoint.gimbal_pitch_deg, -90.0, 20.0):.2f}',
                f'{max(1.0, waypoint.hold_sec):.2f}',
                waypoint.tag,
            ])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Generate a UAV path CSV with a safe takeoff prefix.')
    parser.add_argument('--input', type=Path, default=DEFAULT_INPUT)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--start-x', type=float)
    parser.add_argument('--start-y', type=float)
    parser.add_argument('--start-z', type=float)
    parser.add_argument('--safe-altitude', type=float, default=18.0)
    parser.add_argument('--climb-height', type=float, default=5.0)
    parser.add_argument('--transition-altitude', type=float, default=18.0)
    parser.add_argument('--max-transition-step', type=float, default=15.0)
    parser.add_argument('--hold-sec', type=float, default=2.0)
    parser.add_argument('--gimbal-pitch', type=float, default=-60.0)
    parser.add_argument('--apply', action='store_true')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    scan_waypoints = read_waypoints(args.input)
    if not scan_waypoints:
        raise SystemExit(f'No scan waypoints found in {args.input}')

    start_x, start_y, start_z = infer_start(
        scan_waypoints[0], args.start_x, args.start_y, args.start_z)
    prefix = build_safe_prefix(
        first=scan_waypoints[0],
        start_x=start_x,
        start_y=start_y,
        start_z=start_z,
        climb_height=args.climb_height,
        safe_altitude=args.safe_altitude,
        transition_altitude=args.transition_altitude,
        max_transition_step=args.max_transition_step,
        hold_sec=max(1.0, args.hold_sec),
        gimbal_pitch=args.gimbal_pitch,
    )
    output_waypoints = prefix + scan_waypoints
    write_waypoints(args.output, output_waypoints)
    print(f'Read {len(scan_waypoints)} scan waypoints from {args.input}')
    print(f'Wrote {len(output_waypoints)} safe waypoints to {args.output}')
    if args.apply:
        shutil.copyfile(args.output, args.input)
        print(f'Applied safe CSV to {args.input}')
    else:
        print('Did not overwrite the input CSV. Use --apply only after review.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
