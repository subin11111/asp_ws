#!/usr/bin/env python3
"""Extract marker-like SDF poses into a planning CSV."""

import argparse
import csv
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional


DEFAULT_WORLD = Path('/home/subin/PX4-Autopilot_ASP/Tools/simulation/gz/worlds/default.sdf')
DEFAULT_MODELS = Path('/home/subin/PX4-Autopilot_ASP/Tools/simulation/gz/models')
DEFAULT_OUTPUT = Path('/home/subin/ros2_ws/tools/path_tools/marker_poses.csv')
KEYWORDS = ('aruco', 'marker', 'target', 'tag')


@dataclass(frozen=True)
class MarkerPose:
    name: str
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    source_file: str


def strip_namespace(tag: str) -> str:
    return tag.split('}', 1)[-1] if '}' in tag else tag


def child_text(element: ET.Element, child_name: str) -> Optional[str]:
    for child in list(element):
        if strip_namespace(child.tag) == child_name:
            return (child.text or '').strip()
    return None


def is_marker_name(text: str) -> bool:
    lowered = text.lower()
    return any(keyword in lowered for keyword in KEYWORDS)


def parse_pose(text: Optional[str]) -> Optional[List[float]]:
    if not text:
        return None
    parts = text.split()
    if len(parts) < 3:
        return None
    try:
        values = [float(value) for value in parts[:6]]
    except ValueError:
        return None
    while len(values) < 6:
        values.append(0.0)
    return values


def marker_from_pose(name: str, pose_values: List[float], source_file: Path) -> MarkerPose:
    return MarkerPose(
        name=name,
        x=pose_values[0],
        y=pose_values[1],
        z=pose_values[2],
        roll=pose_values[3],
        pitch=pose_values[4],
        yaw=pose_values[5],
        source_file=str(source_file),
    )


def extract_from_xml(path: Path, world: Path, include_model_local_poses: bool) -> List[MarkerPose]:
    tree = ET.parse(path)
    root = tree.getroot()
    poses: List[MarkerPose] = []

    for element in root.iter():
        tag = strip_namespace(element.tag)
        if tag not in ('include', 'model'):
            continue
        if path != world and tag == 'model' and not include_model_local_poses:
            continue

        name = child_text(element, 'name') or element.attrib.get('name', '')
        uri = child_text(element, 'uri') or ''
        marker_key = name or uri
        if not is_marker_name(f'{name} {uri}'):
            continue

        pose_values = parse_pose(child_text(element, 'pose'))
        if pose_values is None:
            continue

        poses.append(marker_from_pose(marker_key, pose_values, path))

    return poses


def extract_with_regex(path: Path, world: Path, include_model_local_poses: bool) -> List[MarkerPose]:
    text = path.read_text(errors='ignore')
    poses: List[MarkerPose] = []
    block_re = re.compile(r'<(include|model)\b(?P<attrs>[^>]*)>(?P<body>.*?)</\1>', re.DOTALL)
    name_re = re.compile(r'<name>\s*([^<]+?)\s*</name>', re.DOTALL)
    uri_re = re.compile(r'<uri>\s*([^<]+?)\s*</uri>', re.DOTALL)
    pose_re = re.compile(r'<pose(?:\s+[^>]*)?>\s*([^<]+?)\s*</pose>', re.DOTALL)
    model_name_re = re.compile(r'name\s*=\s*["\']([^"\']+)["\']')

    for match in block_re.finditer(text):
        if path != world and match.group(1) == 'model' and not include_model_local_poses:
            continue
        attrs = match.group('attrs') or ''
        body = match.group('body') or ''
        name_match = name_re.search(body)
        uri_match = uri_re.search(body)
        model_name_match = model_name_re.search(attrs)
        name = (
            name_match.group(1).strip()
            if name_match
            else model_name_match.group(1).strip()
            if model_name_match
            else ''
        )
        uri = uri_match.group(1).strip() if uri_match else ''
        marker_key = name or uri
        if not is_marker_name(f'{name} {uri}'):
            continue

        pose_match = pose_re.search(body)
        pose_values = parse_pose(pose_match.group(1) if pose_match else None)
        if pose_values is None:
            continue
        poses.append(marker_from_pose(marker_key, pose_values, path))

    return poses


def sdf_files(world: Path, models: Path) -> Iterable[Path]:
    if world.exists():
        yield world
    if models.exists():
        for path in sorted(models.rglob('*')):
            if path.suffix.lower() in ('.sdf', '.xml'):
                yield path


def deduplicate(poses: Iterable[MarkerPose]) -> List[MarkerPose]:
    seen = set()
    unique: List[MarkerPose] = []
    for pose in poses:
        key = (
            pose.name,
            round(pose.x, 6),
            round(pose.y, 6),
            round(pose.z, 6),
            round(pose.roll, 6),
            round(pose.pitch, 6),
            round(pose.yaw, 6),
        )
        if key in seen:
            continue
        seen.add(key)
        unique.append(pose)
    return unique


def write_csv(path: Path, poses: List[MarkerPose]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow(['name', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'source_file'])
        for pose in poses:
            writer.writerow([
                pose.name,
                pose.x,
                pose.y,
                pose.z,
                pose.roll,
                pose.pitch,
                pose.yaw,
                pose.source_file,
            ])


def print_table(poses: List[MarkerPose]) -> None:
    if not poses:
        print('No marker-like poses found.')
        return

    headers = ['name', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'source_file']
    rows = [
        [
            pose.name,
            f'{pose.x:.3f}',
            f'{pose.y:.3f}',
            f'{pose.z:.3f}',
            f'{pose.roll:.3f}',
            f'{pose.pitch:.3f}',
            f'{pose.yaw:.3f}',
            Path(pose.source_file).name,
        ]
        for pose in poses
    ]
    widths = [len(header) for header in headers]
    for row in rows:
        widths = [max(width, len(cell)) for width, cell in zip(widths, row)]

    print(' | '.join(header.ljust(width) for header, width in zip(headers, widths)))
    print('-+-'.join('-' * width for width in widths))
    for row in rows:
        print(' | '.join(cell.ljust(width) for cell, width in zip(row, widths)))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Extract marker poses from Gazebo SDF/XML files.')
    parser.add_argument('--world', type=Path, default=DEFAULT_WORLD)
    parser.add_argument('--models', type=Path, default=DEFAULT_MODELS)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        '--include-model-local-poses',
        action='store_true',
        help='also include marker poses defined inside model assets under --models',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    found: List[MarkerPose] = []
    for path in sdf_files(args.world, args.models):
        try:
            found.extend(extract_from_xml(path, args.world, args.include_model_local_poses))
        except ET.ParseError:
            found.extend(extract_with_regex(path, args.world, args.include_model_local_poses))
        except OSError as exc:
            print(f'Warning: failed to read {path}: {exc}', file=sys.stderr)

    poses = deduplicate(found)
    write_csv(args.output, poses)
    print_table(poses)
    print(f'\nSaved {len(poses)} marker pose rows to {args.output}')
    return 0 if poses else 1


if __name__ == '__main__':
    raise SystemExit(main())
