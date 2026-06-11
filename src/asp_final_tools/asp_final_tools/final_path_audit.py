import csv
import math
from pathlib import Path


WORKSPACE = Path("/home/desktop1/ros2_ws")
MISSION2_ORIGIN_ESTIMATE = {"x": -131.5354213347476, "y": 61.829714929681145, "z": 0.4}
UAV_CONFIG = WORKSPACE / "src/asp_final_uav/config/uav_params.yaml"
DEFAULT_PATHS = [
    WORKSPACE / "src/asp_final_ugv/path/mission1_carrier.csv",
    WORKSPACE / "src/asp_final_uav/path/mission2_uav_waypoints.csv",
    WORKSPACE / "src/asp_final_ugv/path/mission3_rendezvous.csv",
]


def load_ugv_points(path):
    points = []
    with Path(path).open(newline="") as handle:
        for row in csv.reader(handle):
            if not row or row[0].strip().startswith("#"):
                continue
            speed = float(row[3]) if len(row) > 3 and row[3].strip() else None
            points.append({"x": float(row[0]), "y": float(row[1]), "speed": speed})
    return points


def load_uav_waypoints(path):
    points = []
    with Path(path).open(newline="") as handle:
        for row in csv.reader(handle):
            if not row or row[0].strip().startswith("#"):
                continue
            while len(row) < 6:
                row.append("0")
            points.append(
                {
                    "x": float(row[0]),
                    "y": float(row[1]),
                    "z": float(row[2]),
                    "yaw": float(row[3]),
                    "gimbal_pitch_deg": float(row[4]),
                    "marker_budget": int(float(row[5])),
                }
            )
    return points


def parse_scalar(value):
    value = value.strip().strip('"').strip("'")
    if value.lower() in ("true", "false"):
        return value.lower() == "true"
    try:
        if "." in value or "e" in value.lower():
            return float(value)
        return int(value)
    except ValueError:
        return value


def load_simple_params(path):
    params = {}
    if not Path(path).exists():
        return params
    for raw_line in Path(path).read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()
        if not value or key in ("ros__parameters", "asp_final_uav_mission_node"):
            continue
        params[key] = parse_scalar(value)
    return params


def heading(a, b):
    return math.atan2(b["y"] - a["y"], b["x"] - a["x"])


def angle_delta(a, b):
    return abs(math.atan2(math.sin(b - a), math.cos(b - a)))


def audit_ugv(path):
    points = load_ugv_points(path)
    print(f"=== {path} ===")
    print(f"points={len(points)}")
    warnings = 0
    for index, (a, b) in enumerate(zip(points, points[1:])):
        distance = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        speed = b["speed"]
        turn = 0.0
        if index + 2 < len(points):
            turn = math.degrees(angle_delta(heading(a, b), heading(b, points[index + 2])))
        print(
            f"segment={index:02d} distance_m={distance:.2f} "
            f"next_heading_change_deg={turn:.1f} target_speed={speed if speed is not None else 'profile'}"
        )
        if "mission1" in str(path) and speed is not None and speed > 0.45:
            print("WARNING carrier mode target speed exceeds 0.45 m/s")
            warnings += 1
        if "mission1" in str(path) and turn > 35.0 and (speed is None or speed > 0.25):
            print("WARNING sharp Mission1 turn should use target_speed <= 0.25 m/s")
            warnings += 1
    print(f"warnings={warnings}")
    return warnings


def audit_uav(path):
    points = load_uav_waypoints(path)
    params = load_simple_params(UAV_CONFIG)
    print(f"=== {path} ===")
    print(f"points={len(points)}")
    warnings = 0
    if len(points) < 2:
        print("WARNING UAV path should contain at least two waypoints")
        warnings += 1
    path_name = Path(path).name.lower()
    generated_runtime = "generated" in path_name or "safe" in path_name
    print(f"Generated/safe runtime path selected: {'YES' if generated_runtime else 'NO'}")
    if generated_runtime:
        print("ERROR generated/safe path must not be used as the asp_final Mission2 runtime path")
        warnings += 1
    if points:
        first = points[0]
        dx = first["x"] - MISSION2_ORIGIN_ESTIMATE["x"]
        dy = first["y"] - MISSION2_ORIGIN_ESTIMATE["y"]
        dz = first["z"] - MISSION2_ORIGIN_ESTIMATE["z"]
        dxy = math.hypot(dx, dy)
        d3 = math.sqrt(dx * dx + dy * dy + dz * dz)
        transition_enabled = bool(params.get("enable_mission2_transition_corridor", False))
        max_step = float(params.get("transition_corridor_max_step_m", 0.0))
        arrival_mode = params.get("waypoint_arrival_mode", "3d")
        xy_tol = params.get("waypoint_xy_tolerance_m", "unset")
        z_tol = params.get("waypoint_z_tolerance_m", "unset")
        print(f"Mission2 first WP distance: dxy={dxy:.2f}m d3={d3:.2f}m")
        print(
            "Transition corridor: "
            f"{'enabled' if transition_enabled else 'disabled'}, max_step={max_step:.1f}m"
        )
        print(f"Arrival mode: {arrival_mode}, xy_tol={xy_tol}, z_tol={z_tol}")
        if dxy >= 20.0 and transition_enabled:
            print("Status: OK - long first WP is protected by runtime transition corridor")
        elif dxy >= 20.0:
            print("ERROR first Mission2 UAV waypoint is far and transition corridor is disabled")
            warnings += 1
        if arrival_mode != "xy_z_separate":
            print("WARNING waypoint arrival mode is not xy_z_separate")
            warnings += 1
    for index, point in enumerate(points):
        print(
            f"waypoint={index:02d} x={point['x']:.2f} y={point['y']:.2f} z={point['z']:.2f} "
            f"yaw={point['yaw']:.2f} gimbal_pitch_deg={point['gimbal_pitch_deg']:.1f} "
            f"marker_budget={point['marker_budget']}"
        )
        if not all(math.isfinite(point[key]) for key in ("x", "y", "z", "yaw", "gimbal_pitch_deg")):
            print("WARNING UAV waypoint contains non-finite numeric value")
            warnings += 1
        if point["z"] <= 0.0:
            print("WARNING UAV waypoint altitude should be positive in map ENU")
            warnings += 1
        if point["marker_budget"] < 0:
            print("WARNING UAV marker_budget should not be negative")
            warnings += 1
        if not -120.0 <= point["gimbal_pitch_deg"] <= 45.0:
            print("WARNING UAV gimbal pitch is outside expected range [-120, 45] deg")
            warnings += 1
    for index, (a, b) in enumerate(zip(points, points[1:])):
        horizontal = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        vertical = abs(b["z"] - a["z"])
        distance = math.sqrt(horizontal**2 + vertical**2)
        print(
            f"segment={index:02d} horizontal_m={horizontal:.2f} vertical_m={vertical:.2f} distance_m={distance:.2f}"
        )
        if distance > 80.0:
            print("WARNING UAV waypoint segment is very long")
            warnings += 1
    print(f"warnings={warnings}")
    return warnings


def audit(path):
    if "uav" in Path(path).name or "uav" in str(path):
        return audit_uav(path)
    return audit_ugv(path)


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="*", default=[str(path) for path in DEFAULT_PATHS])
    args = parser.parse_args()
    total = 0
    for path in args.paths:
        total += audit(path)
    return 1 if total else 0


if __name__ == "__main__":
    raise SystemExit(main())
