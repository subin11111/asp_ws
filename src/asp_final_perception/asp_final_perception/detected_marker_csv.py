import csv
import math
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray


def parse_allowed_marker_ids(value):
    if value is None:
        return set()
    if isinstance(value, str):
        tokens = [token.strip() for token in value.split(",")]
    else:
        tokens = [str(token).strip() for token in value]
    allowed = set()
    for token in tokens:
        if not token:
            continue
        try:
            allowed.add(int(token))
        except ValueError:
            continue
    return allowed


@dataclass
class StoredMarker:
    pose: object
    first_seen_time_sec: float
    last_seen_time_sec: float
    best_seen_time_sec: float
    best_wp_distance_m: float
    detection_count: int = 1


class DetectedMarkerCsv(Node):
    def __init__(self):
        super().__init__("asp_final_detected_marker_csv")
        self.declare_parameters(
            "",
            [
                ("marker_detections_topic", "/asp_final/perception/uav/marker_detections"),
                ("csv_path", "/home/desktop1/ros2_ws/mission_logs/asp_final_detected_markers.csv"),
                ("allowed_marker_ids", "0,1,2,3,4,5,6,7,8,9"),
                ("waypoint_path", "mission2_uav_waypoints.csv"),
                ("map_frame", "map"),
                ("uav_base_frame", "x500_gimbal_0/base_link"),
            ],
        )
        self.csv_path = Path(self.get_parameter("csv_path").value)
        self.allowed_marker_ids = parse_allowed_marker_ids(self.get_parameter("allowed_marker_ids").value)
        self.map_frame = self.get_parameter("map_frame").value
        self.uav_base_frame = self.get_parameter("uav_base_frame").value
        self.marker_waypoints = self.load_marker_waypoints(self.get_parameter("waypoint_path").value)
        self.stored_markers = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Detection3DArray,
            self.get_parameter("marker_detections_topic").value,
            self.on_detections,
            10,
        )
        allowed = sorted(self.allowed_marker_ids) if self.allowed_marker_ids else "all"
        self.get_logger().info(
            f"detected marker CSV will be saved to {self.csv_path}; "
            f"allowed_marker_ids={allowed}; waypoint_markers={sorted(self.marker_waypoints.keys())}"
        )

    def resolve_waypoint_path(self, waypoint_path):
        path = Path(waypoint_path)
        if path.is_absolute():
            return path
        return Path(get_package_share_directory("asp_final_uav")) / "path" / path

    def load_marker_waypoints(self, waypoint_path):
        path = self.resolve_waypoint_path(waypoint_path)
        marker_waypoints = {}
        try:
            with path.open(newline="") as handle:
                reader = csv.reader(handle)
                for row in reader:
                    if not row or row[0].strip().startswith("#") or len(row) < 8:
                        continue
                    tag = row[7].strip()
                    marker_id = self.marker_id_from_tag(tag)
                    if marker_id is None:
                        continue
                    marker_waypoints[marker_id] = {
                        "x": float(row[0]),
                        "y": float(row[1]),
                        "z": float(row[2]),
                        "tag": tag,
                    }
        except (OSError, ValueError) as exc:
            self.get_logger().warn(f"failed to load marker waypoint map from {path}: {exc}")
        return marker_waypoints

    def marker_id_from_tag(self, tag):
        parts = tag.split("_")
        if len(parts) < 2 or parts[0] != "marker":
            return None
        try:
            return int(parts[1])
        except ValueError:
            return None

    def current_uav_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.uav_base_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(
                f"waiting for UAV TF {self.map_frame}->{self.uav_base_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        translation = transform.transform.translation
        return translation.x, translation.y, translation.z

    def distance_to_marker_waypoint(self, marker_id, uav_position):
        waypoint = self.marker_waypoints.get(marker_id)
        if waypoint is None or uav_position is None:
            return math.inf
        return math.sqrt(
            (uav_position[0] - waypoint["x"]) ** 2
            + (uav_position[1] - waypoint["y"]) ** 2
            + (uav_position[2] - waypoint["z"]) ** 2
        )

    def on_detections(self, msg):
        stamp = self.get_clock().now()
        seen_time_sec = stamp.nanoseconds * 1e-9
        uav_position = self.current_uav_position()
        for detection in msg.detections:
            if not detection.results:
                continue
            result = detection.results[0]
            marker_id = result.hypothesis.class_id
            try:
                parsed_id = int(marker_id)
            except (TypeError, ValueError):
                self.get_logger().warn(f"ignoring non-integer marker id: {marker_id}", throttle_duration_sec=2.0)
                continue
            if self.allowed_marker_ids and parsed_id not in self.allowed_marker_ids:
                continue
            marker_key = str(marker_id)
            wp_distance = self.distance_to_marker_waypoint(parsed_id, uav_position)
            if marker_key not in self.stored_markers:
                self.stored_markers[marker_key] = StoredMarker(
                    pose=deepcopy(result.pose.pose),
                    first_seen_time_sec=seen_time_sec,
                    last_seen_time_sec=seen_time_sec,
                    best_seen_time_sec=seen_time_sec,
                    best_wp_distance_m=wp_distance,
                )
                self.get_logger().info(f"first detected UAV marker {marker_key} at {seen_time_sec:.3f}s")
                continue
            stored_marker = self.stored_markers[marker_key]
            stored_marker.last_seen_time_sec = seen_time_sec
            stored_marker.detection_count += 1
            if wp_distance < stored_marker.best_wp_distance_m:
                stored_marker.pose = deepcopy(result.pose.pose)
                stored_marker.best_seen_time_sec = seen_time_sec
                stored_marker.best_wp_distance_m = wp_distance

    def save_to_csv(self):
        if not self.stored_markers:
            self.get_logger().info("no detected UAV markers to save")
            return
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        with self.csv_path.open("w", newline="") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    "marker_id",
                    "first_seen_time_sec",
                    "last_seen_time_sec",
                    "detection_count",
                    "pos_x",
                    "pos_y",
                    "pos_z",
                    "best_seen_time_sec",
                    "best_wp_distance_m",
                    "waypoint_tag",
                    "waypoint_x",
                    "waypoint_y",
                    "waypoint_z",
                ],
            )
            writer.writeheader()
            for marker_id in sorted(self.stored_markers.keys(), key=int):
                stored_marker = self.stored_markers[marker_id]
                pose = stored_marker.pose
                waypoint = self.marker_waypoints.get(int(marker_id), {})
                writer.writerow(
                    {
                        "marker_id": marker_id,
                        "first_seen_time_sec": f"{stored_marker.first_seen_time_sec:.6f}",
                        "last_seen_time_sec": f"{stored_marker.last_seen_time_sec:.6f}",
                        "detection_count": stored_marker.detection_count,
                        "pos_x": pose.position.x,
                        "pos_y": pose.position.y,
                        "pos_z": pose.position.z,
                        "best_seen_time_sec": f"{stored_marker.best_seen_time_sec:.6f}",
                        "best_wp_distance_m": (
                            "" if math.isinf(stored_marker.best_wp_distance_m)
                            else f"{stored_marker.best_wp_distance_m:.6f}"
                        ),
                        "waypoint_tag": waypoint.get("tag", ""),
                        "waypoint_x": waypoint.get("x", ""),
                        "waypoint_y": waypoint.get("y", ""),
                        "waypoint_z": waypoint.get("z", ""),
                    }
                )
        self.get_logger().info(f"saved {len(self.stored_markers)} detected UAV markers to {self.csv_path}")


def main(args=None):
    rclpy.init(args=args)
    node = DetectedMarkerCsv()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_to_csv()
        node.destroy_node()
        rclpy.shutdown()
