import csv
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
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
    detection_count: int = 1


class DetectedMarkerCsv(Node):
    def __init__(self):
        super().__init__("asp_final_detected_marker_csv")
        self.declare_parameters(
            "",
            [
                ("marker_detections_topic", "/asp_final/perception/uav/marker_detections"),
                ("csv_path", "/home/subin/ros2_ws/mission_logs/asp_final_detected_markers.csv"),
                ("allowed_marker_ids", "0,1,2,3,4,5,6,7,8,9"),
            ],
        )
        self.csv_path = Path(self.get_parameter("csv_path").value)
        self.allowed_marker_ids = parse_allowed_marker_ids(self.get_parameter("allowed_marker_ids").value)
        self.stored_markers = {}
        self.create_subscription(
            Detection3DArray,
            self.get_parameter("marker_detections_topic").value,
            self.on_detections,
            10,
        )
        allowed = sorted(self.allowed_marker_ids) if self.allowed_marker_ids else "all"
        self.get_logger().info(f"detected marker CSV will be saved to {self.csv_path}; allowed_marker_ids={allowed}")

    def on_detections(self, msg):
        stamp = self.get_clock().now()
        seen_time_sec = stamp.nanoseconds * 1e-9
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
            if marker_key not in self.stored_markers:
                self.stored_markers[marker_key] = StoredMarker(
                    pose=result.pose.pose,
                    first_seen_time_sec=seen_time_sec,
                    last_seen_time_sec=seen_time_sec,
                )
                self.get_logger().info(f"first detected UAV marker {marker_key} at {seen_time_sec:.3f}s")
                continue
            stored_marker = self.stored_markers[marker_key]
            stored_marker.pose = result.pose.pose
            stored_marker.last_seen_time_sec = seen_time_sec
            stored_marker.detection_count += 1

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
                ],
            )
            writer.writeheader()
            for marker_id in sorted(self.stored_markers.keys(), key=int):
                stored_marker = self.stored_markers[marker_id]
                pose = stored_marker.pose
                writer.writerow(
                    {
                        "marker_id": marker_id,
                        "first_seen_time_sec": f"{stored_marker.first_seen_time_sec:.6f}",
                        "last_seen_time_sec": f"{stored_marker.last_seen_time_sec:.6f}",
                        "detection_count": stored_marker.detection_count,
                        "pos_x": pose.position.x,
                        "pos_y": pose.position.y,
                        "pos_z": pose.position.z,
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
