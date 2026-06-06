import csv
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
            self.stored_markers[str(marker_id)] = result.pose.pose

    def save_to_csv(self):
        if not self.stored_markers:
            self.get_logger().info("no detected UAV markers to save")
            return
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        with self.csv_path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=["marker_id", "pos_x", "pos_y", "pos_z"])
            writer.writeheader()
            for marker_id in sorted(self.stored_markers.keys(), key=int):
                pose = self.stored_markers[marker_id]
                writer.writerow(
                    {
                        "marker_id": marker_id,
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
