import csv
import re
from datetime import datetime
from pathlib import Path

import rclpy
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class MissionTimerCsvLogger(Node):
    def __init__(self):
        super().__init__("asp_final_mission_timer_csv_logger")
        self.declare_parameters(
            "",
            [
                ("csv_path", str(Path.home() / "asp_ws" / "mission_logs" / "mission_timer_runs.csv")),
                ("source_node_name", "/mission_timer_node"),
            ],
        )
        self.csv_path = Path(str(self.get_parameter("csv_path").value))
        self.source_node_name = str(self.get_parameter("source_node_name").value)
        self.last_recorded_message = None
        self.finished_re = re.compile(r"Mission finished \(dist=([0-9.]+) m\)\. Total time: ([0-9.]+) s")

        self.create_subscription(Log, "/rosout", self.on_rosout, 100)
        self.get_logger().info(f"mission timer CSV logger ready; writing to {self.csv_path}")

    def ensure_csv_header(self):
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        if self.csv_path.exists():
            return
        with self.csv_path.open("w", newline="") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    "recorded_at",
                    "elapsed_sec",
                    "finish_distance_m",
                    "source_node",
                    "raw_message",
                ],
            )
            writer.writeheader()

    def on_rosout(self, msg):
        if msg.name != self.source_node_name:
            return
        match = self.finished_re.search(msg.msg)
        if not match:
            return
        if msg.msg == self.last_recorded_message:
            return

        self.last_recorded_message = msg.msg
        finish_distance_m = float(match.group(1))
        elapsed_sec = float(match.group(2))
        recorded_at = datetime.now().isoformat(timespec="seconds")

        self.ensure_csv_header()
        with self.csv_path.open("a", newline="") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    "recorded_at",
                    "elapsed_sec",
                    "finish_distance_m",
                    "source_node",
                    "raw_message",
                ],
            )
            writer.writerow(
                {
                    "recorded_at": recorded_at,
                    "elapsed_sec": f"{elapsed_sec:.2f}",
                    "finish_distance_m": f"{finish_distance_m:.2f}",
                    "source_node": msg.name,
                    "raw_message": msg.msg,
                }
            )
        self.get_logger().info(
            f"saved mission timer result: elapsed={elapsed_sec:.2f}s dist={finish_distance_m:.2f}m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionTimerCsvLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
