import csv
from dataclasses import dataclass
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    label: str


def color(r, g, b, a=1.0):
    msg = ColorRGBA()
    msg.r = float(r)
    msg.g = float(g)
    msg.b = float(b)
    msg.a = float(a)
    return msg


class FinalVisualization(Node):
    def __init__(self):
        super().__init__("asp_final_visualization")
        self.declare_parameters(
            "",
            [
                ("map_frame", "map"),
                ("publish_period_s", 1.0),
                ("ugv_mission1_path", "mission1_carrier.csv"),
                ("ugv_rendezvous_path", "mission3_rendezvous.csv"),
                ("uav_waypoint_path", "mission2_uav_waypoints.csv"),
            ],
        )
        self.map_frame = self.get_parameter("map_frame").value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ugv_pub = self.create_publisher(MarkerArray, "/asp_final/visualization/ugv_waypoints", qos)
        self.uav_pub = self.create_publisher(MarkerArray, "/asp_final/visualization/uav_waypoints", qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/asp_final/visualization/detected_aruco_markers", 10)

        self.ugv_waypoints = self.load_ugv_waypoints()
        self.uav_waypoints = self.load_uav_waypoints()
        self.detected_markers = {}

        self.create_subscription(
            Detection3DArray,
            "/asp_final/perception/uav/marker_detections",
            lambda msg: self.on_detections(msg, "uav"),
            10,
        )
        self.create_subscription(
            Detection3DArray,
            "/asp_final/perception/landing/marker_detections",
            lambda msg: self.on_detections(msg, "landing"),
            10,
        )
        period = float(self.get_parameter("publish_period_s").value)
        self.timer = self.create_timer(period, self.publish_static_markers)
        self.publish_static_markers()
        self.get_logger().info("asp_final RViz visualization markers ready")

    def package_path(self, package, subdir, file_name):
        path = Path(file_name)
        if path.is_absolute():
            return path
        return Path(get_package_share_directory(package)) / subdir / file_name

    def load_ugv_waypoints(self):
        waypoints = []
        specs = [
            ("mission1", self.get_parameter("ugv_mission1_path").value),
            ("rendezvous", self.get_parameter("ugv_rendezvous_path").value),
        ]
        for group, file_name in specs:
            path = self.package_path("asp_final_ugv", "path", file_name)
            with path.open(newline="") as handle:
                for index, row in enumerate(csv.reader(handle)):
                    if not row or row[0].strip().startswith("#"):
                        continue
                    waypoints.append(Waypoint(float(row[0]), float(row[1]), 0.15, f"{group}:{index}"))
        return waypoints

    def load_uav_waypoints(self):
        waypoints = []
        path = self.package_path("asp_final_uav", "path", self.get_parameter("uav_waypoint_path").value)
        with path.open(newline="") as handle:
            for index, row in enumerate(csv.reader(handle)):
                if not row or row[0].strip().startswith("#"):
                    continue
                tag = row[7].strip() if len(row) > 7 and row[7].strip() else f"wp_{index}"
                waypoints.append(Waypoint(float(row[0]), float(row[1]), float(row[2]), tag))
        return waypoints

    def marker_base(self, ns, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = int(marker_id)
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def line_marker(self, ns, marker_id, waypoints, marker_color, width):
        marker = self.marker_base(ns, marker_id, Marker.LINE_STRIP)
        marker.scale.x = width
        marker.color = marker_color
        for waypoint in waypoints:
            point = Point()
            point.x = waypoint.x
            point.y = waypoint.y
            point.z = waypoint.z
            marker.points.append(point)
        return marker

    def sphere_marker(self, ns, marker_id, waypoint, marker_color, scale):
        marker = self.marker_base(ns, marker_id, Marker.SPHERE)
        marker.pose.position.x = waypoint.x
        marker.pose.position.y = waypoint.y
        marker.pose.position.z = waypoint.z
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color = marker_color
        return marker

    def text_marker(self, ns, marker_id, waypoint, text, marker_color, scale=1.0):
        marker = self.marker_base(ns, marker_id, Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = waypoint.x
        marker.pose.position.y = waypoint.y
        marker.pose.position.z = waypoint.z + 1.0
        marker.scale.z = scale
        marker.color = marker_color
        marker.text = text
        return marker

    def waypoint_markers(self, waypoints, ns, line_color, point_color, text_color, start_id=0, point_scale=0.8):
        markers = MarkerArray()
        if waypoints:
            markers.markers.append(self.line_marker(ns, start_id, waypoints, line_color, 0.35))
        for index, waypoint in enumerate(waypoints):
            marker_id = start_id + 1 + index
            markers.markers.append(self.sphere_marker(ns, marker_id, waypoint, point_color, point_scale))
            if index == 0 or index == len(waypoints) - 1:
                markers.markers.append(
                    self.text_marker(ns, marker_id + 1000, waypoint, waypoint.label, text_color, scale=1.0)
                )
        return markers

    def publish_static_markers(self):
        ugv_m1 = [wp for wp in self.ugv_waypoints if wp.label.startswith("mission1:")]
        ugv_rv = [wp for wp in self.ugv_waypoints if wp.label.startswith("rendezvous:")]
        ugv = MarkerArray()
        ugv.markers.extend(
            self.waypoint_markers(ugv_m1, "ugv_mission1", color(0.0, 0.85, 1.0), color(0.0, 0.55, 1.0), color(0.65, 0.9, 1.0)).markers
        )
        ugv.markers.extend(
            self.waypoint_markers(ugv_rv, "ugv_rendezvous", color(1.0, 0.48, 0.0), color(1.0, 0.35, 0.0), color(1.0, 0.75, 0.35), 2000).markers
        )
        self.ugv_pub.publish(ugv)
        self.uav_pub.publish(
            self.waypoint_markers(
                self.uav_waypoints,
                "uav_mission2",
                color(0.62, 0.42, 1.0),
                color(0.35, 0.8, 1.0),
                color(0.8, 0.72, 1.0),
                point_scale=1.0,
            )
        )

    def on_detections(self, msg, source):
        changed = False
        for detection in msg.detections:
            if not detection.results:
                continue
            result = detection.results[0]
            marker_id = result.hypothesis.class_id
            key = f"{source}:{marker_id}"
            self.detected_markers[key] = result.pose.pose
            changed = True
        if changed:
            self.publish_detected_markers()

    def publish_detected_markers(self):
        markers = MarkerArray()
        for index, (key, pose) in enumerate(sorted(self.detected_markers.items())):
            marker_id = 4000 + index
            point = Waypoint(pose.position.x, pose.position.y, pose.position.z, key)
            markers.markers.append(self.sphere_marker("detected_aruco", marker_id, point, color(0.0, 1.0, 0.2), 1.25))
            markers.markers.append(
                self.text_marker("detected_aruco_labels", marker_id + 1000, point, key, color(0.45, 1.0, 0.55), scale=1.0)
            )
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = FinalVisualization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
