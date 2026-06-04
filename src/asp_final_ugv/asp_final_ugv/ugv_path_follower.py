import csv
import math
from dataclasses import dataclass
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Waypoint:
    x: float
    y: float
    target_speed: float | None = None


class UgvPathFollower(Node):
    def __init__(self):
        super().__init__("asp_final_ugv_path_follower")
        self.declare_parameters(
            "",
            [
                ("mission1_path", "mission1_carrier.csv"),
                ("rendezvous_path", "mission3_rendezvous.csv"),
                ("map_frame", "map"),
                ("base_frame", "X1_asp/base_link"),
                ("cmd_vel_topic", "/asp_final/ugv/cmd_vel"),
                ("control_period_s", 0.1),
                ("final_tolerance_m", 1.2),
                ("angular_kp", 2.8),
                ("curvature_gain", 3.0),
                ("heading_slowdown_gain", 1.4),
                ("mission1_carrier_mode", True),
                ("mission1_cruise_speed", 0.35),
                ("mission1_max_linear_speed", 0.45),
                ("mission1_min_linear_speed", 0.10),
                ("mission1_max_angular_speed", 0.45),
                ("mission1_max_linear_accel", 0.20),
                ("mission1_max_angular_accel", 0.35),
                ("mission1_lookahead_distance", 2.0),
                ("mission1_waypoint_tolerance", 1.0),
                ("mission1_slow_down_distance", 3.0),
                ("mission1_heading_slowdown_enabled", True),
                ("mission1_heading_slowdown_cos_threshold", 0.65),
                ("mission1_corner_slowdown_enabled", True),
                ("mission1_corner_slowdown_speed", 0.18),
                ("mission3_cruise_speed", 1.0),
                ("mission3_max_linear_speed", 1.3),
                ("mission3_min_linear_speed", 0.25),
                ("mission3_max_angular_speed", 1.2),
                ("mission3_max_linear_accel", 0.60),
                ("mission3_max_angular_accel", 0.90),
                ("mission3_lookahead_distance", 3.5),
                ("mission3_waypoint_tolerance", 1.2),
                ("mission3_slow_down_distance", 2.0),
                ("zero_publish_after_stop_count", 5),
                ("disable_cmd_after_stop", True),
            ],
        )
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.paths = {
            "mission1": self.load_path(self.get_parameter("mission1_path").value),
            "rendezvous": self.load_path(self.get_parameter("rendezvous_path").value),
        }
        self.mode = "idle"
        self.active_mode = "IDLE"
        self.index = 0
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.last_cmd_time = self.now()
        self.completed = {"mission1": False, "rendezvous": False}

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.m1_done_pub = self.create_publisher(Bool, "/asp_final/ugv/mission1_complete", 10)
        self.rv_done_pub = self.create_publisher(Bool, "/asp_final/ugv/rendezvous_reached", 10)
        self.state_pub = self.create_publisher(String, "/asp_final/ugv/state", 10)
        self.event_pub = self.create_publisher(String, "/asp_final/ugv/event", 10)
        self.mission_event_pub = self.create_publisher(String, "/asp_final/ugv/mission_event", 10)
        self.create_subscription(Bool, "/asp_final/ugv/mission1_start", self.on_mission1_start, 10)
        self.create_subscription(Bool, "/asp_final/ugv/rendezvous_start", self.on_rendezvous_start, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(float(self.get_parameter("control_period_s").value), self.tick)
        self.get_logger().info("asp_final UGV follower ready")

    def package_path(self, file_name):
        path = Path(file_name)
        if path.is_absolute():
            return path
        return Path(get_package_share_directory("asp_final_ugv")) / "path" / file_name

    def load_path(self, file_name):
        path = self.package_path(file_name)
        points = []
        with path.open(newline="") as handle:
            for row in csv.reader(handle):
                if not row or row[0].strip().startswith("#"):
                    continue
                target_speed = float(row[3]) if len(row) > 3 and row[3].strip() else None
                points.append(Waypoint(float(row[0]), float(row[1]), target_speed))
        if len(points) < 1:
            raise RuntimeError(f"No waypoints in {path}")
        return points

    def publish_bool(self, pub, value=True):
        msg = Bool()
        msg.data = bool(value)
        pub.publish(msg)

    def publish_text(self, pub, text):
        msg = String()
        msg.data = text
        pub.publish(msg)

    def now(self):
        return self.get_clock().now()

    def start_mode(self, mode):
        if self.completed.get(mode):
            return
        self.mode = mode
        self.active_mode = "MISSION1_CARRIER" if mode == "mission1" else "MISSION3_RENDEZVOUS"
        self.index = 0
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.last_cmd_time = self.now()
        self.publish_text(self.event_pub, f"UGV_MODE:{self.active_mode}")
        self.publish_text(self.event_pub, f"{mode}_started")
        self.log_profile()

    def on_mission1_start(self, msg):
        if msg.data and self.mode == "idle":
            self.start_mode("mission1")

    def on_rendezvous_start(self, msg):
        if msg.data and self.mode in ("idle", "mission1_done"):
            self.start_mode("rendezvous")

    def current_pose(self):
        return self.lookup_pose(self.base_frame)

    def lookup_pose(self, frame):
        transform = self.tf_buffer.lookup_transform(self.map_frame, frame, rclpy.time.Time())
        pos = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        return pos.x, pos.y, pos.z, yaw

    def stop(self):
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.cmd_pub.publish(Twist())

    def finish(self):
        self.stop()
        if self.mode == "mission1":
            self.completed["mission1"] = True
            self.mode = "mission1_done"
            self.active_mode = "MISSION1_COMPLETE"
            self.publish_bool(self.m1_done_pub)
            self.publish_text(self.mission_event_pub, "mission1_complete")
        elif self.mode == "rendezvous":
            self.completed["rendezvous"] = True
            self.mode = "rendezvous_done"
            self.active_mode = "MISSION3_COMPLETE"
            self.publish_bool(self.rv_done_pub)
            self.publish_text(self.mission_event_pub, "rendezvous_reached")

    def curvature_at(self, path, index):
        if index <= 0 or index >= len(path) - 1:
            return 0.0
        ax, ay = path[index - 1].x, path[index - 1].y
        bx, by = path[index].x, path[index].y
        cx, cy = path[index + 1].x, path[index + 1].y
        d1 = math.hypot(bx - ax, by - ay)
        d2 = math.hypot(cx - bx, cy - by)
        d3 = math.hypot(cx - ax, cy - ay)
        if min(d1, d2, d3) < 1e-6:
            return 0.0
        area2 = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
        return 2.0 * area2 / (d1 * d2 * d3)

    def profile_prefix(self):
        return "mission1" if self.mode == "mission1" else "mission3"

    def profile(self):
        prefix = self.profile_prefix()
        return {
            "cruise_speed": float(self.get_parameter(f"{prefix}_cruise_speed").value),
            "max_linear_speed": float(self.get_parameter(f"{prefix}_max_linear_speed").value),
            "min_linear_speed": float(self.get_parameter(f"{prefix}_min_linear_speed").value),
            "max_angular_speed": float(self.get_parameter(f"{prefix}_max_angular_speed").value),
            "max_linear_accel": float(self.get_parameter(f"{prefix}_max_linear_accel").value),
            "max_angular_accel": float(self.get_parameter(f"{prefix}_max_angular_accel").value),
            "waypoint_tolerance": float(self.get_parameter(f"{prefix}_waypoint_tolerance").value),
            "slow_down_distance": float(self.get_parameter(f"{prefix}_slow_down_distance").value),
        }

    def log_profile(self):
        profile = self.profile()
        self.get_logger().info(
            f"{self.active_mode} speed profile: cruise={profile['cruise_speed']:.2f}, "
            f"max_v={profile['max_linear_speed']:.2f}, max_w={profile['max_angular_speed']:.2f}, "
            f"max_accel={profile['max_linear_accel']:.2f}, max_angular_accel={profile['max_angular_accel']:.2f}"
        )

    def limited_cmd(self, target_v, target_w):
        profile = self.profile()
        now = self.now()
        dt = max((now - self.last_cmd_time).nanoseconds * 1e-9, 1e-3)
        self.last_cmd_time = now

        target_v = clamp(target_v, -profile["max_linear_speed"], profile["max_linear_speed"])
        target_w = clamp(target_w, -profile["max_angular_speed"], profile["max_angular_speed"])
        dv = clamp(target_v - self.prev_v, -profile["max_linear_accel"] * dt, profile["max_linear_accel"] * dt)
        dw = clamp(target_w - self.prev_w, -profile["max_angular_accel"] * dt, profile["max_angular_accel"] * dt)
        self.prev_v += dv
        self.prev_w += dw

        cmd = Twist()
        cmd.linear.x = self.prev_v
        cmd.angular.z = self.prev_w
        return cmd

    def tick(self):
        self.publish_text(self.state_pub, self.active_mode)
        if self.mode not in ("mission1", "rendezvous"):
            return
        try:
            x, y, _z, yaw = self.current_pose()
        except TransformException as exc:
            self.get_logger().warn(f"Waiting for UGV TF {self.map_frame}->{self.base_frame}: {exc}", throttle_duration_sec=2.0)
            return

        path = self.paths[self.mode]
        profile = self.profile()
        final_tol = float(self.get_parameter("final_tolerance_m").value)
        wp_tol = profile["waypoint_tolerance"]
        while self.index < len(path):
            tx, ty = path[self.index].x, path[self.index].y
            tolerance = final_tol if self.index == len(path) - 1 else wp_tol
            if math.hypot(tx - x, ty - y) > tolerance:
                break
            self.index += 1
        if self.index >= len(path):
            self.finish()
            return

        target = path[self.index]
        tx, ty = target.x, target.y
        dist = math.hypot(tx - x, ty - y)
        heading = math.atan2(ty - y, tx - x)
        heading_error = angle_wrap(heading - yaw)
        curvature = self.curvature_at(path, self.index)
        target_speed = profile["cruise_speed"]
        if target.target_speed is not None:
            target_speed = min(target_speed, target.target_speed)
        target_speed /= 1.0 + float(self.get_parameter("curvature_gain").value) * curvature
        target_speed /= 1.0 + float(self.get_parameter("heading_slowdown_gain").value) * abs(heading_error)
        if dist < profile["slow_down_distance"]:
            target_speed *= clamp(dist / profile["slow_down_distance"], 0.25, 1.0)
        target_w = float(self.get_parameter("angular_kp").value) * heading_error

        if self.mode == "mission1":
            if bool(self.get_parameter("mission1_heading_slowdown_enabled").value):
                heading_factor = max(0.0, math.cos(abs(heading_error)))
                threshold = float(self.get_parameter("mission1_heading_slowdown_cos_threshold").value)
                if heading_factor < threshold:
                    target_speed = min(target_speed, float(self.get_parameter("mission1_corner_slowdown_speed").value))
            if bool(self.get_parameter("mission1_corner_slowdown_enabled").value):
                angular_ratio = min(1.0, abs(target_w) / max(profile["max_angular_speed"], 1e-3))
                if angular_ratio > 0.65:
                    target_speed = min(target_speed, float(self.get_parameter("mission1_corner_slowdown_speed").value))

        target_speed = clamp(target_speed, profile["min_linear_speed"], profile["max_linear_speed"])
        self.cmd_pub.publish(self.limited_cmd(target_speed, target_w))


def main(args=None):
    rclpy.init(args=args)
    node = UgvPathFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
