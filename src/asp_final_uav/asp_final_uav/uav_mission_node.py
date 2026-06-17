import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    yaw: float
    gimbal_pitch_deg: float
    marker_budget: int
    tag: str = ""
    hold_sec: float = 0.0


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return 0.0, 0.0, qz, qw


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class UavMissionNode(Node):
    def __init__(self):
        super().__init__("asp_final_uav_mission_node")
        self.declare_parameters(
            "",
            [
                ("waypoint_path", "mission2_uav_waypoints.csv"),
                ("map_frame", "map"),
                ("base_frame", "x500_gimbal_0/base_link"),
                ("ugv_base_frame", "X1_asp/base_link"),
                ("ugv_landing_frame", "X1_asp/aruco_marker_10_link"),
                ("rendezvous_path", "mission3_rendezvous.csv"),
                ("rendezvous_goal_x", float("nan")),
                ("rendezvous_goal_y", float("nan")),
                ("control_period_s", 0.2),
                ("takeoff_altitude_m", 24.0),
                ("waypoint_tolerance_m", 1.2),
                ("waypoint_timeout_s", 14.0),
                ("marker_action_timeout_s", 4.0),
                ("hold_after_takeoff_s", 0.0),
                ("takeoff_complete_timeout_s", 3.0),
                ("default_yaw_rad", 0.0),
                ("enable_mission2_transition_corridor", True),
                ("transition_corridor_max_step_m", 10.0),
                ("transition_corridor_min_cruise_altitude", 24.0),
                ("transition_corridor_max_cruise_altitude", 34.0),
                ("transition_corridor_hold_sec", 1.0),
                ("transition_corridor_gimbal_pitch_deg", -60.0),
                ("transition_corridor_tag_prefix", "mission2_transition"),
                ("waypoint_arrival_mode", "xy_z_separate"),
                ("waypoint_xy_tolerance_m", 2.5),
                ("waypoint_z_tolerance_m", 3.0),
                ("ignore_yaw_for_waypoint_reached", True),
                ("yaw_tolerance_deg", 45.0),
                ("waypoint_stuck_timeout_sec", 12.0),
                ("max_same_waypoint_sec", 15.0),
                ("force_advance_on_stuck", True),
                ("force_advance_when_xy_close", True),
                ("xy_close_timeout_sec", 5.0),
                ("continue_on_marker_timeout", True),
                ("marker_wait_timeout_sec", 3.0),
                ("do_not_block_waypoint_progress_on_marker", True),
                ("mandatory_waypoint_tags_csv", ""),
                ("proximity_confirm_waypoint_tags_csv", ""),
                ("proximity_confirm_distance_m", 6.0),
                ("landing_hover_altitude_m", 4.0),
                ("landing_descent_step_m", 1.0),
                ("landing_complete_altitude_m", 0.0),
                ("landing_timeout_s", 30.0),
                ("landing_detection_timeout_s", 2.0),
                ("landing_gimbal_pitch_deg", -90.0),
                ("landing_pre_approach_altitude_m", 2.0),
                ("landing_approach_altitude_m", 3.0),
                ("landing_align_hold_sec", 2.0),
                ("landing_touchdown_setpoint_m", 0.0),
                ("landing_xy_tolerance_m", 1.0),
                ("landing_yaw_tolerance_deg", 12.0),
                ("landing_use_marker_yaw", False),
                ("landing_marker_yaw_max_error_deg", 25.0),
                ("landing_marker_id", 10),
                ("landing_marker_max_ugv_distance_m", 2.0),
                ("landing_target_update_max_jump_m", 0.5),
                ("landing_freeze_target_after_align", True),
            ],
        )
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ugv_base_frame = self.get_parameter("ugv_base_frame").value
        self.ugv_landing_frame = self.get_parameter("ugv_landing_frame").value
        self.csv_waypoints = self.load_waypoints(self.get_parameter("waypoint_path").value)
        self.waypoints = list(self.csv_waypoints)
        self.phase = "idle"
        self.index = 0
        self.phase_started = self.now()
        self.wp_started = self.now()
        self.wp_hold_started = None
        self.xy_close_started = None
        self.last_wp_error_log = self.now()
        self.last_pose = None
        self.last_marker_id = None
        self.detected_marker_ids = set()
        self.last_landing_detection = None
        self.last_landing_detection_time = None
        self.last_ugv_landing_xy = None
        self.takeoff_origin = None
        self.landing_target_xy = None
        self.landing_target_yaw = None
        self.rendezvous_goal_xy = self.load_rendezvous_goal()
        self.landing_stage = "idle"
        self.landing_started = self.now()
        self.landing_land_started = None
        self.landing_align_started = None
        self.landing_complete_published = False
        self.mandatory_waypoint_tags = {
            tag.strip()
            for tag in str(self.get_parameter("mandatory_waypoint_tags_csv").value).split(",")
            if tag.strip()
        }
        self.proximity_confirm_waypoint_tags = {
            tag.strip()
            for tag in str(self.get_parameter("proximity_confirm_waypoint_tags_csv").value).split(",")
            if tag.strip()
        }

        self.cmd_pose_pub = self.create_publisher(PoseStamped, "/asp_final/uav/cmd_pose", 10)
        self.land_pub = self.create_publisher(Bool, "/asp_final/uav/land", 10)
        self.gimbal_pub = self.create_publisher(Float32, "/asp_final/uav/gimbal_pitch_deg", 10)
        self.m2_done_pub = self.create_publisher(Bool, "/asp_final/uav/mission2_complete", 10)
        self.exploration_state_pub = self.create_publisher(String, "/asp_final/uav/exploration_state", 10)
        self.exploration_event_pub = self.create_publisher(String, "/asp_final/uav/exploration_event", 10)
        origin_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.takeoff_origin_pub = self.create_publisher(PoseStamped, "/asp_final/uav/mission2_takeoff_origin", origin_qos)
        self.landing_state_pub = self.create_publisher(String, "/asp_final/landing/state", 10)
        self.landing_event_pub = self.create_publisher(String, "/asp_final/landing/event", 10)
        self.landing_complete_pub = self.create_publisher(Bool, "/asp_final/landing/complete", 10)

        self.create_subscription(Bool, "/asp_final/uav/mission2_start", self.on_mission2_start, 10)
        self.create_subscription(Bool, "/asp_final/landing/start", self.on_landing_start, 10)
        self.create_subscription(Int32, "/asp_final/perception/uav/marker_id", self.on_marker_id, 10)
        self.create_subscription(
            Detection3DArray,
            "/asp_final/perception/landing/marker_detections",
            self.on_landing_detection,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(float(self.get_parameter("control_period_s").value), self.tick)
        self.get_logger().info("asp_final UAV mission node ready")

    def load_waypoints(self, file_name):
        path = Path(file_name)
        if not path.is_absolute():
            path = Path(get_package_share_directory("asp_final_uav")) / "path" / file_name
        waypoints = []
        with path.open(newline="") as handle:
            for row in csv.reader(handle):
                if not row or row[0].strip().startswith("#"):
                    continue
                while len(row) < 6:
                    row.append("0")
                tag = row[7].strip() if len(row) > 7 and row[7].strip() else f"csv_wp_{len(waypoints):03d}"
                hold_sec = float(row[6]) if len(row) > 6 and row[6].strip() else 0.0
                waypoints.append(Waypoint(*(float(row[i]) for i in range(5)), int(float(row[5])), tag, hold_sec))
        if not waypoints:
            raise RuntimeError(f"No UAV waypoints in {path}")
        return waypoints

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

    def pose_msg(self, x, y, z, yaw=None):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        qx, qy, qz, qw = yaw_to_quaternion(float(self.get_parameter("default_yaw_rad").value) if yaw is None else yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def publish_gimbal(self, pitch):
        msg = Float32()
        msg.data = float(pitch)
        self.gimbal_pub.publish(msg)

    def load_rendezvous_goal(self):
        goal_x = float(self.get_parameter("rendezvous_goal_x").value)
        goal_y = float(self.get_parameter("rendezvous_goal_y").value)
        if math.isfinite(goal_x) and math.isfinite(goal_y):
            return goal_x, goal_y

        file_name = str(self.get_parameter("rendezvous_path").value)
        path = Path(file_name)
        if not path.is_absolute():
            path = Path(get_package_share_directory("asp_final_ugv")) / "path" / file_name

        last_row = None
        with path.open(newline="") as handle:
            for row in csv.reader(handle):
                if not row or row[0].strip().startswith("#"):
                    continue
                last_row = row
        if last_row is None:
            raise RuntimeError(f"No rendezvous waypoint in {path}")
        return float(last_row[0]), float(last_row[1])

    def current_pose(self):
        transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        pos = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        return pos.x, pos.y, pos.z, yaw

    def elapsed(self, stamp):
        return (self.now() - stamp).nanoseconds * 1e-9

    def on_marker_id(self, msg):
        self.last_marker_id = msg.data
        self.detected_marker_ids.add(int(msg.data))
        self.publish_text(self.exploration_event_pub, f"MARKER_DETECTED:{msg.data}")

    def on_landing_detection(self, msg):
        if self.phase != "landing":
            return
        detection = self.select_landing_detection(msg)
        if detection:
            self.last_landing_detection = detection
            self.last_landing_detection_time = self.now()
            self.get_logger().info(
                f"LANDING_DETECTION marker={detection['marker_id']} "
                f"map_x={detection['map_x']:.2f} map_y={detection['map_y']:.2f}",
                throttle_duration_sec=1.0,
            )

    def on_mission2_start(self, msg):
        if not msg.data or self.phase not in ("idle", "mission2_complete"):
            return
        self.phase = "takeoff"
        self.index = 0
        self.phase_started = self.now()
        self.wp_started = self.now()
        self.wp_hold_started = None
        self.xy_close_started = None
        self.last_wp_error_log = self.now()
        self.waypoints = list(self.csv_waypoints)
        self.takeoff_origin = None
        self.detected_marker_ids.clear()
        self.publish_text(self.exploration_event_pub, "mission2_started")

    def on_landing_start(self, msg):
        if not msg.data:
            return
        self.phase = "landing"
        self.landing_started = self.now()
        self.rendezvous_goal_xy = self.load_rendezvous_goal()
        self.landing_target_xy = self.rendezvous_goal_xy
        self.landing_target_yaw = None
        self.landing_stage = "pre_approach"
        self.last_landing_detection = None
        self.last_landing_detection_time = None
        self.last_ugv_landing_xy = None
        self.landing_land_started = None
        self.landing_align_started = None
        self.landing_complete_published = False
        self.publish_text(self.landing_event_pub, "landing_started")
        self.publish_text(
            self.landing_event_pub,
            f"landing_pre_approach_started:x={self.rendezvous_goal_xy[0]:.2f}:y={self.rendezvous_goal_xy[1]:.2f}",
        )

    def tick(self):
        self.publish_text(self.exploration_state_pub, self.phase)
        if self.phase == "idle":
            return
        try:
            x, y, z, yaw = self.current_pose()
            self.last_pose = (x, y, z, yaw)
        except TransformException as exc:
            self.get_logger().warn(f"Waiting for UAV TF {self.map_frame}->{self.base_frame}: {exc}", throttle_duration_sec=2.0)
            if self.last_pose is None:
                return
            x, y, z, yaw = self.last_pose

        if self.phase == "takeoff":
            if self.takeoff_origin is None:
                self.takeoff_origin = self.pose_msg(x, y, z, yaw)
                self.takeoff_origin_pub.publish(self.takeoff_origin)
                self.publish_text(self.exploration_event_pub, "TAKEOFF_CLIMB_STARTED")
            target_z = max(z, self.takeoff_origin.pose.position.z + float(self.get_parameter("takeoff_altitude_m").value))
            self.cmd_pose_pub.publish(self.pose_msg(x, y, target_z, yaw))
            if (
                abs(z - target_z) < float(self.get_parameter("waypoint_tolerance_m").value)
                or self.elapsed(self.phase_started) > float(self.get_parameter("takeoff_complete_timeout_s").value)
            ):
                if self.elapsed(self.phase_started) >= float(self.get_parameter("hold_after_takeoff_s").value):
                    self.publish_text(self.exploration_event_pub, "TAKEOFF_HOLD_COMPLETE")
                    self.waypoints = self.build_mission2_runtime_waypoints(x, y, z)
                    self.phase = "explore"
                    self.index = 0
                    self.start_current_waypoint()
                    self.publish_text(self.exploration_event_pub, "takeoff_complete_exploration_started")
            return

        if self.phase == "explore":
            if self.index >= len(self.waypoints):
                self.phase = "mission2_complete"
                self.publish_bool(self.m2_done_pub)
                self.publish_text(self.exploration_event_pub, "mission2_complete")
                return
            wp = self.waypoints[self.index]
            expected_marker_id = self.expected_marker_id(wp)
            if (
                not self.is_mandatory_waypoint(wp)
                and expected_marker_id is not None
                and expected_marker_id in self.detected_marker_ids
            ):
                self.advance_waypoint("marker_already_detected", wp, 0.0, 0.0)
                return
            self.cmd_pose_pub.publish(self.pose_msg(wp.x, wp.y, wp.z, wp.yaw))
            self.publish_gimbal(wp.gimbal_pitch_deg)
            dx, dy, dz, dxy, d3 = self.compute_waypoint_error((x, y, z), wp)
            self.log_waypoint_error(wp, dxy, dz, d3)
            if (
                not self.is_mandatory_waypoint(wp)
                and expected_marker_id is not None
                and expected_marker_id in self.detected_marker_ids
            ):
                self.advance_waypoint("marker_detected", wp, dxy, dz)
                return
            reached = self.waypoint_reached(wp, dxy, dz, d3)
            timed_out = self.elapsed(self.wp_started) > float(self.get_parameter("waypoint_timeout_s").value)
            stuck = (
                bool(self.get_parameter("force_advance_on_stuck").value)
                and self.elapsed(self.wp_started) > float(self.get_parameter("waypoint_stuck_timeout_sec").value)
            )
            max_same_elapsed = (
                float(self.get_parameter("max_same_waypoint_sec").value) > 0.0
                and self.elapsed(self.wp_started) > float(self.get_parameter("max_same_waypoint_sec").value)
            )
            xy_close_force = self.xy_close_force_advance(wp, dxy, dz)
            near_waypoint = dxy < max(float(self.get_parameter("waypoint_xy_tolerance_m").value) * 2.0, 2.5)
            marker_wait_done = (
                not self.is_mandatory_waypoint(wp)
                and wp.marker_budget > 0
                and near_waypoint
                and bool(self.get_parameter("continue_on_marker_timeout").value)
                and bool(self.get_parameter("do_not_block_waypoint_progress_on_marker").value)
                and self.elapsed(self.wp_started) > float(self.get_parameter("marker_wait_timeout_sec").value)
            )
            if reached:
                if wp.hold_sec > 0.0:
                    if self.wp_hold_started is None:
                        self.wp_hold_started = self.now()
                        self.publish_text(self.exploration_event_pub, f"WAYPOINT_REACHED:{wp.tag}")
                    if self.elapsed(self.wp_hold_started) < wp.hold_sec:
                        return
                self.advance_waypoint("arrived", wp, dxy, dz)
            elif self.proximity_confirm_reached(wp, dxy):
                self.advance_waypoint("proximity_confirmed", wp, dxy, dz)
            elif xy_close_force and not self.is_mandatory_waypoint(wp):
                self.advance_waypoint("xy_close_force", wp, dxy, dz)
            elif (stuck or max_same_elapsed) and not self.is_mandatory_waypoint(wp):
                self.advance_waypoint("stuck_skip", wp, dxy, dz)
            elif marker_wait_done:
                self.advance_waypoint("marker_timeout_continue", wp, dxy, dz)
            elif timed_out and not self.is_mandatory_waypoint(wp):
                self.advance_waypoint("timeout_skip", wp, dxy, dz)
            return

        if self.phase == "landing":
            self.publish_text(self.landing_state_pub, "landing")
            self.publish_gimbal(float(self.get_parameter("landing_gimbal_pitch_deg").value))
            ugv_pose = self.current_ugv_landing_pose()
            ugv_xy = (ugv_pose[0], ugv_pose[1]) if ugv_pose is not None else None
            default_landing_yaw = ugv_pose[2] if ugv_pose is not None else yaw
            pre_approach_alt = float(self.get_parameter("landing_pre_approach_altitude_m").value)
            descent_step = float(self.get_parameter("landing_descent_step_m").value)
            complete_alt = float(self.get_parameter("landing_complete_altitude_m").value)
            approach_alt = float(self.get_parameter("landing_approach_altitude_m").value)
            xy_tolerance = float(self.get_parameter("landing_xy_tolerance_m").value)
            yaw_tolerance = math.radians(float(self.get_parameter("landing_yaw_tolerance_deg").value))
            align_hold_sec = float(self.get_parameter("landing_align_hold_sec").value)
            if self.landing_target_xy is None:
                self.landing_target_xy = self.rendezvous_goal_xy
            if self.landing_target_yaw is None:
                self.landing_target_yaw = default_landing_yaw
            target_x, target_y = self.landing_target_xy
            landing_yaw = self.landing_target_yaw
            xy_error = math.hypot(target_x - x, target_y - y)
            yaw_error = abs(math.atan2(math.sin(landing_yaw - yaw), math.cos(landing_yaw - yaw)))

            if self.landing_stage == "pre_approach":
                target_z = pre_approach_alt
                self.cmd_pose_pub.publish(self.pose_msg(target_x, target_y, target_z, landing_yaw))
                z_error = abs(z - target_z)
                if xy_error <= xy_tolerance and yaw_error <= yaw_tolerance and z_error <= max(0.3, descent_step):
                    self.landing_stage = "marker_search"
                    self.publish_text(self.landing_event_pub, "landing_pre_approach_complete")
                    self.publish_text(self.landing_event_pub, "landing_marker_search_started")
                return

            if self.landing_stage == "marker_search":
                target_z = pre_approach_alt
                self.cmd_pose_pub.publish(self.pose_msg(target_x, target_y, target_z, landing_yaw))
                if self.landing_detection_is_usable(ugv_xy):
                    self.landing_target_xy = ugv_xy if ugv_xy is not None else self.landing_detection_xy()
                    self.landing_target_yaw = self.choose_landing_yaw(default_landing_yaw)
                    self.landing_stage = "descend"
                    self.landing_align_started = None
                    self.get_logger().info(
                        f"LANDING_MARKER_LOCK target_x={self.landing_target_xy[0]:.2f} "
                        f"target_y={self.landing_target_xy[1]:.2f} "
                        f"target_yaw_deg={math.degrees(self.landing_target_yaw):.1f}"
                    )
                    self.publish_text(
                        self.landing_event_pub,
                        "landing_marker_lock_acquired:"
                        f"x={self.landing_target_xy[0]:.2f}:y={self.landing_target_xy[1]:.2f}:"
                        f"yaw_deg={math.degrees(self.landing_target_yaw):.1f}",
                    )
                return

            freeze_after_align = bool(self.get_parameter("landing_freeze_target_after_align").value)
            allow_target_updates = self.landing_land_started is None and not (
                freeze_after_align and self.landing_align_started is not None
            )
            if allow_target_updates and self.landing_detection_is_usable(ugv_xy):
                detection_xy = ugv_xy if ugv_xy is not None else self.landing_detection_xy()
                jump_limit = float(self.get_parameter("landing_target_update_max_jump_m").value)
                jump_distance = math.hypot(
                    detection_xy[0] - self.landing_target_xy[0],
                    detection_xy[1] - self.landing_target_xy[1],
                )
                if jump_limit <= 0.0 or jump_distance <= jump_limit:
                    self.landing_target_xy = detection_xy
                    self.landing_target_yaw = self.choose_landing_yaw(default_landing_yaw)
                    target_x, target_y = self.landing_target_xy
                    landing_yaw = self.landing_target_yaw
                    xy_error = math.hypot(target_x - x, target_y - y)
                    yaw_error = abs(math.atan2(math.sin(landing_yaw - yaw), math.cos(landing_yaw - yaw)))
                    self.get_logger().info(
                        f"LANDING_TARGET_UPDATE target_x={target_x:.2f} target_y={target_y:.2f} "
                        f"xy_error={xy_error:.2f} yaw_error_deg={math.degrees(yaw_error):.1f}",
                        throttle_duration_sec=1.0,
                    )
                else:
                    self.get_logger().warn(
                        f"LANDING_TARGET_JUMP_REJECTED jump={jump_distance:.2f} "
                        f"candidate_x={detection_xy[0]:.2f} candidate_y={detection_xy[1]:.2f}",
                        throttle_duration_sec=1.0,
                    )
            at_approach_alt = z <= approach_alt + max(0.1, descent_step * 0.5)

            target_z = max(complete_alt, z - descent_step)
            if self.landing_land_started is not None:
                target_z = float(self.get_parameter("landing_touchdown_setpoint_m").value)
            elif xy_error > xy_tolerance or yaw_error > yaw_tolerance:
                target_z = max(approach_alt, z - descent_step)
                self.landing_align_started = None
            elif not at_approach_alt:
                target_z = max(approach_alt, z - descent_step)
                self.landing_align_started = None
            else:
                target_z = approach_alt
                if self.landing_align_started is None:
                    self.landing_align_started = self.now()
            self.cmd_pose_pub.publish(self.pose_msg(target_x, target_y, target_z, landing_yaw))
            ready_to_land = (
                self.landing_align_started is not None
                and self.elapsed(self.landing_align_started) >= align_hold_sec
                and xy_error <= xy_tolerance
                and yaw_error <= yaw_tolerance
                and at_approach_alt
            )
            timed_out_on_target = (
                self.elapsed(self.landing_started) > float(self.get_parameter("landing_timeout_s").value)
                and xy_error <= xy_tolerance
                and yaw_error <= yaw_tolerance
                and self.landing_stage == "descend"
            )
            if ready_to_land or timed_out_on_target:
                if self.landing_land_started is None:
                    self.landing_land_started = self.now()
                    self.publish_text(self.landing_event_pub, "landing_land_command_started")
                self.publish_bool(self.land_pub)
                if not self.landing_complete_published:
                    self.landing_complete_published = True
                    self.publish_text(self.landing_event_pub, "landing_touchdown_pending")

    def select_landing_detection(self, msg):
        landing_marker_id = int(self.get_parameter("landing_marker_id").value)
        for detection in msg.detections:
            if not detection.results:
                continue
            result = detection.results[0]
            try:
                marker_id = int(result.hypothesis.class_id)
            except (TypeError, ValueError):
                continue
            if landing_marker_id >= 0 and marker_id != landing_marker_id:
                continue
            pose = result.pose.pose
            return {
                "marker_id": marker_id,
                "map_x": pose.position.x,
                "map_y": pose.position.y,
                "map_z": pose.position.z,
                "yaw": yaw_from_quaternion(pose.orientation),
                "has_map": True,
            }
        return None

    def current_ugv_landing_xy(self):
        pose = self.current_ugv_landing_pose()
        return (pose[0], pose[1]) if pose is not None else self.last_ugv_landing_xy

    def current_ugv_landing_pose(self):
        last_error = None
        for frame in (self.ugv_landing_frame, self.ugv_base_frame):
            try:
                transform = self.tf_buffer.lookup_transform(self.map_frame, frame, rclpy.time.Time())
                pos = transform.transform.translation
                yaw = yaw_from_quaternion(transform.transform.rotation)
                self.last_ugv_landing_xy = (pos.x, pos.y)
                return pos.x, pos.y, yaw
            except TransformException as exc:
                last_error = (frame, exc)
        if last_error is not None:
            frame, exc = last_error
            self.get_logger().warn(
                f"Waiting for UGV landing TF {self.map_frame}->{frame}: {exc}",
                throttle_duration_sec=2.0,
            )
        return None

    def landing_detection_has_map(self):
        if not self.last_landing_detection or not self.last_landing_detection.get("has_map", False):
            return False
        if self.last_landing_detection_time is None:
            return False
        if self.elapsed(self.last_landing_detection_time) > float(self.get_parameter("landing_detection_timeout_s").value):
            return False
        for key in ("map_x", "map_y"):
            try:
                if not math.isfinite(float(self.last_landing_detection[key])):
                    return False
            except (KeyError, TypeError, ValueError):
                return False
        return True

    def landing_detection_is_usable(self, ugv_xy):
        if not self.landing_detection_has_map():
            return False
        if ugv_xy is None:
            return True
        marker_xy = (
            float(self.last_landing_detection["map_x"]),
            float(self.last_landing_detection["map_y"]),
        )
        max_distance = float(self.get_parameter("landing_marker_max_ugv_distance_m").value)
        return math.hypot(marker_xy[0] - ugv_xy[0], marker_xy[1] - ugv_xy[1]) <= max_distance

    def landing_detection_xy(self):
        return (
            float(self.last_landing_detection["map_x"]),
            float(self.last_landing_detection["map_y"]),
        )

    def landing_detection_yaw(self, fallback_yaw):
        if not self.last_landing_detection:
            return fallback_yaw
        try:
            yaw = float(self.last_landing_detection["yaw"])
        except (KeyError, TypeError, ValueError):
            return fallback_yaw
        return yaw if math.isfinite(yaw) else fallback_yaw

    def choose_landing_yaw(self, fallback_yaw):
        if not bool(self.get_parameter("landing_use_marker_yaw").value):
            return fallback_yaw
        marker_yaw = self.landing_detection_yaw(fallback_yaw)
        max_error = math.radians(float(self.get_parameter("landing_marker_yaw_max_error_deg").value))
        yaw_error = abs(math.atan2(math.sin(marker_yaw - fallback_yaw), math.cos(marker_yaw - fallback_yaw)))
        if yaw_error > max_error:
            self.get_logger().warn(
                f"LANDING_MARKER_YAW_REJECTED marker_yaw_deg={math.degrees(marker_yaw):.1f} "
                f"fallback_yaw_deg={math.degrees(fallback_yaw):.1f} "
                f"yaw_error_deg={math.degrees(yaw_error):.1f}",
                throttle_duration_sec=1.0,
            )
            return fallback_yaw
        return marker_yaw

    def build_mission2_runtime_waypoints(self, start_x, start_y, start_z):
        if not self.csv_waypoints or not bool(self.get_parameter("enable_mission2_transition_corridor").value):
            return list(self.csv_waypoints)

        first_wp = self.csv_waypoints[0]
        min_z = float(self.get_parameter("transition_corridor_min_cruise_altitude").value)
        max_z = float(self.get_parameter("transition_corridor_max_cruise_altitude").value)
        cruise_z = max(min_z, min(first_wp.z, max_z))
        hold_sec = max(0.0, float(self.get_parameter("transition_corridor_hold_sec").value))
        pitch = float(self.get_parameter("transition_corridor_gimbal_pitch_deg").value)
        tag_prefix = str(self.get_parameter("transition_corridor_tag_prefix").value)
        transition = []

        def append_transition(x, y, z, yaw):
            tag = f"{tag_prefix}_{len(transition) + 1:03d}"
            transition.append(Waypoint(x, y, z, yaw, pitch, 0, tag, hold_sec))

        if start_z < cruise_z - 0.1:
            append_transition(start_x, start_y, cruise_z, first_wp.yaw)

        self.publish_text(
            self.exploration_event_pub,
            "MISSION2_TRANSITION_CORRIDOR_CREATED "
            f"count={len(transition)} start=({start_x:.2f},{start_y:.2f},{start_z:.2f}) "
            f"first_wp=({first_wp.x:.2f},{first_wp.y:.2f},{first_wp.z:.2f}) cruise_z={cruise_z:.2f}",
        )
        return transition + list(self.csv_waypoints)

    def start_current_waypoint(self):
        while self.index < len(self.waypoints):
            waypoint = self.waypoints[self.index]
            expected_marker_id = self.expected_marker_id(waypoint)
            if (
                self.is_mandatory_waypoint(waypoint)
                or expected_marker_id is None
                or expected_marker_id not in self.detected_marker_ids
            ):
                break
            self.publish_text(
                self.exploration_event_pub,
                f"WAYPOINT_SKIPPED_ALREADY_DETECTED:{waypoint.tag}:marker={expected_marker_id}",
            )
            self.index += 1
        self.wp_started = self.now()
        self.wp_hold_started = None
        self.xy_close_started = None
        self.last_wp_error_log = self.now()
        if self.index < len(self.waypoints):
            self.publish_text(self.exploration_event_pub, f"NEXT_WAYPOINT:{self.index}:{self.waypoints[self.index].tag}")

    def compute_waypoint_error(self, current, target):
        x, y, z = current
        dx = target.x - x
        dy = target.y - y
        dz = target.z - z
        dxy = math.hypot(dx, dy)
        d3 = math.sqrt(dx * dx + dy * dy + dz * dz)
        return dx, dy, dz, dxy, d3

    def waypoint_tolerances(self, waypoint):
        xy_tol = float(self.get_parameter("waypoint_xy_tolerance_m").value)
        z_tol = float(self.get_parameter("waypoint_z_tolerance_m").value)
        if waypoint.tag.startswith(str(self.get_parameter("transition_corridor_tag_prefix").value)):
            xy_tol = max(xy_tol, 3.0)
            z_tol = max(z_tol, 4.0)
        return xy_tol, z_tol

    def waypoint_reached(self, waypoint, dxy, dz, d3):
        if str(self.get_parameter("waypoint_arrival_mode").value) == "xy_z_separate":
            xy_tol, z_tol = self.waypoint_tolerances(waypoint)
            return dxy <= xy_tol and abs(dz) <= z_tol
        return d3 <= float(self.get_parameter("waypoint_tolerance_m").value)

    def expected_marker_id(self, waypoint):
        match = re.search(r"marker_(\d+)", waypoint.tag)
        if not match:
            return None
        return int(match.group(1))

    def is_mandatory_waypoint(self, waypoint):
        return waypoint.tag in self.mandatory_waypoint_tags

    def proximity_confirm_reached(self, waypoint, dxy):
        return (
            waypoint.tag in self.proximity_confirm_waypoint_tags
            and dxy <= float(self.get_parameter("proximity_confirm_distance_m").value)
        )

    def xy_close_force_advance(self, waypoint, dxy, dz):
        if not bool(self.get_parameter("force_advance_when_xy_close").value):
            self.xy_close_started = None
            return False
        xy_tol, _ = self.waypoint_tolerances(waypoint)
        if dxy > xy_tol:
            self.xy_close_started = None
            return False
        if self.xy_close_started is None:
            self.xy_close_started = self.now()
            return False
        return self.elapsed(self.xy_close_started) >= float(self.get_parameter("xy_close_timeout_sec").value)

    def log_waypoint_error(self, waypoint, dxy, dz, d3):
        if self.elapsed(self.last_wp_error_log) < 1.0:
            return
        self.last_wp_error_log = self.now()
        self.get_logger().info(
            f"WP_ERROR idx={self.index} tag={waypoint.tag} dxy={dxy:.2f} dz={dz:.2f} d3={d3:.2f}"
        )

    def advance_waypoint(self, reason, waypoint, dxy, dz):
        if reason == "arrived":
            event = f"WAYPOINT_REACHED:{waypoint.tag}:dxy={dxy:.2f}:dz={dz:.2f}"
        elif reason == "xy_close_force":
            event = f"WAYPOINT_XY_CLOSE_FORCE_ADVANCE:{waypoint.tag}:dxy={dxy:.2f}:dz={dz:.2f}"
        elif reason == "stuck_skip":
            event = f"WAYPOINT_STUCK_SKIP:{waypoint.tag}:dxy={dxy:.2f}:dz={dz:.2f}"
        elif reason == "marker_timeout_continue":
            event = f"MARKER_TIMEOUT_CONTINUE:{waypoint.tag}:dxy={dxy:.2f}:dz={dz:.2f}"
        elif reason == "marker_detected":
            marker_id = self.expected_marker_id(waypoint)
            event = f"WAYPOINT_MARKER_DETECTED_ADVANCE:{waypoint.tag}:marker={marker_id}:dxy={dxy:.2f}:dz={dz:.2f}"
        elif reason == "marker_already_detected":
            marker_id = self.expected_marker_id(waypoint)
            event = f"WAYPOINT_MARKER_ALREADY_DETECTED_SKIP:{waypoint.tag}:marker={marker_id}"
        elif reason == "proximity_confirmed":
            event = f"WAYPOINT_PROXIMITY_CONFIRMED:{waypoint.tag}:dxy={dxy:.2f}:dz={dz:.2f}"
        else:
            event = f"WAYPOINT_TIMEOUT_SKIP:{waypoint.tag}:dxy={dxy:.2f}:dz={dz:.2f}"
        self.publish_text(self.exploration_event_pub, event)
        self.publish_text(self.exploration_event_pub, f"waypoint_{self.index}_{reason}")
        self.index += 1
        self.start_current_waypoint()


def main(args=None):
    rclpy.init(args=args)
    node = UavMissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
