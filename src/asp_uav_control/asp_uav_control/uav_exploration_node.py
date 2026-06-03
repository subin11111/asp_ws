import csv
import json
import math
import re
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Set

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener


class ExplorationState(str, Enum):
    IDLE = 'IDLE'
    WAITING_FOR_TF = 'WAITING_FOR_TF'
    WAITING_FOR_MISSION2_ORIGIN = 'WAITING_FOR_MISSION2_ORIGIN'
    MISSION2_ORIGIN_LATCHED = 'MISSION2_ORIGIN_LATCHED'
    READY = 'READY'
    TAKEOFF_CLIMB = 'TAKEOFF_CLIMB'
    TAKEOFF_HOLD = 'TAKEOFF_HOLD'
    TAKEOFF_COMPLETE = 'TAKEOFF_COMPLETE'
    EXPLORING = 'EXPLORING'
    HOLDING = 'HOLDING'
    WAYPOINT_REACHED = 'WAYPOINT_REACHED'
    EXPLORATION_COMPLETE = 'EXPLORATION_COMPLETE'
    TIMEOUT = 'TIMEOUT'
    ERROR = 'ERROR'


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    yaw_deg: float
    gimbal_pitch_deg: float
    hold_sec: float
    tag: str


class UavExplorationNode(Node):
    def __init__(self):
        super().__init__('uav_exploration_node')
        self.declare_params()
        self.read_params()

        self.pose_pub = self.create_publisher(PoseStamped, self.command_pose_topic, 10)
        self.gimbal_pub = self.create_publisher(Float32, self.gimbal_pitch_topic, 10)
        self.state_pub = self.create_publisher(String, self.exploration_state_topic, 10)
        self.event_pub = self.create_publisher(String, self.exploration_event_topic, 10)
        self.complete_pub = self.create_publisher(Bool, self.exploration_complete_topic, 10)
        self.mission2_origin_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.mission2_origin_pub = self.create_publisher(
            PoseStamped, self.mission2_takeoff_origin_topic, self.mission2_origin_qos)

        self.create_subscription(String, self.mission_state_topic, self.mission_state_cb, 10)
        self.create_subscription(String, self.mission_event_topic, self.mission_event_cb, 10)
        self.create_subscription(Bool, self.exploration_start_topic, self.exploration_start_cb, 10)
        self.create_subscription(String, self.marker_detections_topic, self.marker_detections_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.raw_waypoints: List[Waypoint] = self.load_path(self.path_csv)
        self.scan_waypoints: List[Waypoint] = self.filter_scan_waypoints(self.raw_waypoints)
        self.waypoints: List[Waypoint] = []
        self.state = ExplorationState.IDLE if self.scan_waypoints else ExplorationState.ERROR
        self.current_index = 0
        self.hold_start = None
        self.takeoff_target: Optional[Waypoint] = None
        self.takeoff_hold_start = None
        self.path_following_started = False
        self.exploration_start_time = None
        self.complete_published = False
        self.unique_marker_ids: Set[int] = set()
        self.current_pose: Optional[PoseStamped] = None
        self.mission2_origin_latched = False
        self.mission2_origin_stamp = None
        self.mission2_origin_x = 0.0
        self.mission2_origin_y = 0.0
        self.mission2_origin_z = 0.0
        self.last_pose_publish_tag: Optional[str] = None
        self.first_pose_published = False
        self.pending_start_requested = False
        self.pending_start_reason = ''
        if self.clear_origin_on_startup:
            self.clear_mission2_takeoff_origin()

        period = 1.0 / max(1.0, self.pose_publish_rate_hz)
        self.timer = self.create_timer(period, self.timer_cb)

        if self.start_on_launch and self.scan_waypoints:
            self.start_exploration('start_on_launch')

        self.get_logger().info(f'UAV exploration path_csv: {self.path_csv}')
        self.get_logger().info(f'Loaded raw waypoints: {len(self.raw_waypoints)}')
        self.get_logger().info(f'Loaded UAV scan waypoints: {len(self.scan_waypoints)}')
        if self.scan_waypoints:
            first = self.scan_waypoints[0]
            self.get_logger().info(
                'First scan waypoint: '
                f'x={first.x}, y={first.y}, z={first.z}, '
                f'yaw_deg={first.yaw_deg}, '
                f'gimbal_pitch_deg={first.gimbal_pitch_deg}, tag={first.tag}')
        self.get_logger().info(f'command_pose_topic: {self.command_pose_topic}')
        if self.clear_origin_on_startup:
            self.get_logger().info('Mission2 takeoff origin cleared on startup.')

    def declare_params(self):
        self.declare_parameter('path_csv', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'x500_gimbal_0/base_link')
        self.declare_parameter('command_pose_topic', '/command/pose')
        self.declare_parameter('gimbal_pitch_topic', '/gimbal_pitch_degree')
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('mission_event_topic', '/ugv/mission_event')
        self.declare_parameter('mission2_start_event', 'MISSION2_START_REACHED')
        self.declare_parameter('exploration_start_topic', '/uav/exploration_start')
        self.declare_parameter('exploration_state_topic', '/uav/exploration_state')
        self.declare_parameter('exploration_event_topic', '/uav/exploration_event')
        self.declare_parameter('exploration_complete_topic', '/mission/uav_exploration_complete')
        self.declare_parameter('mission2_takeoff_origin_topic', '/uav/mission2_takeoff_origin')
        self.declare_parameter('use_mission2_latched_origin', True)
        self.declare_parameter('require_mission2_latched_origin', True)
        self.declare_parameter('clear_origin_on_startup', True)
        self.declare_parameter('mission2_origin_map_frame', 'map')
        self.declare_parameter('mission2_origin_uav_frame', 'x500_gimbal_0/base_link')
        self.declare_parameter('marker_detections_topic', '/perception/uav/marker_detections')
        self.declare_parameter('start_on_launch', False)
        self.declare_parameter('start_on_mission_state', True)
        self.declare_parameter('required_start_state', 'UAV_EXPLORATION_READY')
        self.declare_parameter('required_start_states', ['UAV_EXPLORATION_READY', 'UAV_TAKEOFF_READY'])
        self.declare_parameter('pose_publish_rate_hz', 10.0)
        self.declare_parameter('waypoint_tolerance', 1.0)
        self.declare_parameter('yaw_tolerance_deg', 20.0)
        self.declare_parameter('default_hold_sec', 3.0)
        self.declare_parameter('complete_on_path_done', True)
        self.declare_parameter('minimum_unique_markers', 0)
        self.declare_parameter('exploration_timeout_sec', 300.0)
        self.declare_parameter('resend_current_pose_before_start', True)
        self.declare_parameter('use_safe_path', False)
        self.declare_parameter('safe_path_csv', '')
        self.declare_parameter('min_marker_id', 0)
        self.declare_parameter('max_marker_id', 49)
        self.declare_parameter('dynamic_safe_prefix', True)
        self.declare_parameter('force_takeoff_before_path', True)
        self.declare_parameter('takeoff_mode', 'relative')
        self.declare_parameter('takeoff_relative_height', 8.0)
        self.declare_parameter('takeoff_absolute_altitude', 18.0)
        self.declare_parameter('takeoff_hold_sec', 3.0)
        self.declare_parameter('takeoff_tolerance', 0.6)
        self.declare_parameter('takeoff_xy_tolerance', 0.75)
        self.declare_parameter('takeoff_gimbal_pitch_deg', -60.0)
        self.declare_parameter('block_pose_if_not_latched', True)
        self.declare_parameter('block_pose_if_xy_not_origin', True)
        self.declare_parameter('forbidden_spawn_guard_enabled', True)
        self.declare_parameter('forbidden_spawn_x', -55.0)
        self.declare_parameter('forbidden_spawn_y', 80.0)
        self.declare_parameter('forbidden_spawn_radius', 5.0)
        self.declare_parameter('safe_takeoff_relative_height', 5.0)
        self.declare_parameter('safe_altitude', 18.0)
        self.declare_parameter('safe_altitude_after_takeoff', 18.0)
        self.declare_parameter('transition_altitude', 18.0)
        self.declare_parameter('max_transition_step', 15.0)
        self.declare_parameter('safe_prefix_hold_sec', 2.0)
        self.declare_parameter('safe_prefix_gimbal_pitch_deg', -60.0)

    def read_params(self):
        self.path_csv = self.get_parameter('path_csv').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.command_pose_topic = self.get_parameter('command_pose_topic').value
        self.gimbal_pitch_topic = self.get_parameter('gimbal_pitch_topic').value
        self.mission_state_topic = self.get_parameter('mission_state_topic').value
        self.mission_event_topic = self.get_parameter('mission_event_topic').value
        self.mission2_start_event = self.get_parameter('mission2_start_event').value
        self.exploration_start_topic = self.get_parameter('exploration_start_topic').value
        self.exploration_state_topic = self.get_parameter('exploration_state_topic').value
        self.exploration_event_topic = self.get_parameter('exploration_event_topic').value
        self.exploration_complete_topic = self.get_parameter('exploration_complete_topic').value
        self.mission2_takeoff_origin_topic = self.get_parameter(
            'mission2_takeoff_origin_topic').value
        self.use_mission2_latched_origin = self.get_parameter(
            'use_mission2_latched_origin').value
        self.require_mission2_latched_origin = self.get_parameter(
            'require_mission2_latched_origin').value
        self.clear_origin_on_startup = self.get_parameter('clear_origin_on_startup').value
        self.mission2_origin_map_frame = self.get_parameter('mission2_origin_map_frame').value
        self.mission2_origin_uav_frame = self.get_parameter('mission2_origin_uav_frame').value
        self.marker_detections_topic = self.get_parameter('marker_detections_topic').value
        self.start_on_launch = self.get_parameter('start_on_launch').value
        self.start_on_mission_state = self.get_parameter('start_on_mission_state').value
        self.required_start_state = self.get_parameter('required_start_state').value
        self.required_start_states = list(self.get_parameter('required_start_states').value)
        self.pose_publish_rate_hz = float(self.get_parameter('pose_publish_rate_hz').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.yaw_tolerance_deg = float(self.get_parameter('yaw_tolerance_deg').value)
        self.default_hold_sec = float(self.get_parameter('default_hold_sec').value)
        self.complete_on_path_done = self.get_parameter('complete_on_path_done').value
        self.minimum_unique_markers = int(self.get_parameter('minimum_unique_markers').value)
        self.exploration_timeout_sec = float(self.get_parameter('exploration_timeout_sec').value)
        self.resend_current_pose_before_start = self.get_parameter(
            'resend_current_pose_before_start').value
        self.use_safe_path = self.get_parameter('use_safe_path').value
        self.safe_path_csv = self.get_parameter('safe_path_csv').value
        self.min_marker_id = int(self.get_parameter('min_marker_id').value)
        self.max_marker_id = int(self.get_parameter('max_marker_id').value)
        self.dynamic_safe_prefix = self.get_parameter('dynamic_safe_prefix').value
        self.force_takeoff_before_path = self.get_parameter('force_takeoff_before_path').value
        self.takeoff_mode = str(self.get_parameter('takeoff_mode').value).lower()
        self.takeoff_relative_height = float(self.get_parameter('takeoff_relative_height').value)
        self.takeoff_absolute_altitude = float(
            self.get_parameter('takeoff_absolute_altitude').value)
        self.takeoff_hold_sec = float(self.get_parameter('takeoff_hold_sec').value)
        self.takeoff_tolerance = float(self.get_parameter('takeoff_tolerance').value)
        self.takeoff_xy_tolerance = float(self.get_parameter('takeoff_xy_tolerance').value)
        self.takeoff_gimbal_pitch_deg = float(
            self.get_parameter('takeoff_gimbal_pitch_deg').value)
        self.block_pose_if_not_latched = self.get_parameter('block_pose_if_not_latched').value
        self.block_pose_if_xy_not_origin = self.get_parameter(
            'block_pose_if_xy_not_origin').value
        self.forbidden_spawn_guard_enabled = self.get_parameter(
            'forbidden_spawn_guard_enabled').value
        self.forbidden_spawn_x = float(self.get_parameter('forbidden_spawn_x').value)
        self.forbidden_spawn_y = float(self.get_parameter('forbidden_spawn_y').value)
        self.forbidden_spawn_radius = float(self.get_parameter('forbidden_spawn_radius').value)
        self.safe_takeoff_relative_height = float(
            self.get_parameter('safe_takeoff_relative_height').value)
        self.safe_altitude = float(self.get_parameter('safe_altitude').value)
        self.safe_altitude_after_takeoff = float(
            self.get_parameter('safe_altitude_after_takeoff').value)
        self.transition_altitude = float(self.get_parameter('transition_altitude').value)
        self.max_transition_step = float(self.get_parameter('max_transition_step').value)
        self.safe_prefix_hold_sec = float(self.get_parameter('safe_prefix_hold_sec').value)
        self.safe_prefix_gimbal_pitch_deg = float(
            self.get_parameter('safe_prefix_gimbal_pitch_deg').value)

    def load_path(self, path_csv: str) -> List[Waypoint]:
        if not path_csv:
            self.get_logger().error('path_csv parameter is empty')
            return []
        waypoints: List[Waypoint] = []
        try:
            with open(path_csv, newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    waypoints.append(Waypoint(
                        x=float(row['x']),
                        y=float(row['y']),
                        z=float(row['z']),
                        yaw_deg=float(row['yaw_deg']),
                        gimbal_pitch_deg=float(row['gimbal_pitch_deg']),
                        hold_sec=float(row.get('hold_sec') or self.default_hold_sec),
                        tag=row.get('tag', f'wp_{len(waypoints)}'),
                    ))
        except (OSError, KeyError, ValueError) as exc:
            self.get_logger().error(f'Failed to load UAV path CSV {path_csv}: {exc}')
            return []
        return waypoints

    def filter_scan_waypoints(self, waypoints: List[Waypoint]) -> List[Waypoint]:
        if not (self.dynamic_safe_prefix or self.use_mission2_latched_origin):
            return waypoints
        scan_waypoints = [
            waypoint
            for waypoint in waypoints
            if not self.is_static_safe_prefix_tag(waypoint.tag, waypoint)
        ]
        removed_count = len(waypoints) - len(scan_waypoints)
        self.get_logger().info(f'Removed static/spawn prefix rows: {removed_count}')
        self.get_logger().info(f'Remaining scan waypoints: {len(scan_waypoints)}')
        return scan_waypoints

    def is_static_safe_prefix_tag(self, tag: str, waypoint: Optional[Waypoint] = None) -> bool:
        tag_lower = tag.lower()
        is_prefix = (
            tag_lower == 'takeoff_climb'
            or tag_lower == 'forced_takeoff_climb'
            or tag_lower == 'safe_altitude'
            or tag_lower.startswith('transition_')
            or 'spawn' in tag_lower
        )
        if is_prefix:
            return True
        if waypoint is None:
            return False
        return (
            self.is_near_forbidden_spawn(waypoint.x, waypoint.y)
            and (
                'takeoff' in tag_lower
                or 'safe' in tag_lower
                or 'transition' in tag_lower
            )
        )

    def clear_mission2_takeoff_origin(self):
        self.mission2_origin_latched = False
        self.mission2_origin_stamp = None
        self.mission2_origin_x = 0.0
        self.mission2_origin_y = 0.0
        self.mission2_origin_z = 0.0

    def is_near_forbidden_spawn(self, x: float, y: float) -> bool:
        return (
            math.hypot(x - self.forbidden_spawn_x, y - self.forbidden_spawn_y)
            <= self.forbidden_spawn_radius
        )

    def mission_state_cb(self, msg: String):
        if (
            self.start_on_mission_state
            and self.is_required_start_state(msg.data)
            and self.state in (
                ExplorationState.IDLE,
                ExplorationState.READY,
                ExplorationState.WAITING_FOR_TF,
                ExplorationState.WAITING_FOR_MISSION2_ORIGIN,
                ExplorationState.MISSION2_ORIGIN_LATCHED,
            )
        ):
            self.start_exploration(f'mission_state:{msg.data}')

    def is_required_start_state(self, state: str) -> bool:
        return state == self.required_start_state or state in self.required_start_states

    def exploration_start_cb(self, msg: Bool):
        if msg.data and self.state in (
            ExplorationState.IDLE,
            ExplorationState.READY,
            ExplorationState.WAITING_FOR_TF,
            ExplorationState.WAITING_FOR_MISSION2_ORIGIN,
            ExplorationState.MISSION2_ORIGIN_LATCHED,
        ):
            self.start_exploration('manual_start')

    def mission_event_cb(self, msg: String):
        if msg.data.strip() != self.mission2_start_event:
            return

        origin_tf = self.lookup_tf(
            self.mission2_origin_map_frame, self.mission2_origin_uav_frame)
        if origin_tf is None:
            self.clear_mission2_takeoff_origin()
            self.publish_event('MISSION2_TAKEOFF_ORIGIN_LATCH_FAILED')
            self.get_logger().warn('Failed to latch Mission2 takeoff origin from UAV TF.')
            return

        self.mission2_origin_x = origin_tf.transform.translation.x
        self.mission2_origin_y = origin_tf.transform.translation.y
        self.mission2_origin_z = origin_tf.transform.translation.z
        self.mission2_origin_latched = True
        self.mission2_origin_stamp = self.get_clock().now()
        if self.state in (
            ExplorationState.IDLE,
            ExplorationState.READY,
            ExplorationState.WAITING_FOR_TF,
            ExplorationState.WAITING_FOR_MISSION2_ORIGIN,
        ):
            self.state = ExplorationState.MISSION2_ORIGIN_LATCHED
        self.publish_mission2_takeoff_origin()
        self.publish_event('MISSION2_TAKEOFF_ORIGIN_LATCHED')
        self.get_logger().info(
            'Mission2 takeoff origin latched from UAV TF: '
            f'x={self.mission2_origin_x}, y={self.mission2_origin_y}, '
            f'z={self.mission2_origin_z}')
        if self.pending_start_requested:
            reason = self.pending_start_reason or 'pending_start'
            self.pending_start_requested = False
            self.pending_start_reason = ''
            self.publish_event('PENDING_START_RESUMED_AFTER_ORIGIN_LATCH')
            self.start_exploration(reason)

    def marker_detections_cb(self, msg: String):
        marker_ids = self.extract_marker_ids(msg.data)
        if not marker_ids:
            self.get_logger().warn(
                f'No marker_id key found in marker detection: {msg.data}',
                throttle_duration_sec=2.0)
            return
        for marker_id in marker_ids:
            if not self.is_marker_id_in_range(marker_id):
                self.get_logger().warn(
                    f'Ignoring out-of-range UAV marker ID: {marker_id}',
                    throttle_duration_sec=2.0)
                continue
            if marker_id not in self.unique_marker_ids:
                self.unique_marker_ids.add(marker_id)
                self.get_logger().info(f'Unique UAV marker recorded: {marker_id}')

    def extract_marker_ids(self, text: str) -> List[int]:
        try:
            parsed = json.loads(text)
            ids = self.marker_ids_from_json(parsed)
            if ids:
                return ids
        except json.JSONDecodeError:
            pass

        return [
            int(match.group(1))
            for match in re.finditer(r'(?:marker_id|id)\s*[:=]\s*(-?\d+)', text)
        ]

    def marker_ids_from_json(self, parsed) -> List[int]:
        if isinstance(parsed, dict):
            for key in ('marker_id', 'id'):
                if key in parsed:
                    try:
                        return [int(parsed[key])]
                    except (TypeError, ValueError):
                        return []
            ids: List[int] = []
            for value in parsed.values():
                ids.extend(self.marker_ids_from_json(value))
            return ids
        if isinstance(parsed, list):
            ids: List[int] = []
            for item in parsed:
                ids.extend(self.marker_ids_from_json(item))
            return ids
        return []

    def is_marker_id_in_range(self, marker_id: int) -> bool:
        return self.min_marker_id <= marker_id <= self.max_marker_id

    def start_exploration(self, reason: str):
        self.get_logger().info('Exploration start requested.')
        if not self.scan_waypoints:
            self.state = ExplorationState.ERROR
            self.publish_event('ERROR:NO_WAYPOINTS')
            return

        current_tf = self.lookup_current_tf()
        if current_tf is not None:
            self.get_logger().info(
                'Current UAV TF at exploration start request: '
                f'x={current_tf.transform.translation.x}, '
                f'y={current_tf.transform.translation.y}, '
                f'z={current_tf.transform.translation.z}')

        if self.use_mission2_latched_origin:
            if not self.mission2_origin_latched:
                self.state = ExplorationState.WAITING_FOR_MISSION2_ORIGIN
                self.pending_start_requested = True
                self.pending_start_reason = reason
                self.get_logger().warn(
                    'Refusing UAV exploration start: Mission2 takeoff origin is not latched.')
                self.publish_event('WAITING_FOR_MISSION2_ORIGIN')
                self.publish_event('PENDING_START_STORED_WAITING_FOR_MISSION2_ORIGIN')
                return
            start_x = self.mission2_origin_x
            start_y = self.mission2_origin_y
            start_z = self.mission2_origin_z
            start_tf = None
        else:
            start_tf = current_tf
            start_x = start_tf.transform.translation.x if start_tf is not None else None
            start_y = start_tf.transform.translation.y if start_tf is not None else None
            start_z = start_tf.transform.translation.z if start_tf is not None else None

        if start_x is None or start_y is None or start_z is None:
            if start_tf is None:
                self.state = ExplorationState.WAITING_FOR_TF
                self.publish_event('WAITING_FOR_TF:START_POSE')
                return
            start_x = start_tf.transform.translation.x
            start_y = start_tf.transform.translation.y
            start_z = start_tf.transform.translation.z

        self.get_logger().info(
            f'UAV takeoff origin selected: x={start_x}, y={start_y}, z={start_z}, '
            f'mission2_origin_latched={self.mission2_origin_latched}')
        self.get_logger().info(f'Dynamic safe prefix enabled: {self.dynamic_safe_prefix}')
        self.get_logger().info(f'Original scan waypoint count: {len(self.scan_waypoints)}')

        self.current_index = 0
        self.hold_start = None
        self.takeoff_hold_start = None
        self.path_following_started = False
        self.first_pose_published = False
        self.last_pose_publish_tag = None
        self.exploration_start_time = self.get_clock().now()
        self.complete_published = False
        self.get_logger().info(f'Exploration start reason: {reason}')
        self.publish_event('EXPLORATION_STARTED')

        if self.force_takeoff_before_path:
            self.waypoints = []
            self.takeoff_target = self.create_forced_takeoff_target(start_x, start_y, start_z)
            self.state = ExplorationState.TAKEOFF_CLIMB
            self.get_logger().info('Forced takeoff before CSV/path following is enabled.')
            self.get_logger().info(
                f'Takeoff target: x={self.takeoff_target.x}, y={self.takeoff_target.y}, '
                f'z={self.takeoff_target.z}, mode={self.takeoff_mode}, '
                f'hold_sec={self.takeoff_target.hold_sec}')
            self.get_logger().info('CSV scan waypoints start only after forced takeoff hold completes.')
            self.publish_event('TAKEOFF_CLIMB_STARTED')
            return

        if start_tf is not None:
            path_started = self.start_path_following_from_tf(start_tf, include_takeoff_prefix=True)
        else:
            path_started = self.start_path_following_from_origin(
                start_x, start_y, start_z, include_takeoff_prefix=True)
        if not path_started:
            return
        self.get_logger().info(f'Exploration start reason: {reason}')

    def create_forced_takeoff_target(self, start_x: float, start_y: float, start_z: float) -> Waypoint:
        first_scan = self.scan_waypoints[0]
        heading_to_scan = self.yaw_to_target_deg(start_x, start_y, first_scan.x, first_scan.y)
        return Waypoint(
            x=start_x,
            y=start_y,
            z=self.calculate_takeoff_target_z(start_z),
            yaw_deg=heading_to_scan,
            gimbal_pitch_deg=self.clamp(self.takeoff_gimbal_pitch_deg, -90.0, 20.0),
            hold_sec=max(0.0, self.takeoff_hold_sec),
            tag='forced_takeoff_climb',
        )

    def calculate_takeoff_target_z(self, start_z: float) -> float:
        if self.takeoff_mode == 'absolute':
            return max(self.takeoff_absolute_altitude, start_z + 1.0)
        return start_z + self.takeoff_relative_height

    def start_path_following_from_tf(self, current_tf, include_takeoff_prefix: bool) -> bool:
        start_x = current_tf.transform.translation.x
        start_y = current_tf.transform.translation.y
        start_z = current_tf.transform.translation.z
        return self.start_path_following_from_origin(
            start_x, start_y, start_z, include_takeoff_prefix=include_takeoff_prefix)

    def start_path_following_from_origin(
        self,
        start_x: float,
        start_y: float,
        start_z: float,
        include_takeoff_prefix: bool,
    ) -> bool:
        if self.dynamic_safe_prefix:
            self.waypoints = self.create_dynamic_safe_path(
                start_x, start_y, start_z, include_takeoff_prefix=include_takeoff_prefix)
            self.get_logger().info('Dynamic safe prefix created from current TF pose.')
            self.publish_event('DYNAMIC_SAFE_PREFIX_CREATED')
        else:
            self.waypoints = list(self.scan_waypoints)

        if not self.waypoints:
            self.state = ExplorationState.ERROR
            self.publish_event('ERROR:NO_WAYPOINTS')
            return False

        self.log_start_path_summary()
        self.current_index = 0
        self.hold_start = None
        self.state = ExplorationState.EXPLORING
        self.path_following_started = True
        self.publish_event('PATH_FOLLOWING_STARTED')
        return True

    def create_dynamic_safe_path(
        self,
        start_x: float,
        start_y: float,
        start_z: float,
        include_takeoff_prefix: bool = True,
    ) -> List[Waypoint]:
        first_scan = self.scan_waypoints[0]
        heading_to_scan = self.yaw_to_target_deg(start_x, start_y, first_scan.x, first_scan.y)
        safe_pitch = self.clamp(self.safe_prefix_gimbal_pitch_deg, -90.0, 20.0)
        safe_hold = max(0.0, self.safe_prefix_hold_sec)
        waypoints: List[Waypoint] = []

        if include_takeoff_prefix:
            waypoints.append(Waypoint(
                x=start_x,
                y=start_y,
                z=max(5.0, start_z + self.safe_takeoff_relative_height),
                yaw_deg=heading_to_scan,
                gimbal_pitch_deg=safe_pitch,
                hold_sec=safe_hold,
                tag='takeoff_climb',
            ))
            waypoints.append(Waypoint(
                x=start_x,
                y=start_y,
                z=max(5.0, self.safe_altitude_after_takeoff),
                yaw_deg=heading_to_scan,
                gimbal_pitch_deg=safe_pitch,
                hold_sec=safe_hold,
                tag='safe_altitude',
            ))

        distance = math.hypot(first_scan.x - start_x, first_scan.y - start_y)
        if self.max_transition_step > 0.0 and distance > self.max_transition_step:
            transition_count = max(0, math.ceil(distance / self.max_transition_step) - 1)
            for index in range(1, transition_count + 1):
                ratio = index / (transition_count + 1)
                x = start_x + (first_scan.x - start_x) * ratio
                y = start_y + (first_scan.y - start_y) * ratio
                next_ratio = min(1.0, (index + 1) / (transition_count + 1))
                next_x = start_x + (first_scan.x - start_x) * next_ratio
                next_y = start_y + (first_scan.y - start_y) * next_ratio
                waypoints.append(Waypoint(
                    x=x,
                    y=y,
                    z=max(5.0, self.transition_altitude, start_z),
                    yaw_deg=self.yaw_to_target_deg(x, y, next_x, next_y),
                    gimbal_pitch_deg=safe_pitch,
                    hold_sec=safe_hold,
                    tag=f'transition_{index:03d}',
                ))

        waypoints.extend(self.scan_waypoints)
        return waypoints

    def log_start_path_summary(self):
        self.get_logger().info(f'Final waypoint count with runtime prefix: {len(self.waypoints)}')
        for index, waypoint in enumerate(self.waypoints[:5]):
            self.get_logger().info(
                f'Waypoint preview {index}: tag={waypoint.tag}, '
                f'x={waypoint.x}, y={waypoint.y}, z={waypoint.z}, '
                f'yaw_deg={waypoint.yaw_deg}, '
                f'gimbal_pitch_deg={waypoint.gimbal_pitch_deg}')
        first_scan = self.scan_waypoints[0]
        self.get_logger().info(
            f'First scan waypoint: tag={first_scan.tag}, '
            f'x={first_scan.x}, y={first_scan.y}, z={first_scan.z}, '
            f'yaw_deg={first_scan.yaw_deg}, '
            f'gimbal_pitch_deg={first_scan.gimbal_pitch_deg}')

    def yaw_to_target_deg(self, current_x: float, current_y: float, target_x: float, target_y: float) -> float:
        return self.normalize_yaw_deg(math.degrees(math.atan2(target_y - current_y, target_x - current_x)))

    def normalize_yaw_deg(self, yaw_deg: float) -> float:
        while yaw_deg > 180.0:
            yaw_deg -= 360.0
        while yaw_deg < -180.0:
            yaw_deg += 360.0
        return yaw_deg

    def clamp(self, value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    def timer_cb(self):
        if self.mission2_origin_latched:
            self.publish_mission2_takeoff_origin()

        if self.state == ExplorationState.ERROR:
            self.publish_state()
            return

        if self.state == ExplorationState.IDLE:
            self.publish_state()
            return

        if self.state in (
            ExplorationState.WAITING_FOR_MISSION2_ORIGIN,
            ExplorationState.MISSION2_ORIGIN_LATCHED,
        ):
            self.publish_state()
            return

        if self.state in (ExplorationState.EXPLORATION_COMPLETE, ExplorationState.TIMEOUT):
            self.publish_current_target_pose()
            self.publish_state()
            return

        if self.exploration_start_time is not None:
            elapsed = (self.get_clock().now() - self.exploration_start_time).nanoseconds / 1e9
            if elapsed > self.exploration_timeout_sec:
                self.state = ExplorationState.TIMEOUT
                self.publish_complete(False)
                self.publish_event('TIMEOUT')
                self.publish_state()
                return

        if self.state in (ExplorationState.TAKEOFF_CLIMB, ExplorationState.TAKEOFF_HOLD):
            self.handle_forced_takeoff()
            return

        if self.state == ExplorationState.TAKEOFF_COMPLETE:
            current_tf = self.lookup_current_tf()
            if current_tf is None:
                self.publish_state()
                return
            self.start_path_following_from_tf(current_tf, include_takeoff_prefix=False)
            self.publish_state()
            return

        if self.state == ExplorationState.WAITING_FOR_TF and not self.path_following_started:
            self.publish_state()
            return

        if self.current_index >= len(self.waypoints):
            self.finish_if_ready()
            return

        current_tf = self.lookup_current_tf()
        if current_tf is None:
            self.state = ExplorationState.WAITING_FOR_TF
            self.publish_state()
            return

        if self.state == ExplorationState.WAITING_FOR_TF:
            if self.exploration_start_time is None:
                self.publish_state()
                return
            self.state = ExplorationState.EXPLORING

        waypoint = self.waypoints[self.current_index]
        self.publish_waypoint(waypoint)

        distance = self.distance_to_waypoint(current_tf, waypoint)
        if self.state != ExplorationState.HOLDING and distance <= self.waypoint_tolerance:
            self.state = ExplorationState.HOLDING
            self.hold_start = self.get_clock().now()
            self.publish_event(f'WAYPOINT_REACHED:{waypoint.tag}')

        if self.state == ExplorationState.HOLDING:
            hold_elapsed = (self.get_clock().now() - self.hold_start).nanoseconds / 1e9
            if hold_elapsed >= max(0.0, waypoint.hold_sec):
                self.current_index += 1
                self.hold_start = None
                self.state = ExplorationState.EXPLORING
                if self.current_index >= len(self.waypoints):
                    self.finish_if_ready()

        self.publish_state()

    def handle_forced_takeoff(self):
        if self.takeoff_target is None:
            self.state = ExplorationState.ERROR
            self.publish_event('ERROR:NO_TAKEOFF_TARGET')
            self.publish_state()
            return

        self.publish_waypoint(self.takeoff_target)
        current_tf = self.lookup_current_tf()
        if current_tf is None:
            self.publish_state()
            return

        current_z = current_tf.transform.translation.z
        target_z = self.takeoff_target.z
        if (
            self.state == ExplorationState.TAKEOFF_CLIMB
            and current_z >= target_z - self.takeoff_tolerance
        ):
            self.state = ExplorationState.TAKEOFF_HOLD
            self.takeoff_hold_start = self.get_clock().now()
            self.publish_event('TAKEOFF_ALTITUDE_REACHED')

        if self.state == ExplorationState.TAKEOFF_HOLD:
            if self.takeoff_hold_start is None:
                self.takeoff_hold_start = self.get_clock().now()
            hold_elapsed = (self.get_clock().now() - self.takeoff_hold_start).nanoseconds / 1e9
            if hold_elapsed >= max(0.0, self.takeoff_hold_sec):
                self.state = ExplorationState.TAKEOFF_COMPLETE
                self.publish_event('TAKEOFF_HOLD_COMPLETE')
                latest_tf = self.lookup_current_tf()
                if latest_tf is None:
                    self.publish_state()
                    return
                self.start_path_following_from_tf(latest_tf, include_takeoff_prefix=False)

        self.publish_state()

    def lookup_current_tf(self):
        return self.lookup_tf(self.map_frame, self.base_frame)

    def lookup_tf(self, target_frame: str, source_frame: str):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except TransformException as exc:
            self.get_logger().warn(f'TF lookup failed: {exc}', throttle_duration_sec=2.0)
            return None

    def distance_to_waypoint(self, transform, waypoint: Waypoint) -> float:
        dx = waypoint.x - transform.transform.translation.x
        dy = waypoint.y - transform.transform.translation.y
        dz = waypoint.z - transform.transform.translation.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def publish_current_target_pose(self):
        if self.current_index < len(self.waypoints):
            self.publish_waypoint(self.waypoints[self.current_index])
        elif self.takeoff_target is not None:
            self.publish_waypoint(self.takeoff_target)

    def publish_waypoint(self, waypoint: Waypoint):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.position.z = waypoint.z
        yaw = math.radians(waypoint.yaw_deg)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        if not self.publish_command_pose_guarded(pose, waypoint.tag):
            return
        self.current_pose = pose

        gimbal_msg = Float32()
        gimbal_msg.data = float(waypoint.gimbal_pitch_deg)
        self.gimbal_pub.publish(gimbal_msg)

    def publish_command_pose_guarded(self, pose: PoseStamped, tag: str) -> bool:
        pose_x = pose.pose.position.x
        pose_y = pose.pose.position.y

        if self.state in (
            ExplorationState.IDLE,
            ExplorationState.WAITING_FOR_MISSION2_ORIGIN,
            ExplorationState.MISSION2_ORIGIN_LATCHED,
        ):
            self.block_command_pose(
                'POSE_BLOCKED_NO_MISSION2_ORIGIN',
                tag,
                'Command pose blocked before exploration/takeoff is active.')
            return False

        if self.block_pose_if_not_latched and not self.mission2_origin_latched:
            self.block_command_pose(
                'POSE_BLOCKED_NO_MISSION2_ORIGIN',
                tag,
                'Command pose blocked because Mission2 takeoff origin is not latched.')
            return False

        if not self.first_pose_published:
            if tag != 'forced_takeoff_climb':
                self.block_command_pose(
                    'POSE_BLOCKED_FIRST_TAKEOFF_NOT_ORIGIN',
                    tag,
                    'First command pose must be forced_takeoff_climb at Mission2 origin.')
                return False
            if (
                abs(pose_x - self.mission2_origin_x) > self.takeoff_xy_tolerance
                or abs(pose_y - self.mission2_origin_y) > self.takeoff_xy_tolerance
            ):
                self.block_command_pose(
                    'POSE_BLOCKED_FIRST_TAKEOFF_NOT_ORIGIN',
                    tag,
                    'First takeoff pose x/y differs from Mission2 origin. '
                    f'pose=({pose_x}, {pose_y}), '
                    f'origin=({self.mission2_origin_x}, {self.mission2_origin_y})')
                return False
            self.publish_event(
                'FIRST_TAKEOFF_POSE_CONFIRMED '
                f'origin=({self.mission2_origin_x},{self.mission2_origin_y},{self.mission2_origin_z}) '
                f'pose=({pose.pose.position.x},{pose.pose.position.y},{pose.pose.position.z})')

        if (
            self.block_pose_if_xy_not_origin
            and self.state in (ExplorationState.TAKEOFF_CLIMB, ExplorationState.TAKEOFF_HOLD)
            and (
                abs(pose_x - self.mission2_origin_x) > self.takeoff_xy_tolerance
                or abs(pose_y - self.mission2_origin_y) > self.takeoff_xy_tolerance
            )
        ):
            self.block_command_pose(
                'POSE_BLOCKED_XY_NOT_ORIGIN',
                tag,
                'Command pose blocked because takeoff x/y differs from Mission2 origin. '
                f'pose=({pose_x}, {pose_y}), '
                f'origin=({self.mission2_origin_x}, {self.mission2_origin_y})')
            return False

        if (
            self.forbidden_spawn_guard_enabled
            and self.is_near_forbidden_spawn(pose_x, pose_y)
            and (
                not self.mission2_origin_latched
                or not self.is_near_forbidden_spawn(self.mission2_origin_x, self.mission2_origin_y)
            )
        ):
            self.block_command_pose(
                'POSE_BLOCKED_FORBIDDEN_SPAWN',
                tag,
                'Command pose blocked near forbidden spawn coordinates. '
                f'pose=({pose_x}, {pose_y}), '
                f'forbidden=({self.forbidden_spawn_x}, {self.forbidden_spawn_y})')
            return False

        self.pose_pub.publish(pose)
        self.first_pose_published = True
        if self.last_pose_publish_tag != tag:
            self.publish_event(f'POSE_PUBLISHED:{tag}')
            self.last_pose_publish_tag = tag
        self.get_logger().info(
            f'Published guarded /command/pose tag={tag}: '
            f'x={pose.pose.position.x}, y={pose.pose.position.y}, z={pose.pose.position.z}',
            throttle_duration_sec=2.0)
        return True

    def block_command_pose(self, event: str, tag: str, reason: str):
        self.publish_event(event)
        self.get_logger().error(f'{event}: tag={tag}. {reason}')

    def finish_if_ready(self):
        enough_markers = len(self.unique_marker_ids) >= self.minimum_unique_markers
        if self.complete_on_path_done and enough_markers:
            self.state = ExplorationState.EXPLORATION_COMPLETE
            self.publish_complete(True)
            self.publish_event('EXPLORATION_COMPLETE')
        else:
            self.state = ExplorationState.HOLDING
        self.publish_state()

    def publish_complete(self, value: bool):
        if self.complete_published and value:
            return
        msg = Bool()
        msg.data = value
        self.complete_pub.publish(msg)
        self.complete_published = self.complete_published or value

    def publish_mission2_takeoff_origin(self):
        if not self.mission2_origin_latched:
            return
        pose = PoseStamped()
        pose.header.stamp = (
            self.mission2_origin_stamp.to_msg()
            if self.mission2_origin_stamp is not None
            else self.get_clock().now().to_msg()
        )
        pose.header.frame_id = self.mission2_origin_map_frame
        pose.pose.position.x = self.mission2_origin_x
        pose.pose.position.y = self.mission2_origin_y
        pose.pose.position.z = self.mission2_origin_z
        pose.pose.orientation.w = 1.0
        self.mission2_origin_pub.publish(pose)

    def publish_state(self):
        msg = String()
        status = {
            'state': self.state.value,
            'current_index': self.current_index,
            'waypoints': len(self.waypoints),
            'scan_waypoints': len(self.scan_waypoints),
            'unique_markers': sorted(self.unique_marker_ids),
            'takeoff_target_z': self.takeoff_target.z if self.takeoff_target else None,
            'path_following_started': self.path_following_started,
            'mission2_origin_latched': self.mission2_origin_latched,
            'mission2_takeoff_origin': {
                'x': self.mission2_origin_x,
                'y': self.mission2_origin_y,
                'z': self.mission2_origin_z,
            } if self.mission2_origin_latched else None,
        }
        msg.data = json.dumps(status)
        self.state_pub.publish(msg)

    def publish_event(self, text: str):
        msg = String()
        msg.data = text
        self.event_pub.publish(msg)
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = UavExplorationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
