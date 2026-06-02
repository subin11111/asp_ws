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
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener


class ExplorationState(str, Enum):
    IDLE = 'IDLE'
    WAITING_FOR_TF = 'WAITING_FOR_TF'
    READY = 'READY'
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

        self.create_subscription(String, self.mission_state_topic, self.mission_state_cb, 10)
        self.create_subscription(Bool, self.exploration_start_topic, self.exploration_start_cb, 10)
        self.create_subscription(String, self.marker_detections_topic, self.marker_detections_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_waypoints: List[Waypoint] = self.load_path(self.path_csv)
        self.waypoints: List[Waypoint] = []
        self.state = ExplorationState.IDLE if self.scan_waypoints else ExplorationState.ERROR
        self.current_index = 0
        self.hold_start = None
        self.exploration_start_time = None
        self.complete_published = False
        self.unique_marker_ids: Set[int] = set()
        self.current_pose: Optional[PoseStamped] = None

        period = 1.0 / max(1.0, self.pose_publish_rate_hz)
        self.timer = self.create_timer(period, self.timer_cb)

        if self.start_on_launch and self.scan_waypoints:
            self.start_exploration('start_on_launch')

        self.get_logger().info(f'UAV exploration path_csv: {self.path_csv}')
        self.get_logger().info(f'Loaded UAV scan waypoints: {len(self.scan_waypoints)}')
        if self.scan_waypoints:
            first = self.scan_waypoints[0]
            self.get_logger().info(
                'First scan waypoint: '
                f'x={first.x}, y={first.y}, z={first.z}, '
                f'yaw_deg={first.yaw_deg}, '
                f'gimbal_pitch_deg={first.gimbal_pitch_deg}, tag={first.tag}')
        self.get_logger().info(f'command_pose_topic: {self.command_pose_topic}')

    def declare_params(self):
        self.declare_parameter('path_csv', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'x500_gimbal_0/base_link')
        self.declare_parameter('command_pose_topic', '/command/pose')
        self.declare_parameter('gimbal_pitch_topic', '/gimbal_pitch_degree')
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('exploration_start_topic', '/uav/exploration_start')
        self.declare_parameter('exploration_state_topic', '/uav/exploration_state')
        self.declare_parameter('exploration_event_topic', '/uav/exploration_event')
        self.declare_parameter('exploration_complete_topic', '/mission/uav_exploration_complete')
        self.declare_parameter('marker_detections_topic', '/perception/uav/marker_detections')
        self.declare_parameter('start_on_launch', False)
        self.declare_parameter('start_on_mission_state', True)
        self.declare_parameter('required_start_state', 'UAV_EXPLORATION_READY')
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
        self.declare_parameter('safe_takeoff_relative_height', 5.0)
        self.declare_parameter('safe_altitude', 18.0)
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
        self.exploration_start_topic = self.get_parameter('exploration_start_topic').value
        self.exploration_state_topic = self.get_parameter('exploration_state_topic').value
        self.exploration_event_topic = self.get_parameter('exploration_event_topic').value
        self.exploration_complete_topic = self.get_parameter('exploration_complete_topic').value
        self.marker_detections_topic = self.get_parameter('marker_detections_topic').value
        self.start_on_launch = self.get_parameter('start_on_launch').value
        self.start_on_mission_state = self.get_parameter('start_on_mission_state').value
        self.required_start_state = self.get_parameter('required_start_state').value
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
        self.safe_takeoff_relative_height = float(
            self.get_parameter('safe_takeoff_relative_height').value)
        self.safe_altitude = float(self.get_parameter('safe_altitude').value)
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

    def mission_state_cb(self, msg: String):
        if (
            self.start_on_mission_state
            and msg.data == self.required_start_state
            and self.state in (ExplorationState.IDLE, ExplorationState.READY, ExplorationState.WAITING_FOR_TF)
        ):
            self.start_exploration(f'mission_state:{msg.data}')

    def exploration_start_cb(self, msg: Bool):
        if msg.data and self.state in (ExplorationState.IDLE, ExplorationState.READY, ExplorationState.WAITING_FOR_TF):
            self.start_exploration('manual_start')

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
        start_tf = self.lookup_current_tf()
        if start_tf is None:
            self.state = ExplorationState.WAITING_FOR_TF
            self.publish_event('WAITING_FOR_TF:START_POSE')
            return

        start_x = start_tf.transform.translation.x
        start_y = start_tf.transform.translation.y
        start_z = start_tf.transform.translation.z
        self.get_logger().info(
            f'Current UAV start pose from TF: x={start_x}, y={start_y}, z={start_z}')
        self.get_logger().info(f'Dynamic safe prefix enabled: {self.dynamic_safe_prefix}')
        self.get_logger().info(f'Original scan waypoint count: {len(self.scan_waypoints)}')

        if self.dynamic_safe_prefix:
            self.waypoints = self.create_dynamic_safe_path(start_x, start_y, start_z)
            self.publish_event('DYNAMIC_SAFE_PREFIX_CREATED')
        else:
            self.waypoints = list(self.scan_waypoints)

        if not self.waypoints:
            self.state = ExplorationState.ERROR
            self.publish_event('ERROR:NO_WAYPOINTS')
            return

        self.log_start_path_summary()
        self.current_index = 0
        self.hold_start = None
        self.exploration_start_time = self.get_clock().now()
        self.complete_published = False
        self.state = ExplorationState.EXPLORING
        self.get_logger().info(f'Exploration start reason: {reason}')
        self.publish_event('EXPLORATION_STARTED')

    def create_dynamic_safe_path(self, start_x: float, start_y: float, start_z: float) -> List[Waypoint]:
        first_scan = self.scan_waypoints[0]
        heading_to_scan = self.yaw_to_target_deg(start_x, start_y, first_scan.x, first_scan.y)
        safe_pitch = self.clamp(self.safe_prefix_gimbal_pitch_deg, -90.0, 20.0)
        safe_hold = max(0.0, self.safe_prefix_hold_sec)
        waypoints = [
            Waypoint(
                x=start_x,
                y=start_y,
                z=max(5.0, start_z + self.safe_takeoff_relative_height),
                yaw_deg=heading_to_scan,
                gimbal_pitch_deg=safe_pitch,
                hold_sec=safe_hold,
                tag='takeoff_climb',
            ),
            Waypoint(
                x=start_x,
                y=start_y,
                z=max(5.0, self.safe_altitude),
                yaw_deg=heading_to_scan,
                gimbal_pitch_deg=safe_pitch,
                hold_sec=safe_hold,
                tag='safe_altitude',
            ),
        ]

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
                    z=max(5.0, self.transition_altitude),
                    yaw_deg=self.yaw_to_target_deg(x, y, next_x, next_y),
                    gimbal_pitch_deg=safe_pitch,
                    hold_sec=safe_hold,
                    tag=f'transition_{index:03d}',
                ))

        waypoints.extend(self.scan_waypoints)
        return waypoints

    def log_start_path_summary(self):
        self.get_logger().info(f'Final waypoint count with safe prefix: {len(self.waypoints)}')
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
        if self.state == ExplorationState.ERROR:
            self.publish_state()
            return

        if self.state == ExplorationState.IDLE:
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

    def lookup_current_tf(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
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
        self.pose_pub.publish(pose)
        self.current_pose = pose

        gimbal_msg = Float32()
        gimbal_msg.data = float(waypoint.gimbal_pitch_deg)
        self.gimbal_pub.publish(gimbal_msg)

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

    def publish_state(self):
        msg = String()
        status = {
            'state': self.state.value,
            'current_index': self.current_index,
            'waypoints': len(self.waypoints),
            'scan_waypoints': len(self.scan_waypoints),
            'unique_markers': sorted(self.unique_marker_ids),
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
