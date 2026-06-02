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

        self.waypoints: List[Waypoint] = self.load_path(self.path_csv)
        self.state = ExplorationState.IDLE if self.waypoints else ExplorationState.ERROR
        self.current_index = 0
        self.hold_start = None
        self.exploration_start_time = None
        self.complete_published = False
        self.unique_marker_ids: Set[int] = set()
        self.current_pose: Optional[PoseStamped] = None

        period = 1.0 / max(1.0, self.pose_publish_rate_hz)
        self.timer = self.create_timer(period, self.timer_cb)

        if self.start_on_launch and self.waypoints:
            self.start_exploration('start_on_launch')

        self.get_logger().info(f'uav_exploration_node loaded {len(self.waypoints)} waypoints')
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
            and self.state in (ExplorationState.IDLE, ExplorationState.READY)
        ):
            self.start_exploration(f'mission_state:{msg.data}')

    def exploration_start_cb(self, msg: Bool):
        if msg.data and self.state in (ExplorationState.IDLE, ExplorationState.READY):
            self.start_exploration('manual_start')

    def marker_detections_cb(self, msg: String):
        for token in re.findall(r'-?\d+', msg.data):
            marker_id = int(token)
            if marker_id not in self.unique_marker_ids:
                self.unique_marker_ids.add(marker_id)
                self.get_logger().info(f'Unique UAV marker recorded: {marker_id}')

    def start_exploration(self, reason: str):
        if not self.waypoints:
            self.state = ExplorationState.ERROR
            self.publish_event('ERROR:NO_WAYPOINTS')
            return
        self.current_index = 0
        self.hold_start = None
        self.exploration_start_time = self.get_clock().now()
        self.complete_published = False
        self.state = ExplorationState.EXPLORING
        self.publish_event(f'EXPLORATION_STARTED:{reason}')

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
            self.publish_current_target_pose()
            self.publish_state()
            return

        if self.state == ExplorationState.WAITING_FOR_TF:
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
