import json
import math
from enum import Enum
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class LandingState(str, Enum):
    IDLE = 'IDLE'
    APPROACH_ABOVE_UGV = 'APPROACH_ABOVE_UGV'
    SEARCH_MARKER = 'SEARCH_MARKER'
    ALIGN_TO_MARKER = 'ALIGN_TO_MARKER'
    STEP_DESCEND = 'STEP_DESCEND'
    FINAL_LAND = 'FINAL_LAND'
    LANDING_COMPLETE = 'LANDING_COMPLETE'
    ABORTED = 'ABORTED'


class PrecisionLandingNode(Node):
    def __init__(self):
        super().__init__('precision_landing_node')
        self.declare_params()
        self.read_params()

        self.command_pose_pub = self.create_publisher(PoseStamped, self.command_pose_topic, 10)
        self.command_land_pub = self.create_publisher(Bool, self.command_land_topic, 10)
        self.complete_pub = self.create_publisher(Bool, self.landing_complete_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.event_pub = self.create_publisher(String, self.event_topic, 10)

        self.create_subscription(Bool, self.start_topic, self.start_cb, 10)
        self.create_subscription(String, self.marker_detections_topic, self.marker_cb, 10)
        self.create_subscription(String, self.mission_state_topic, self.mission_state_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = LandingState.IDLE
        self.active = False
        self.start_time = None
        self.step_hold_start = None
        self.target_z: Optional[float] = None
        self.last_marker = None
        self.last_marker_time = None
        self.last_pose = None
        self.mission_state = ''
        self.complete_published = False

        self.timer = self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('precision_landing_node started')

    def declare_params(self):
        self.declare_parameter('start_topic', '/mission/precision_landing_start')
        self.declare_parameter('marker_detections_topic', '/perception/uav/marker_detections')
        self.declare_parameter('command_pose_topic', '/command/pose')
        self.declare_parameter('command_land_topic', '/command/land')
        self.declare_parameter('landing_complete_topic', '/status/landing_complete')
        self.declare_parameter('state_topic', '/precision_landing/state')
        self.declare_parameter('event_topic', '/precision_landing/event')
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('uav_base_frame', 'x500_gimbal_0/base_link')
        self.declare_parameter('ugv_base_frame', 'X1_asp/base_link')
        self.declare_parameter('landing_marker_id', -1)
        self.declare_parameter('require_marker_for_descent', True)
        self.declare_parameter('approach_altitude', 18.0)
        self.declare_parameter('align_altitude', 10.0)
        self.declare_parameter('descend_step_m', 1.0)
        self.declare_parameter('final_land_altitude', 3.0)
        self.declare_parameter('xy_tolerance_m', 0.6)
        self.declare_parameter('pixel_tolerance_px', 60.0)
        self.declare_parameter('marker_timeout_sec', 2.0)
        self.declare_parameter('max_lost_marker_time_sec', 5.0)
        self.declare_parameter('hold_sec_each_step', 1.5)
        self.declare_parameter('max_landing_time_sec', 180.0)

    def read_params(self):
        self.start_topic = self.get_parameter('start_topic').value
        self.marker_detections_topic = self.get_parameter('marker_detections_topic').value
        self.command_pose_topic = self.get_parameter('command_pose_topic').value
        self.command_land_topic = self.get_parameter('command_land_topic').value
        self.landing_complete_topic = self.get_parameter('landing_complete_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.event_topic = self.get_parameter('event_topic').value
        self.mission_state_topic = self.get_parameter('mission_state_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.uav_base_frame = self.get_parameter('uav_base_frame').value
        self.ugv_base_frame = self.get_parameter('ugv_base_frame').value
        self.landing_marker_id = int(self.get_parameter('landing_marker_id').value)
        self.require_marker_for_descent = self.get_parameter('require_marker_for_descent').value
        self.approach_altitude = float(self.get_parameter('approach_altitude').value)
        self.align_altitude = float(self.get_parameter('align_altitude').value)
        self.descend_step_m = float(self.get_parameter('descend_step_m').value)
        self.final_land_altitude = float(self.get_parameter('final_land_altitude').value)
        self.xy_tolerance_m = float(self.get_parameter('xy_tolerance_m').value)
        self.pixel_tolerance_px = float(self.get_parameter('pixel_tolerance_px').value)
        self.marker_timeout_sec = float(self.get_parameter('marker_timeout_sec').value)
        self.max_lost_marker_time_sec = float(
            self.get_parameter('max_lost_marker_time_sec').value)
        self.hold_sec_each_step = float(self.get_parameter('hold_sec_each_step').value)
        self.max_landing_time_sec = float(self.get_parameter('max_landing_time_sec').value)

    def start_cb(self, msg: Bool):
        if not msg.data or self.active:
            return
        self.active = True
        self.complete_published = False
        self.start_time = self.get_clock().now()
        self.step_hold_start = None
        self.target_z = None
        self.transition_to(LandingState.APPROACH_ABOVE_UGV, 'start true')

    def marker_cb(self, msg: String):
        try:
            detection = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        marker_id = detection.get('marker_id', detection.get('id'))
        try:
            marker_id = int(marker_id)
        except (TypeError, ValueError):
            return
        if self.landing_marker_id >= 0 and marker_id != self.landing_marker_id:
            return
        detection['marker_id'] = marker_id
        self.last_marker = detection
        self.last_marker_time = self.get_clock().now()

    def mission_state_cb(self, msg: String):
        self.mission_state = msg.data

    def timer_cb(self):
        if not self.active:
            self.publish_state()
            return
        if self.start_time is not None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > self.max_landing_time_sec:
                self.abort('LANDING_TIMEOUT')
                return

        uav_tf = self.lookup_tf(self.map_frame, self.uav_base_frame)
        ugv_tf = self.lookup_tf(self.map_frame, self.ugv_base_frame)
        if uav_tf is None:
            self.publish_state()
            return

        current_x = uav_tf.transform.translation.x
        current_y = uav_tf.transform.translation.y
        current_z = uav_tf.transform.translation.z

        if self.state == LandingState.APPROACH_ABOVE_UGV:
            if ugv_tf is None:
                self.publish_hold(current_x, current_y, current_z)
                self.publish_state()
                return
            target_x = ugv_tf.transform.translation.x
            target_y = ugv_tf.transform.translation.y
            self.publish_pose(target_x, target_y, self.approach_altitude)
            if (
                abs(current_z - self.approach_altitude) <= 1.0
                and math.hypot(target_x - current_x, target_y - current_y) <= 1.5
            ):
                self.transition_to(LandingState.SEARCH_MARKER, 'approach reached')

        elif self.state == LandingState.SEARCH_MARKER:
            if self.marker_is_fresh(require_map=False):
                if self.marker_has_map():
                    self.transition_to(LandingState.ALIGN_TO_MARKER, 'fresh marker with map')
                else:
                    self.publish_event('MARKER_SEEN_WITHOUT_MAP_HOLD')
                    self.publish_hold(current_x, current_y, current_z)
            else:
                self.publish_hold(current_x, current_y, current_z)
                if self.marker_lost_too_long():
                    self.publish_event('MARKER_SEARCHING')

        elif self.state == LandingState.ALIGN_TO_MARKER:
            if not self.marker_is_fresh(require_map=True):
                self.transition_to(LandingState.SEARCH_MARKER, 'marker lost during align')
                self.publish_hold(current_x, current_y, current_z)
            else:
                marker_x = float(self.last_marker['map_x'])
                marker_y = float(self.last_marker['map_y'])
                align_z = max(self.align_altitude, current_z)
                self.publish_pose(marker_x, marker_y, align_z)
                if math.hypot(marker_x - current_x, marker_y - current_y) <= self.xy_tolerance_m:
                    self.target_z = max(self.final_land_altitude, current_z - self.descend_step_m)
                    self.step_hold_start = self.get_clock().now()
                    self.transition_to(LandingState.STEP_DESCEND, 'aligned')

        elif self.state == LandingState.STEP_DESCEND:
            if self.require_marker_for_descent and not self.marker_is_fresh(require_map=True):
                self.transition_to(LandingState.SEARCH_MARKER, 'marker lost during descent')
                self.publish_hold(current_x, current_y, current_z)
            else:
                marker_x = float(self.last_marker['map_x'])
                marker_y = float(self.last_marker['map_y'])
                if self.target_z is None:
                    self.target_z = max(self.final_land_altitude, current_z - self.descend_step_m)
                    self.step_hold_start = self.get_clock().now()
                self.publish_pose(marker_x, marker_y, self.target_z)
                hold_elapsed = (
                    (self.get_clock().now() - self.step_hold_start).nanoseconds / 1e9
                    if self.step_hold_start is not None else 0.0
                )
                xy_error = math.hypot(marker_x - current_x, marker_y - current_y)
                if current_z <= self.final_land_altitude + 0.5 and xy_error <= self.xy_tolerance_m:
                    self.transition_to(LandingState.FINAL_LAND, 'final altitude reached')
                elif hold_elapsed >= self.hold_sec_each_step:
                    self.target_z = max(self.final_land_altitude, current_z - self.descend_step_m)
                    self.step_hold_start = self.get_clock().now()
                    self.publish_event(f'LANDING_DESCEND_STEP:z={self.target_z:.2f}')

        elif self.state == LandingState.FINAL_LAND:
            self.publish_land()
            self.publish_complete(True)
            self.transition_to(LandingState.LANDING_COMPLETE, 'land command published')

        elif self.state == LandingState.LANDING_COMPLETE:
            self.publish_complete(True)

        self.publish_state()

    def marker_is_fresh(self, require_map: bool) -> bool:
        if self.last_marker is None or self.last_marker_time is None:
            return False
        age = (self.get_clock().now() - self.last_marker_time).nanoseconds / 1e9
        if age > self.marker_timeout_sec:
            return False
        return self.marker_has_map() if require_map else True

    def marker_lost_too_long(self) -> bool:
        if self.last_marker_time is None:
            return True
        age = (self.get_clock().now() - self.last_marker_time).nanoseconds / 1e9
        return age > self.max_lost_marker_time_sec

    def marker_has_map(self) -> bool:
        if self.last_marker is None or not self.last_marker.get('has_map', False):
            return False
        for key in ('map_x', 'map_y', 'map_z'):
            value = self.last_marker.get(key)
            if value is None:
                return False
            try:
                if not math.isfinite(float(value)):
                    return False
            except (TypeError, ValueError):
                return False
        return True

    def lookup_tf(self, target_frame: str, source_frame: str):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except TransformException as exc:
            self.get_logger().warn(f'TF lookup failed: {exc}', throttle_duration_sec=2.0)
            return None

    def publish_hold(self, x: float, y: float, z: float):
        self.publish_pose(x, y, z)

    def publish_pose(self, x: float, y: float, z: float):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.command_pose_pub.publish(pose)
        self.last_pose = {'x': x, 'y': y, 'z': z}

    def publish_land(self):
        msg = Bool()
        msg.data = True
        self.command_land_pub.publish(msg)
        self.publish_event('COMMAND_LAND_TRUE')

    def publish_complete(self, value: bool):
        if self.complete_published and value:
            return
        msg = Bool()
        msg.data = value
        self.complete_pub.publish(msg)
        self.complete_published = self.complete_published or value

    def abort(self, reason: str):
        self.transition_to(LandingState.ABORTED, reason)
        self.publish_complete(False)
        self.active = False

    def transition_to(self, state: LandingState, reason: str):
        if self.state == state:
            return
        previous = self.state
        self.state = state
        self.publish_event(f'{previous.value}->{state.value}:{reason}')

    def publish_state(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.state.value,
            'active': self.active,
            'mission_state': self.mission_state,
            'last_pose': self.last_pose,
            'has_fresh_marker': self.marker_is_fresh(require_map=False),
            'has_map_marker': self.marker_is_fresh(require_map=True),
            'landing_marker_id': self.landing_marker_id,
        })
        self.state_pub.publish(msg)

    def publish_event(self, text: str):
        msg = String()
        msg.data = text
        self.event_pub.publish(msg)
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
