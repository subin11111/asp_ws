import json
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class MissionState(str, Enum):
    INIT = 'INIT'
    READY = 'READY'
    MISSION1_RUNNING = 'MISSION1_RUNNING'
    MISSION2_TRIGGERED = 'MISSION2_TRIGGERED'
    UAV_TAKEOFF_READY = 'UAV_TAKEOFF_READY'
    UAV_TAKEOFF_REQUESTED = 'UAV_TAKEOFF_REQUESTED'
    UAV_EXPLORATION_READY = 'UAV_EXPLORATION_READY'
    UAV_EXPLORATION_RUNNING = 'UAV_EXPLORATION_RUNNING'
    UAV_EXPLORATION_COMPLETE = 'UAV_EXPLORATION_COMPLETE'


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('auto_start', True),
                ('publish_rate_hz', 2.0),
                ('mission_start_topic', '/mission/start'),
                ('mission_reset_topic', '/mission/reset'),
                ('ugv_state_topic', '/ugv/state'),
                ('ugv_mission_event_topic', '/ugv/mission_event'),
                ('perception_trigger_topic', '/perception/mission2_trigger'),
                ('mission_state_topic', '/mission/state'),
                ('mission_status_topic', '/mission/status'),
                ('mission2_trigger_topic', '/mission/mission2_trigger'),
                ('command_takeoff_topic', '/command/takeoff'),
                ('auto_publish_takeoff', False),
                ('uav_exploration_start_topic', '/uav/exploration_start'),
                ('uav_exploration_complete_topic', '/mission/uav_exploration_complete'),
                ('auto_start_exploration', False),
                ('required_mission2_event', 'MISSION2_START_REACHED'),
            ],
        )

        self.auto_start = self.get_parameter('auto_start').value
        self.publish_rate_hz = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self.required_mission2_event = self.get_parameter('required_mission2_event').value
        self.auto_publish_takeoff = self.get_parameter('auto_publish_takeoff').value
        self.auto_start_exploration = self.get_parameter('auto_start_exploration').value

        self.state = MissionState.INIT
        self.ugv_state = ''
        self.last_ugv_event = ''
        self.mission2_trigger_published = False
        self.perception_trigger_seen = False
        self.takeoff_requested = False
        self.exploration_start_published = False
        self.uav_exploration_complete = False

        self.state_pub = self.create_publisher(
            String, self.get_parameter('mission_state_topic').value, 10)
        self.status_pub = self.create_publisher(
            String, self.get_parameter('mission_status_topic').value, 10)
        self.mission2_trigger_pub = self.create_publisher(
            Bool, self.get_parameter('mission2_trigger_topic').value, 10)
        self.takeoff_pub = self.create_publisher(
            Bool, self.get_parameter('command_takeoff_topic').value, 10)
        self.exploration_start_pub = self.create_publisher(
            Bool, self.get_parameter('uav_exploration_start_topic').value, 10)

        self.create_subscription(
            Bool, self.get_parameter('mission_start_topic').value,
            self.mission_start_callback, 10)
        self.create_subscription(
            Bool, self.get_parameter('mission_reset_topic').value,
            self.mission_reset_callback, 10)
        self.create_subscription(
            String, self.get_parameter('ugv_state_topic').value,
            self.ugv_state_callback, 10)
        self.create_subscription(
            String, self.get_parameter('ugv_mission_event_topic').value,
            self.ugv_mission_event_callback, 10)
        self.create_subscription(
            Bool, self.get_parameter('perception_trigger_topic').value,
            self.perception_trigger_callback, 10)
        self.create_subscription(
            Bool, self.get_parameter('uav_exploration_complete_topic').value,
            self.uav_exploration_complete_callback, 10)

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info('mission_manager_node started')

    def mission_start_callback(self, msg: Bool):
        if msg.data and self.state == MissionState.READY:
            self.transition_to(MissionState.MISSION1_RUNNING, 'mission/start true')

    def mission_reset_callback(self, msg: Bool):
        if not msg.data:
            return

        self.ugv_state = ''
        self.last_ugv_event = ''
        self.mission2_trigger_published = False
        self.perception_trigger_seen = False
        self.takeoff_requested = False
        self.exploration_start_published = False
        self.uav_exploration_complete = False
        self.publish_mission2_trigger(False)
        self.transition_to(MissionState.READY, 'mission/reset true')

    def ugv_state_callback(self, msg: String):
        self.ugv_state = msg.data

    def ugv_mission_event_callback(self, msg: String):
        self.last_ugv_event = msg.data
        if msg.data == self.required_mission2_event:
            if self.state in (MissionState.READY, MissionState.MISSION1_RUNNING):
                self.handle_mission2_event()
            else:
                self.get_logger().info(
                    f'Ignoring duplicate Mission2 event in state {self.state.value}')
        else:
            self.get_logger().info(f'Unknown UGV mission event: {msg.data}')

    def perception_trigger_callback(self, msg: Bool):
        if msg.data:
            self.perception_trigger_seen = True
            self.get_logger().info(
                'Perception Mission2 trigger seen; FSM waits for UGV position event.')

    def uav_exploration_complete_callback(self, msg: Bool):
        if msg.data:
            self.uav_exploration_complete = True
            self.transition_to(
                MissionState.UAV_EXPLORATION_COMPLETE,
                'mission/uav_exploration_complete true')

    def timer_callback(self):
        if self.state == MissionState.INIT:
            self.transition_to(MissionState.READY, 'initial timer')

        if self.state == MissionState.READY and self.auto_start:
            self.transition_to(MissionState.MISSION1_RUNNING, 'auto_start')

        if (
            self.state == MissionState.UAV_TAKEOFF_READY
            and self.auto_publish_takeoff
            and not self.takeoff_requested
        ):
            msg = Bool()
            msg.data = True
            self.takeoff_pub.publish(msg)
            self.takeoff_requested = True
            self.transition_to(MissionState.UAV_TAKEOFF_REQUESTED, 'auto_publish_takeoff')

        if (
            self.state == MissionState.UAV_EXPLORATION_READY
            and self.auto_start_exploration
            and not self.exploration_start_published
        ):
            msg = Bool()
            msg.data = True
            self.exploration_start_pub.publish(msg)
            self.exploration_start_published = True
            self.transition_to(MissionState.UAV_EXPLORATION_RUNNING, 'auto_start_exploration')

        self.publish_state()
        self.publish_status()

    def handle_mission2_event(self):
        self.transition_to(MissionState.MISSION2_TRIGGERED, self.required_mission2_event)
        if not self.mission2_trigger_published:
            self.publish_mission2_trigger(True)
            self.mission2_trigger_published = True
            self.get_logger().warn('Mission2 trigger published from UGV position event')

        self.publish_state()
        self.transition_to(MissionState.UAV_TAKEOFF_READY, 'mission2 trigger complete')
        if not self.auto_publish_takeoff:
            self.transition_to(MissionState.UAV_EXPLORATION_READY, 'manual takeoff/exploration gate')

    def transition_to(self, next_state: MissionState, reason: str):
        if self.state == next_state:
            return
        previous = self.state
        self.state = next_state
        self.get_logger().info(
            f'State transition: {previous.value} -> {next_state.value} ({reason})')

    def publish_mission2_trigger(self, value: bool):
        msg = Bool()
        msg.data = value
        self.mission2_trigger_pub.publish(msg)

    def publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.state.value,
            'ugv_state': self.ugv_state,
            'last_ugv_event': self.last_ugv_event,
            'mission2_trigger_published': self.mission2_trigger_published,
            'perception_trigger_seen': self.perception_trigger_seen,
            'exploration_start_published': self.exploration_start_published,
            'uav_exploration_complete': self.uav_exploration_complete,
        })
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
