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
    MISSION2_UAV_EXPLORATION_AND_UGV_RENDEZVOUS = (
        'MISSION2_UAV_EXPLORATION_AND_UGV_RENDEZVOUS'
    )
    UAV_EXPLORATION_COMPLETE = 'UAV_EXPLORATION_COMPLETE'
    UGV_RENDEZVOUS_READY = 'UGV_RENDEZVOUS_READY'
    UGV_RENDEZVOUS_RUNNING = 'UGV_RENDEZVOUS_RUNNING'
    UGV_RENDEZVOUS_COMPLETE = 'UGV_RENDEZVOUS_COMPLETE'
    PRECISION_LANDING_READY = 'PRECISION_LANDING_READY'
    PRECISION_LANDING_RUNNING = 'PRECISION_LANDING_RUNNING'
    MISSION_COMPLETE = 'MISSION_COMPLETE'
    MISSION_ABORTED = 'MISSION_ABORTED'


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
                ('mission_abort_topic', '/mission/abort'),
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
                ('auto_start_exploration', True),
                ('start_uav_exploration_on_mission2_start', True),
                ('ugv_rendezvous_start_topic', '/ugv/rendezvous_start'),
                ('ugv_rendezvous_reached_topic', '/ugv/rendezvous_reached'),
                ('start_rendezvous_on_mission2_start', True),
                ('wait_both_uav_and_ugv_before_landing', True),
                ('precision_landing_start_topic', '/mission/precision_landing_start'),
                ('landing_complete_topic', '/status/landing_complete'),
                ('mission_complete_topic', '/mission/mission_complete'),
                ('required_mission2_event', 'MISSION2_START_REACHED'),
            ],
        )

        self.auto_start = self.get_parameter('auto_start').value
        self.publish_rate_hz = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self.required_mission2_event = self.get_parameter('required_mission2_event').value
        self.auto_publish_takeoff = self.get_parameter('auto_publish_takeoff').value
        self.auto_start_exploration = self.get_parameter('auto_start_exploration').value
        self.start_uav_exploration_on_mission2_start = self.get_parameter(
            'start_uav_exploration_on_mission2_start').value
        self.start_rendezvous_on_mission2_start = self.get_parameter(
            'start_rendezvous_on_mission2_start').value
        self.wait_both_uav_and_ugv_before_landing = self.get_parameter(
            'wait_both_uav_and_ugv_before_landing').value

        self.state = MissionState.INIT
        self.ugv_state = ''
        self.last_ugv_event = ''
        self.last_manager_event = ''
        self.perception_trigger_seen = False
        self.mission2_trigger_published = False
        self.takeoff_requested = False
        self.exploration_start_published = False
        self.uav_exploration_complete = False
        self.rendezvous_start_published = False
        self.rendezvous_reached = False
        self.parallel_mission_started = False
        self.precision_landing_start_published = False
        self.landing_complete = False
        self.mission_complete_published = False

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
        self.rendezvous_start_pub = self.create_publisher(
            Bool, self.get_parameter('ugv_rendezvous_start_topic').value, 10)
        self.precision_landing_start_pub = self.create_publisher(
            Bool, self.get_parameter('precision_landing_start_topic').value, 10)
        self.mission_complete_pub = self.create_publisher(
            Bool, self.get_parameter('mission_complete_topic').value, 10)

        self.create_subscription(
            Bool, self.get_parameter('mission_start_topic').value,
            self.mission_start_callback, 10)
        self.create_subscription(
            Bool, self.get_parameter('mission_reset_topic').value,
            self.mission_reset_callback, 10)
        self.create_subscription(
            Bool, self.get_parameter('mission_abort_topic').value,
            self.mission_abort_callback, 10)
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
        self.create_subscription(
            Bool, self.get_parameter('ugv_rendezvous_reached_topic').value,
            self.rendezvous_reached_callback, 10)
        self.create_subscription(
            Bool, self.get_parameter('landing_complete_topic').value,
            self.landing_complete_callback, 10)

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info('mission_manager_node started')

    def mission_start_callback(self, msg: Bool):
        if msg.data and self.state == MissionState.READY:
            self.transition_to(MissionState.MISSION1_RUNNING, 'mission/start true')

    def mission_reset_callback(self, msg: Bool):
        if not msg.data:
            return
        self.reset_runtime_flags()
        self.publish_bool(self.mission2_trigger_pub, False)
        self.publish_bool(self.mission_complete_pub, False)
        self.transition_to(MissionState.READY, 'mission/reset true')

    def mission_abort_callback(self, msg: Bool):
        if msg.data:
            self.transition_to(MissionState.MISSION_ABORTED, 'mission/abort true')

    def reset_runtime_flags(self):
        self.ugv_state = ''
        self.last_ugv_event = ''
        self.last_manager_event = ''
        self.perception_trigger_seen = False
        self.mission2_trigger_published = False
        self.takeoff_requested = False
        self.exploration_start_published = False
        self.uav_exploration_complete = False
        self.rendezvous_start_published = False
        self.rendezvous_reached = False
        self.parallel_mission_started = False
        self.precision_landing_start_published = False
        self.landing_complete = False
        self.mission_complete_published = False

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
        elif msg.data == 'RENDEZVOUS_REACHED':
            self.handle_rendezvous_complete('ugv mission event')
        else:
            self.get_logger().info(f'Unknown UGV mission event: {msg.data}')

    def perception_trigger_callback(self, msg: Bool):
        if msg.data:
            self.perception_trigger_seen = True
            self.get_logger().info(
                'Perception Mission2 trigger seen; FSM waits for UGV position event.')

    def uav_exploration_complete_callback(self, msg: Bool):
        if not msg.data or self.uav_exploration_complete:
            return

        self.uav_exploration_complete = True
        self.record_event(
            'UAV_EXPLORATION_COMPLETE_RECEIVED',
            'mission/uav_exploration_complete true')

        if (
            not self.start_rendezvous_on_mission2_start
            and not self.rendezvous_start_published
        ):
            self.transition_to(
                MissionState.UAV_EXPLORATION_COMPLETE,
                'mission/uav_exploration_complete true')
            self.start_rendezvous_once('UAV exploration complete')
            return

        if self.state != MissionState.MISSION2_UAV_EXPLORATION_AND_UGV_RENDEZVOUS:
            self.transition_to(
                MissionState.UAV_EXPLORATION_COMPLETE,
                'mission/uav_exploration_complete true')

        self.start_precision_landing_once('UAV exploration complete')

    def rendezvous_reached_callback(self, msg: Bool):
        if msg.data:
            self.handle_rendezvous_complete('/ugv/rendezvous_reached true')

    def landing_complete_callback(self, msg: Bool):
        if msg.data and not self.landing_complete:
            self.landing_complete = True
            self.transition_to(MissionState.MISSION_COMPLETE, 'landing_complete true')
            self.publish_mission_complete_once()

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
            self.publish_bool(self.takeoff_pub, True)
            self.takeoff_requested = True
            self.transition_to(MissionState.UAV_TAKEOFF_REQUESTED, 'auto_publish_takeoff')

        if (
            self.state == MissionState.UAV_EXPLORATION_READY
            and self.auto_start_exploration
        ):
            self.start_exploration_once()

        self.publish_state()
        self.publish_status()

    def handle_mission2_event(self):
        self.transition_to(MissionState.MISSION2_TRIGGERED, self.required_mission2_event)
        if not self.mission2_trigger_published:
            self.publish_bool(self.mission2_trigger_pub, True)
            self.mission2_trigger_published = True
            self.get_logger().warn('Mission2 trigger published from UGV position event')

        if (
            self.start_uav_exploration_on_mission2_start
            and self.start_rendezvous_on_mission2_start
        ):
            self.get_logger().info(
                'MISSION2_START_REACHED: starting UAV exploration and UGV rendezvous in parallel.')
            self.parallel_mission_started = True
            self.transition_to(
                MissionState.MISSION2_UAV_EXPLORATION_AND_UGV_RENDEZVOUS,
                'PARALLEL_MISSION2_3_STARTED')
            self.publish_exploration_start_once(self.required_mission2_event)
            self.publish_rendezvous_start_once(self.required_mission2_event)
            self.record_event(
                'PARALLEL_MISSION2_3_STARTED',
                'UAV exploration and UGV rendezvous launched from Mission2 start.')
            return

        self.transition_to(MissionState.UAV_TAKEOFF_READY, 'mission2 trigger complete')
        if self.start_rendezvous_on_mission2_start:
            self.publish_rendezvous_start_once(self.required_mission2_event)
        if self.start_uav_exploration_on_mission2_start:
            if self.publish_exploration_start_once(self.required_mission2_event):
                self.transition_to(
                    MissionState.UAV_EXPLORATION_RUNNING,
                    'start UAV exploration on Mission2 start')
            return
        if not self.auto_publish_takeoff:
            self.transition_to(MissionState.UAV_EXPLORATION_READY, 'mission2 exploration gate')
            if self.auto_start_exploration:
                self.start_exploration_once()

    def start_exploration_once(self):
        if self.publish_exploration_start_once('start UAV exploration'):
            self.transition_to(MissionState.UAV_EXPLORATION_RUNNING, 'start UAV exploration')

    def publish_exploration_start_once(self, reason: str) -> bool:
        if self.exploration_start_published:
            return False
        self.publish_bool(self.exploration_start_pub, True)
        self.exploration_start_published = True
        self.record_event('UAV_EXPLORATION_START_PUBLISHED', reason)
        return True

    def start_rendezvous_once(self, reason: str = 'UAV exploration complete'):
        if self.rendezvous_start_published:
            return
        self.transition_to(MissionState.UGV_RENDEZVOUS_READY, reason)
        if self.publish_rendezvous_start_once(reason):
            self.transition_to(MissionState.UGV_RENDEZVOUS_RUNNING, 'start UGV rendezvous')

    def publish_rendezvous_start_once(self, reason: str) -> bool:
        if self.rendezvous_start_published:
            return False
        self.publish_bool(self.rendezvous_start_pub, True)
        self.rendezvous_start_published = True
        self.record_event('UGV_RENDEZVOUS_START_PUBLISHED', reason)
        return True

    def handle_rendezvous_complete(self, reason: str):
        if self.rendezvous_reached:
            return
        self.rendezvous_reached = True
        self.record_event('UGV_RENDEZVOUS_REACHED', reason)
        if (
            self.state != MissionState.MISSION2_UAV_EXPLORATION_AND_UGV_RENDEZVOUS
            or self.uav_exploration_complete
        ):
            self.transition_to(MissionState.UGV_RENDEZVOUS_COMPLETE, reason)
        self.start_precision_landing_once()

    def start_precision_landing_once(self, reason: str = 'rendezvous complete'):
        if self.precision_landing_start_published:
            return
        if (
            self.wait_both_uav_and_ugv_before_landing
            and not (self.uav_exploration_complete and self.rendezvous_reached)
        ):
            self.record_event('PRECISION_LANDING_WAITING_FOR_UAV_AND_UGV', reason)
            return

        if self.state == MissionState.MISSION2_UAV_EXPLORATION_AND_UGV_RENDEZVOUS:
            self.transition_to(
                MissionState.UGV_RENDEZVOUS_COMPLETE,
                'UAV exploration and UGV rendezvous complete')

        self.transition_to(MissionState.PRECISION_LANDING_READY, reason)
        self.publish_bool(self.precision_landing_start_pub, True)
        self.precision_landing_start_published = True
        self.record_event('PRECISION_LANDING_START_PUBLISHED', reason)
        self.transition_to(MissionState.PRECISION_LANDING_RUNNING, 'start precision landing')

    def publish_mission_complete_once(self):
        if self.mission_complete_published:
            return
        self.publish_bool(self.mission_complete_pub, True)
        self.mission_complete_published = True

    def transition_to(self, next_state: MissionState, reason: str):
        if self.state == next_state:
            return
        previous = self.state
        self.state = next_state
        self.get_logger().info(
            f'State transition: {previous.value} -> {next_state.value} ({reason})')

    def record_event(self, event: str, reason: str = ''):
        self.last_manager_event = event
        if reason:
            self.get_logger().info(f'{event}: {reason}')
        else:
            self.get_logger().info(event)

    def publish_bool(self, publisher, value: bool):
        msg = Bool()
        msg.data = value
        publisher.publish(msg)

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
            'last_manager_event': self.last_manager_event,
            'mission2_trigger_published': self.mission2_trigger_published,
            'perception_trigger_seen': self.perception_trigger_seen,
            'takeoff_requested': self.takeoff_requested,
            'exploration_start_published': self.exploration_start_published,
            'uav_exploration_complete': self.uav_exploration_complete,
            'rendezvous_start_published': self.rendezvous_start_published,
            'rendezvous_reached': self.rendezvous_reached,
            'parallel_mission_started': self.parallel_mission_started,
            'precision_landing_start_published': self.precision_landing_start_published,
            'landing_complete': self.landing_complete,
            'mission_complete_published': self.mission_complete_published,
            'start_uav_exploration_on_mission2_start': (
                self.start_uav_exploration_on_mission2_start),
            'start_rendezvous_on_mission2_start': self.start_rendezvous_on_mission2_start,
            'wait_both_uav_and_ugv_before_landing': (
                self.wait_both_uav_and_ugv_before_landing),
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
