from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class MissionState(str, Enum):
    IDLE = "IDLE"
    MISSION1_CARRIER = "MISSION1_CARRIER"
    MISSION2_3_PARALLEL = "MISSION2_3_PARALLEL"
    MISSION4_LANDING = "MISSION4_LANDING"
    COMPLETE = "COMPLETE"


class MissionSupervisor(Node):
    def __init__(self):
        super().__init__("asp_final_mission_supervisor")
        self.state = MissionState.IDLE
        self.mission1_done = False
        self.mission2_done = False
        self.rendezvous_done = False
        self.landing_done = False

        qos = 10
        self.state_pub = self.create_publisher(String, "/asp_final/mission/state", qos)
        self.status_pub = self.create_publisher(String, "/asp_final/mission/status", qos)
        self.complete_pub = self.create_publisher(Bool, "/asp_final/mission/complete", qos)
        self.m1_start_pub = self.create_publisher(Bool, "/asp_final/ugv/mission1_start", qos)
        self.m2_start_pub = self.create_publisher(Bool, "/asp_final/uav/mission2_start", qos)
        self.offboard_enable_pub = self.create_publisher(Bool, "/asp_final/uav/offboard_command_enable", qos)
        self.m3_start_pub = self.create_publisher(Bool, "/asp_final/ugv/rendezvous_start", qos)
        self.landing_start_pub = self.create_publisher(Bool, "/asp_final/landing/start", qos)

        self.create_subscription(Bool, "/asp_final/mission/start", self.on_start, qos)
        self.create_subscription(Bool, "/asp_final/mission/reset", self.on_reset, qos)
        self.create_subscription(Bool, "/asp_final/ugv/mission1_complete", self.on_mission1_done, qos)
        self.create_subscription(Bool, "/asp_final/uav/mission2_complete", self.on_mission2_done, qos)
        self.create_subscription(String, "/asp_final/uav/exploration_state", self.on_uav_state, qos)
        self.create_subscription(Bool, "/asp_final/ugv/rendezvous_reached", self.on_rendezvous_done, qos)
        self.create_subscription(String, "/asp_final/ugv/state", self.on_ugv_state, qos)
        self.create_subscription(Bool, "/asp_final/landing/complete", self.on_landing_done, qos)

        self.timer = self.create_timer(0.5, self.tick)
        self.get_logger().info("asp_final mission supervisor ready; publish true to /asp_final/mission/start")

    def publish_bool(self, pub, value=True):
        msg = Bool()
        msg.data = bool(value)
        pub.publish(msg)

    def publish_text(self, pub, text):
        msg = String()
        msg.data = text
        pub.publish(msg)

    def on_start(self, msg):
        if msg.data and self.state == MissionState.IDLE:
            self.state = MissionState.MISSION1_CARRIER
            self.publish_bool(self.offboard_enable_pub, False)
            self.publish_bool(self.m1_start_pub)
            self.publish_text(self.status_pub, "Mission1 carrier path started")

    def on_reset(self, msg):
        if not msg.data:
            return
        self.state = MissionState.IDLE
        self.mission1_done = False
        self.mission2_done = False
        self.rendezvous_done = False
        self.landing_done = False
        self.publish_bool(self.offboard_enable_pub, False)
        self.publish_bool(self.complete_pub, False)
        self.publish_text(self.status_pub, "Mission reset")

    def on_mission1_done(self, msg):
        if msg.data:
            self.mission1_done = True

    def on_mission2_done(self, msg):
        if msg.data:
            self.mission2_done = True

    def on_uav_state(self, msg):
        if msg.data in ("mission2_complete", "complete"):
            self.mission2_done = True

    def on_rendezvous_done(self, msg):
        if msg.data:
            self.rendezvous_done = True

    def on_ugv_state(self, msg):
        if msg.data == "MISSION3_COMPLETE":
            self.rendezvous_done = True

    def on_landing_done(self, msg):
        if msg.data:
            self.landing_done = True

    def tick(self):
        if self.state == MissionState.MISSION1_CARRIER and self.mission1_done:
            self.state = MissionState.MISSION2_3_PARALLEL
            self.publish_bool(self.offboard_enable_pub, True)
            self.publish_bool(self.m2_start_pub)
            self.publish_bool(self.m3_start_pub)
            self.publish_text(self.status_pub, "Mission2 UAV exploration and Mission3 UGV rendezvous started in parallel")

        if self.state == MissionState.MISSION2_3_PARALLEL and self.mission2_done and self.rendezvous_done:
            self.state = MissionState.MISSION4_LANDING
            self.publish_bool(self.landing_start_pub)
            self.publish_text(self.status_pub, "Mission4 precision landing started after UAV waypoints and UGV Mission3 completed")

        if self.state == MissionState.MISSION4_LANDING and self.landing_done:
            self.state = MissionState.COMPLETE
            self.publish_bool(self.complete_pub, True)
            self.publish_text(self.status_pub, "Final mission complete")

        if self.state == MissionState.COMPLETE:
            self.publish_bool(self.complete_pub, True)

        self.publish_text(self.state_pub, self.state.value)


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
