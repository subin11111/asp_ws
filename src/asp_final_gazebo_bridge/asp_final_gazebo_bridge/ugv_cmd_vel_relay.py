import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class UgvCmdVelRelay(Node):
    def __init__(self):
        super().__init__("asp_final_ugv_cmd_vel_relay")
        self.pub = self.create_publisher(Twist, "/model/X1_asp/cmd_vel", 10)
        self.create_subscription(Twist, "/asp_final/ugv/cmd_vel", self.pub.publish, 10)
        self.get_logger().info("Relaying /asp_final/ugv/cmd_vel to /model/X1_asp/cmd_vel")


def main(args=None):
    rclpy.init(args=args)
    node = UgvCmdVelRelay()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
