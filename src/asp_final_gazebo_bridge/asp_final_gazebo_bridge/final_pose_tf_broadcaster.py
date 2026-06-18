import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def multiply_quaternion(lhs, rhs):
    lx, ly, lz, lw = lhs.x, lhs.y, lhs.z, lhs.w
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


class FinalPoseTfBroadcaster(Node):
    def __init__(self):
        super().__init__("asp_final_pose_tf_broadcaster")
        self.declare_parameters(
            "",
            [
                ("map_frame", "map"),
                ("ugv_base_frame", "X1_asp/base_link"),
                ("uav_base_frame", "x500_gimbal_0/base_link"),
                ("publish_rate_hz", 30.0),
            ],
        )
        self.map_frame = self.get_parameter("map_frame").value
        self.frames = {
            "ugv": self.get_parameter("ugv_base_frame").value,
            "uav": self.get_parameter("uav_base_frame").value,
        }
        self.last_transform = {"ugv": None, "uav": None}
        self.tf_broadcaster = TransformBroadcaster(self)
        self.camera_rdf_frame = "x500_gimbal_0/camera_link_rdf"
        self.camera_frame = "x500_gimbal_0/camera_link"
        self.camera_rdf_rotation = quaternion_from_rpy(-math.pi / 2.0, 0.0, math.pi / 2.0)

        qos = 10
        for topic in (
            "/model/X1_asp/pose",
            "/model/X1_asp/pose_static",
        ):
            self.create_subscription(TFMessage, topic, self.make_tf_callback("ugv", topic), qos)

        for topic in (
            "/model/x500_gimbal_0/pose",
            "/model/x500_gimbal_0/pose_static",
        ):
            self.create_subscription(TFMessage, topic, self.make_tf_callback("uav", topic), qos)

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self.publish_latest)
        self.get_logger().info(
            "Publishing Gazebo pose TFs as map->X1_asp/base_link and map->x500_gimbal_0/base_link"
        )

    def make_tf_callback(self, robot_key, topic):
        def callback(msg):
            matched = False
            for transform in msg.transforms:
                normalized = self.normalized_transform(transform)
                self.tf_broadcaster.sendTransform(normalized)
                self.publish_camera_rdf(normalized)
                if self.is_robot_transform(robot_key, normalized):
                    self.last_transform[robot_key] = normalized
                    self.publish_alias(robot_key, normalized)
                    matched = True
            if matched:
                self.get_logger().debug(f"Updated {robot_key} TF from {topic}")

        return callback

    def publish_latest(self):
        for robot_key, transform in self.last_transform.items():
            if transform is not None:
                self.publish_alias(robot_key, transform)

    def normalized_transform(self, transform):
        normalized = TransformStamped()
        normalized.header = transform.header
        normalized.header.stamp = self.get_clock().now().to_msg()
        normalized.header.frame_id = self.map_frame if transform.header.frame_id in ("", "default", "world") else transform.header.frame_id
        normalized.child_frame_id = transform.child_frame_id
        normalized.transform = transform.transform
        return normalized

    def is_robot_transform(self, robot_key, transform):
        child = transform.child_frame_id
        if robot_key == "ugv":
            return child == self.frames["ugv"] or child == "X1_asp"
        return child == self.frames["uav"] or child == "x500_gimbal_0"

    def publish_alias(self, robot_key, source):
        alias = TransformStamped()
        alias.header.stamp = self.get_clock().now().to_msg()
        alias.header.frame_id = self.map_frame
        alias.child_frame_id = self.frames[robot_key]
        alias.transform = source.transform
        self.tf_broadcaster.sendTransform(alias)

    def publish_camera_rdf(self, source):
        if source.child_frame_id != self.camera_frame:
            return

        rdf = TransformStamped()
        rdf.header = source.header
        rdf.child_frame_id = self.camera_rdf_frame
        rdf.transform.translation = source.transform.translation
        qx, qy, qz, qw = multiply_quaternion(
            source.transform.rotation,
            self.camera_rdf_rotation,
        )
        rdf.transform.rotation.x = qx
        rdf.transform.rotation.y = qy
        rdf.transform.rotation.z = qz
        rdf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(rdf)


def main(args=None):
    rclpy.init(args=args)
    node = FinalPoseTfBroadcaster()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
