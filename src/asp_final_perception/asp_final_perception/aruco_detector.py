import json
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401


DICTIONARIES = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
}


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("asp_final_aruco_detector")
        self.declare_parameters(
            "",
            [
                ("mode", "uav"),
                ("image_topic", "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image"),
                ("camera_info_topic", "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info"),
                ("map_frame", "map"),
                ("camera_frame", "x500_gimbal_0/camera_link"),
                ("dictionary", "DICT_4X4_50"),
                ("marker_size_m", 1.0),
            ],
        )
        self.mode = self.get_parameter("mode").value
        self.map_frame = self.get_parameter("map_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        prefix = "/asp_final/perception/landing" if self.mode == "landing" else "/asp_final/perception/uav"
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        dictionary_name = self.get_parameter("dictionary").value
        self.dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARIES.get(dictionary_name, cv2.aruco.DICT_4X4_50))
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params) if hasattr(cv2.aruco, "ArucoDetector") else None

        qos = rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.get_parameter("image_topic").value, self.on_image, qos)
        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value, self.on_camera_info, qos)
        self.detections_pub = self.create_publisher(String, f"{prefix}/marker_detections", 10)
        self.id_pub = self.create_publisher(Int32, f"{prefix}/marker_id", 10)
        self.annotated_pub = self.create_publisher(Image, "/asp_final/perception/uav/aruco/annotated", qos)
        self.get_logger().info(f"asp_final {self.mode} ArUco detector ready")

    def on_camera_info(self, msg):
        if self.camera_matrix is not None:
            return
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def detect(self, image):
        if self.detector:
            return self.detector.detectMarkers(image)
        return cv2.aruco.detectMarkers(image, self.dictionary, parameters=self.params)

    def estimate_pose(self, corners):
        if self.camera_matrix is None:
            return None
        marker_size = float(self.get_parameter("marker_size_m").value)
        object_points = np.array(
            [
                [-marker_size / 2, marker_size / 2, 0.0],
                [marker_size / 2, marker_size / 2, 0.0],
                [marker_size / 2, -marker_size / 2, 0.0],
                [-marker_size / 2, -marker_size / 2, 0.0],
            ],
            dtype=np.float32,
        )
        ok, rvec, tvec = cv2.solvePnP(object_points, corners.reshape(4, 2), self.camera_matrix, self.dist_coeffs)
        if not ok:
            return None
        return rvec.reshape(3).tolist(), tvec.reshape(3).tolist()

    def transform_to_map(self, tvec):
        if tvec is None or not all(math.isfinite(float(value)) for value in tvec):
            return None
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = self.camera_frame
        point.point.x = float(tvec[0])
        point.point.y = float(tvec[1])
        point.point.z = float(tvec[2])
        try:
            return self.tf_buffer.transform(point, self.map_frame, timeout=Duration(seconds=0.05))
        except TransformException as exc:
            self.get_logger().warn(f"marker map transform failed: {exc}", throttle_duration_sec=2.0)
            return None

    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}", throttle_duration_sec=2.0)
            return

        corners, ids, _ = self.detect(image)
        detections = []
        if ids is not None and len(ids) > 0:
            for marker_id, marker_corners in zip(ids.flatten().tolist(), corners):
                pose = self.estimate_pose(marker_corners)
                detection = {"id": int(marker_id), "marker_id": int(marker_id)}
                if pose:
                    _, tvec = pose
                    detection.update(
                        {
                            "camera_x": float(tvec[0]),
                            "camera_y": float(tvec[1]),
                            "camera_z": float(tvec[2]),
                        }
                    )
                    map_point = self.transform_to_map(tvec)
                    if map_point is not None:
                        detection.update(
                            {
                                "map_x": float(map_point.point.x),
                                "map_y": float(map_point.point.y),
                                "map_z": float(map_point.point.z),
                                "has_map": True,
                            }
                        )
                    else:
                        detection["has_map"] = False
                detections.append(detection)
                id_msg = Int32()
                id_msg.data = int(marker_id)
                self.id_pub.publish(id_msg)
            cv2.aruco.drawDetectedMarkers(image, corners, ids)

        out = String()
        out.data = json.dumps(detections)
        self.detections_pub.publish(out)
        if self.mode == "uav":
            annotated = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            annotated.header = msg.header
            self.annotated_pub.publish(annotated)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
