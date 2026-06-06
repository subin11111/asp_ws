import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
import tf2_geometry_msgs  # noqa: F401


DICTIONARIES = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
}


def parse_allowed_marker_ids(value):
    if value is None:
        return set()
    if isinstance(value, str):
        tokens = [token.strip() for token in value.split(",")]
    else:
        tokens = [str(token).strip() for token in value]
    allowed = set()
    for token in tokens:
        if not token:
            continue
        try:
            allowed.add(int(token))
        except ValueError:
            continue
    return allowed


def quaternion_from_rotation_matrix(matrix):
    trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0, 1] + matrix[1, 0]) / scale
        qz = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / scale
        qx = (matrix[0, 1] + matrix[1, 0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / scale
        qx = (matrix[0, 2] + matrix[2, 0]) / scale
        qy = (matrix[1, 2] + matrix[2, 1]) / scale
        qz = 0.25 * scale
    return qx, qy, qz, qw


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
                ("use_image_stamp_for_tf", False),
                ("allowed_marker_ids", ""),
            ],
        )
        self.mode = self.get_parameter("mode").value
        self.map_frame = self.get_parameter("map_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.allowed_marker_ids = parse_allowed_marker_ids(self.get_parameter("allowed_marker_ids").value)
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
        self.detections_pub = self.create_publisher(Detection3DArray, f"{prefix}/marker_detections", 10)
        self.id_pub = self.create_publisher(Int32, f"{prefix}/marker_id", 10)
        self.annotated_pub = self.create_publisher(Image, "/asp_final/perception/uav/aruco/annotated", qos)
        allowed = sorted(self.allowed_marker_ids) if self.allowed_marker_ids else "all"
        self.get_logger().info(f"asp_final {self.mode} ArUco detector ready; allowed_marker_ids={allowed}")

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

    def transform_to_map(self, rvec, tvec, stamp):
        if tvec is None or not all(math.isfinite(float(value)) for value in tvec):
            return None
        pose = PoseStamped()
        pose.header.stamp = stamp if bool(self.get_parameter("use_image_stamp_for_tf").value) else rclpy.time.Time().to_msg()
        pose.header.frame_id = self.camera_frame
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])
        if rvec is not None and all(math.isfinite(float(value)) for value in rvec):
            rotation_matrix, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float64).reshape(3, 1))
            qx, qy, qz, qw = quaternion_from_rotation_matrix(rotation_matrix)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
        else:
            pose.pose.orientation.w = 1.0
        try:
            return self.tf_buffer.transform(pose, self.map_frame, timeout=Duration(seconds=0.1))
        except TransformException as exc:
            self.get_logger().warn(f"marker map transform failed: {exc}", throttle_duration_sec=2.0)
            return None

    def detection_msg(self, marker_id, marker_pose):
        detection = Detection3D()
        detection.header = marker_pose.header
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = str(int(marker_id))
        result.hypothesis.score = 1.0
        result.pose.pose = marker_pose.pose
        detection.results.append(result)
        return detection

    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}", throttle_duration_sec=2.0)
            return

        corners, ids, _ = self.detect(image)
        detections_msg = Detection3DArray()
        detections_msg.header.stamp = msg.header.stamp
        detections_msg.header.frame_id = self.map_frame
        if ids is not None and len(ids) > 0:
            for marker_id, marker_corners in zip(ids.flatten().tolist(), corners):
                if self.allowed_marker_ids and int(marker_id) not in self.allowed_marker_ids:
                    continue
                pose = self.estimate_pose(marker_corners)
                if pose:
                    rvec, tvec = pose
                    map_pose = self.transform_to_map(rvec, tvec, msg.header.stamp)
                    if map_pose is not None:
                        detections_msg.detections.append(self.detection_msg(marker_id, map_pose))
                id_msg = Int32()
                id_msg.data = int(marker_id)
                self.id_pub.publish(id_msg)
            cv2.aruco.drawDetectedMarkers(image, corners, ids)

        if detections_msg.detections:
            self.detections_pub.publish(detections_msg)
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
