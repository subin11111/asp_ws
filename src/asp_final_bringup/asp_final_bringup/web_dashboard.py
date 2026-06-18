import argparse
import csv
import errno
import json
import math
import os
import socketserver
import threading
import time
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray

try:
    import cv2
except ImportError:  # pragma: no cover - runtime dependency in ROS env
    cv2 = None


MISSION_STEPS = [
    "IDLE",
    "MISSION1_CARRIER",
    "MISSION2_3_PARALLEL",
    "MISSION4_LANDING",
    "COMPLETE",
]

UAV_IMAGE_TOPIC = "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image"
UGV_IMAGE_TOPIC = "/world/default/model/X1_asp/link/base_link/sensor/camera_front/image"
ANNOTATED_IMAGE_TOPIC = "/asp_final/perception/uav/aruco/annotated"


def dashboard_root():
    return Path(get_package_share_directory("asp_final_bringup")) / "web"


def package_path(package, subdir, file_name):
    return Path(get_package_share_directory(package)) / subdir / file_name


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_dict(x=0.0, y=0.0, z=0.0, yaw=0.0):
    return {"x": float(x), "y": float(y), "z": float(z), "yaw": float(yaw)}


class DashboardState:
    def __init__(self):
        self.lock = threading.RLock()
        self.started_at = time.monotonic()
        self.mission_state = "IDLE"
        self.status_text = "Waiting for mission state"
        self.mission_complete = False
        self.uav_phase = "UNKNOWN"
        self.ugv_state = "UNKNOWN"
        self.landing_state = "UNKNOWN"
        self.landing_complete = False
        self.px4_status = {}
        self.marker_status = {
            marker_id: {
                "id": marker_id,
                "type": "landing" if marker_id == 10 else "mission",
                "detected": False,
            }
            for marker_id in range(11)
        }
        self.events = []
        self.poses = {
            "uav": pose_dict(),
            "ugv": pose_dict(z=0.15),
            "landingTarget": pose_dict(44.2, 23.1, 0.1),
        }
        self.velocities = {"uav": {"vx": 0.0, "vy": 0.0, "vz": 0.0, "speed": 0.0}, "ugv": {"speed": 0.0}}
        self.previous_poses = {}
        self.last_tf_time = 0.0
        self.last_detector_time = 0.0
        self.last_image_time = {"uav": 0.0, "landing": 0.0, "ugv": 0.0}
        self.jpeg = {"uav": None, "landing": None, "ugv": None}
        self.paths = self.load_paths()

    def load_paths(self):
        uav_waypoints = []
        ugv_waypoints = []
        try:
            with package_path("asp_final_uav", "path", "mission2_uav_waypoints.csv").open(newline="") as handle:
                for row in csv.reader(handle):
                    if not row or row[0].strip().startswith("#"):
                        continue
                    uav_waypoints.append(pose_dict(row[0], row[1], row[2]))
        except Exception:
            uav_waypoints = [pose_dict(8, 6, 9), pose_dict(18, 12, 10), pose_dict(38, 21, 9)]
        for file_name in ("mission1_carrier.csv", "mission3_rendezvous.csv"):
            try:
                with package_path("asp_final_ugv", "path", file_name).open(newline="") as handle:
                    for row in csv.reader(handle):
                        if not row or row[0].strip().startswith("#"):
                            continue
                        ugv_waypoints.append(pose_dict(row[0], row[1], 0.15))
            except Exception:
                continue
        if not ugv_waypoints:
            ugv_waypoints = [pose_dict(4, 4, 0.15), pose_dict(20, 12, 0.15), pose_dict(44, 23, 0.15)]
        return {
            "uavWaypoints": uav_waypoints,
            "ugvWaypoints": ugv_waypoints,
            "uavTrail": [],
            "ugvTrail": [],
        }

    def add_event(self, level, message):
        now = time.strftime("%H:%M:%S")
        event = {"time": now, "level": level, "message": str(message)}
        with self.lock:
            if self.events and self.events[0]["message"] == event["message"]:
                return
            self.events.insert(0, event)
            self.events = self.events[:12]

    def update_pose(self, key, pose):
        now = time.monotonic()
        previous = self.previous_poses.get(key)
        if previous:
            dt = max(1e-3, now - previous["time"])
            vx = (pose["x"] - previous["pose"]["x"]) / dt
            vy = (pose["y"] - previous["pose"]["y"]) / dt
            vz = (pose["z"] - previous["pose"]["z"]) / dt
            speed = math.sqrt(vx * vx + vy * vy)
            if key == "uav":
                self.velocities[key] = {"vx": vx, "vy": vy, "vz": vz, "speed": speed}
            else:
                self.velocities[key] = {"speed": speed}
        self.previous_poses[key] = {"pose": pose, "time": now}
        self.poses[key] = pose
        trail_key = "uavTrail" if key == "uav" else "ugvTrail"
        self.paths[trail_key].append(pose)
        self.paths[trail_key] = self.paths[trail_key][-80:]

    def update_marker(self, marker_id, pose, confidence=1.0):
        if marker_id not in self.marker_status:
            return
        now = time.monotonic()
        self.marker_status[marker_id].update(
            {
                "detected": True,
                "lastSeenAt": now,
                "confidence": float(confidence),
                "pose": pose,
            }
        )
        if marker_id == 10:
            self.poses["landingTarget"] = pose
        self.last_detector_time = now

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            state_index = max(0, MISSION_STEPS.index(self.mission_state)) if self.mission_state in MISSION_STEPS else 0
            progress = 100 if self.mission_complete else int((state_index / (len(MISSION_STEPS) - 1)) * 100)
            landing_target = self.poses["landingTarget"]
            uav_pose = self.poses["uav"]
            ugv_pose = self.poses["ugv"]
            landing_error = math.hypot(uav_pose["x"] - landing_target["x"], uav_pose["y"] - landing_target["y"])
            markers = []
            for marker in self.marker_status.values():
                item = dict(marker)
                if "lastSeenAt" in item:
                    item["lastSeenSecAgo"] = round(max(0.0, now - item.pop("lastSeenAt")), 1)
                markers.append(item)
            px4_offboard = bool(self.px4_status.get("px4_offboard", False))
            px4_armed = bool(self.px4_status.get("px4_armed", False))
            return {
                "mission": {
                    "state": self.mission_state,
                    "progress": progress,
                    "elapsedSec": int(now - self.started_at),
                    "simTimeSec": int(now - self.started_at),
                    "statusText": self.status_text,
                    "complete": self.mission_complete,
                },
                "uav": {
                    "pose": uav_pose,
                    "velocity": self.velocities["uav"],
                    "phase": self.uav_phase,
                    "armed": px4_armed,
                    "offboard": px4_offboard,
                    "targetPose": landing_target,
                    "landingTarget": landing_target,
                    "landingXYError": landing_error,
                    "markerUsable": self.marker_status[10].get("detected", False),
                    "altitude": uav_pose["z"],
                },
                "ugv": {
                    "pose": ugv_pose,
                    "linearSpeed": self.velocities["ugv"]["speed"],
                    "angularSpeed": 0.0,
                    "state": self.ugv_state,
                    "stopped": self.velocities["ugv"]["speed"] < 0.05,
                    "rendezvousDistance": math.hypot(ugv_pose["x"] - landing_target["x"], ugv_pose["y"] - landing_target["y"]),
                },
                "paths": self.paths,
                "markers": sorted(markers, key=lambda marker: marker["id"]),
                "health": {
                    "rosBridgeConnected": True,
                    "tfFresh": (now - self.last_tf_time) < 2.0,
                    "px4Connected": bool(self.px4_status),
                    "px4Offboard": px4_offboard,
                    "px4Armed": px4_armed,
                    "gazeboClockActive": True,
                    "uavCameraActive": (now - self.last_image_time["uav"]) < 2.0,
                    "ugvCameraActive": (now - self.last_image_time["ugv"]) < 2.0,
                    "markerDetectorActive": (now - self.last_detector_time) < 3.0,
                },
                "imageTopics": [
                    {
                        "name": UAV_IMAGE_TOPIC,
                        "label": "UAV front camera",
                        "status": "ArUco overlay",
                        "fps": 30,
                        "latencyMs": 0,
                        "src": "/stream/uav.mjpg",
                    },
                    {
                        "name": ANNOTATED_IMAGE_TOPIC,
                        "label": "Landing camera",
                        "status": "marker_10 lock",
                        "fps": 24,
                        "latencyMs": 0,
                        "src": "/stream/landing.mjpg",
                    },
                ],
                "events": list(self.events),
            }


class DashboardRosNode(Node):
    def __init__(self, state):
        super().__init__("asp_final_web_dashboard_bridge")
        self.state = state
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(String, "/asp_final/mission/state", self.on_mission_state, 10)
        self.create_subscription(String, "/asp_final/mission/status", self.on_mission_status, 10)
        self.create_subscription(Bool, "/asp_final/mission/complete", self.on_mission_complete, 10)
        self.create_subscription(String, "/asp_final/uav/exploration_state", self.on_uav_phase, 10)
        self.create_subscription(String, "/asp_final/landing/state", self.on_landing_state, 10)
        self.create_subscription(String, "/asp_final/landing/event", self.on_landing_event, 10)
        self.create_subscription(Bool, "/asp_final/landing/complete", self.on_landing_complete, 10)
        self.create_subscription(String, "/asp_final/ugv/state", self.on_ugv_state, 10)
        self.create_subscription(String, "/asp_final/ugv/event", self.on_ugv_event, 10)
        self.create_subscription(String, "/asp_final/px4/status", self.on_px4_status, 10)
        self.create_subscription(Int32, "/asp_final/perception/uav/marker_id", self.on_marker_id, 10)
        self.create_subscription(Detection3DArray, "/asp_final/perception/uav/marker_detections", self.on_detections, 10)
        self.create_subscription(Detection3DArray, "/asp_final/perception/landing/marker_detections", self.on_detections, 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, UAV_IMAGE_TOPIC, lambda msg: self.on_image(msg, "uav"), qos)
        self.create_subscription(Image, UGV_IMAGE_TOPIC, lambda msg: self.on_image(msg, "ugv"), qos)
        self.create_subscription(Image, ANNOTATED_IMAGE_TOPIC, lambda msg: self.on_image(msg, "landing"), qos)
        self.create_timer(0.1, self.update_tf)
        self.get_logger().info("ASP dashboard bridge ready: /dashboard.json and /stream/*.mjpg")

    def on_mission_state(self, msg):
        with self.state.lock:
            self.state.mission_state = msg.data

    def on_mission_status(self, msg):
        with self.state.lock:
            self.state.status_text = msg.data
        self.state.add_event("info", msg.data)

    def on_mission_complete(self, msg):
        with self.state.lock:
            self.state.mission_complete = bool(msg.data)
            if msg.data:
                self.state.mission_state = "COMPLETE"
        if msg.data:
            self.state.add_event("success", "mission_complete")

    def on_uav_phase(self, msg):
        with self.state.lock:
            self.state.uav_phase = msg.data

    def on_landing_state(self, msg):
        with self.state.lock:
            self.state.landing_state = msg.data

    def on_landing_complete(self, msg):
        with self.state.lock:
            self.state.landing_complete = bool(msg.data)
        if msg.data:
            self.state.add_event("success", "landing_complete")

    def on_landing_event(self, msg):
        self.state.add_event("success" if "complete" in msg.data or "detected" in msg.data else "info", msg.data)

    def on_ugv_state(self, msg):
        with self.state.lock:
            self.state.ugv_state = msg.data

    def on_ugv_event(self, msg):
        self.state.add_event("info", msg.data)

    def on_px4_status(self, msg):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            status = {}
        with self.state.lock:
            self.state.px4_status = status

    def on_marker_id(self, msg):
        marker_id = int(msg.data)
        with self.state.lock:
            marker = self.state.marker_status.get(marker_id)
            if marker:
                marker.update({"detected": True, "lastSeenAt": time.monotonic(), "confidence": 1.0})

    def on_detections(self, msg):
        with self.state.lock:
            for detection in msg.detections:
                if not detection.results:
                    continue
                result = detection.results[0]
                try:
                    marker_id = int(result.hypothesis.class_id)
                except ValueError:
                    continue
                pose = result.pose.pose
                self.state.update_marker(
                    marker_id,
                    pose_dict(pose.position.x, pose.position.y, pose.position.z, yaw_from_quaternion(pose.orientation)),
                    result.hypothesis.score,
                )

    def on_image(self, msg, key):
        if cv2 is None:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 72])
        except Exception as exc:
            self.get_logger().warn(f"dashboard image conversion failed: {exc}", throttle_duration_sec=2.0)
            return
        if not ok:
            return
        with self.state.lock:
            self.state.jpeg[key] = encoded.tobytes()
            self.state.last_image_time[key] = time.monotonic()
            if key == "uav" and self.state.jpeg.get("landing") is None:
                self.state.jpeg["landing"] = self.state.jpeg[key]
                self.state.last_image_time["landing"] = self.state.last_image_time[key]

    def update_tf(self):
        for key, frame in (("uav", "x500_gimbal_0/base_link"), ("ugv", "X1_asp/base_link")):
            try:
                transform = self.tf_buffer.lookup_transform("map", frame, rclpy.time.Time())
            except TransformException:
                continue
            t = transform.transform.translation
            r = transform.transform.rotation
            with self.state.lock:
                self.state.update_pose(key, pose_dict(t.x, t.y, t.z, yaw_from_quaternion(r)))
                self.state.last_tf_time = time.monotonic()
        try:
            transform = self.tf_buffer.lookup_transform("map", "X1_asp/aruco_marker_10_link", rclpy.time.Time())
            t = transform.transform.translation
            r = transform.transform.rotation
            with self.state.lock:
                self.state.poses["landingTarget"] = pose_dict(t.x, t.y, t.z, yaw_from_quaternion(r))
        except TransformException:
            pass


class DashboardRequestHandler(SimpleHTTPRequestHandler):
    state = None

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_GET(self):
        if self.path == "/dashboard.json":
            self.send_dashboard_json()
            return
        if self.path.startswith("/stream/"):
            key = self.path.rsplit("/", 1)[-1].split(".", 1)[0]
            self.send_mjpeg(key)
            return
        super().do_GET()

    def send_dashboard_json(self):
        body = json.dumps(self.state.snapshot(), separators=(",", ":")).encode()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_mjpeg(self, key):
        if key not in ("uav", "landing", "ugv"):
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        last_frame = None
        while True:
            with self.state.lock:
                frame = self.state.jpeg.get(key)
            if frame and frame is not last_frame:
                try:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ")
                    self.wfile.write(str(len(frame)).encode())
                    self.wfile.write(b"\r\n\r\n")
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    last_frame = frame
                except (BrokenPipeError, ConnectionResetError):
                    return
            time.sleep(0.05)


class DashboardTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


def spin_node(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main(args=None):
    parser = argparse.ArgumentParser(description="Serve the ASP final mission dashboard.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8088, type=int)
    parsed, _ = parser.parse_known_args(args)

    rclpy.init(args=[])
    state = DashboardState()
    node = DashboardRosNode(state)
    spin_thread = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spin_thread.start()

    os.chdir(dashboard_root())
    DashboardRequestHandler.state = state
    try:
        with DashboardTCPServer((parsed.host, parsed.port), DashboardRequestHandler) as httpd:
            print(f"ASP dashboard: http://{parsed.host}:{parsed.port}/", flush=True)
            try:
                httpd.serve_forever()
            except KeyboardInterrupt:
                print("ASP dashboard stopped.", flush=True)
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            print(f"ASP dashboard port is already in use: http://{parsed.host}:{parsed.port}/", flush=True)
            print("Use another --port value, or stop the existing dashboard process.", flush=True)
            return
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
