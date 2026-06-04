import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleControlMode,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, String


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Px4OffboardBridge(Node):
    MY_SYSID = 46
    MY_COMPID = 47
    TARGET_SYSID = 1
    TARGET_COMPID = 1

    def __init__(self):
        super().__init__("asp_final_px4_offboard_bridge")
        self.declare_parameters(
            "",
            [
                ("setpoint_rate_hz", 20.0),
                ("auto_arm", True),
                ("auto_offboard", True),
                ("preoffboard_setpoint_count", 20),
                ("enu_to_ned", True),
                ("fast_climb_velocity_feedforward_mps", 8.0),
                ("fast_climb_acceleration_feedforward_mps2", 4.0),
                ("fast_climb_error_threshold_m", 1.0),
                ("auto_disarm_after_landed", True),
                ("landed_disarm_delay_sec", 1.0),
            ],
        )
        self.last_pose = None
        self.gimbal_pitch = None
        self.offboard_sent = False
        self.arm_sent = False
        self.setpoint_counter = 0
        self.local_position = None
        self.vehicle_status = None
        self.vehicle_control_mode = None
        self.last_command_ack = None
        self.map_anchor = None
        self.px4_anchor = None
        self.last_offboard_request_ns = 0
        self.last_arm_request_ns = 0
        self.last_setpoint_log_ns = 0
        self.last_status_pub_ns = 0
        self.land_requested = False
        self.disarm_after_land_sent = False
        self.landed_since_ns = 0
        self.land_detected = None
        self.px4_offboard_enabled = False
        self.px4_armed_flag = False

        px4_pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        origin_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", px4_pub_qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", px4_pub_qos)
        self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", px4_pub_qos)
        self.status_pub = self.create_publisher(String, "/asp_final/px4/status", 10)
        self.create_subscription(PoseStamped, "/asp_final/uav/cmd_pose", self.on_cmd_pose, 10)
        self.create_subscription(Bool, "/asp_final/uav/land", self.on_land, 10)
        self.create_subscription(Float32, "/asp_final/uav/gimbal_pitch_deg", self.on_gimbal_pitch, 10)
        self.create_subscription(PoseStamped, "/asp_final/uav/mission2_takeoff_origin", self.on_takeoff_origin, origin_qos)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.on_local_position, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.on_vehicle_status, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.on_vehicle_status, qos_profile_sensor_data)
        self.create_subscription(VehicleControlMode, "/fmu/out/vehicle_control_mode", self.on_vehicle_control_mode, qos_profile_sensor_data)
        self.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected", self.on_land_detected, qos_profile_sensor_data)
        self.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack", self.on_vehicle_command_ack, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / float(self.get_parameter("setpoint_rate_hz").value), self.tick)
        self.get_logger().info("asp_final PX4 offboard bridge ready")

    def timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.command = int(command)
        msg.param1 = float(params.get("param1", 0.0))
        msg.param2 = float(params.get("param2", 0.0))
        msg.param3 = float(params.get("param3", 0.0))
        msg.param4 = float(params.get("param4", 0.0))
        msg.param5 = float(params.get("param5", 0.0))
        msg.param6 = float(params.get("param6", 0.0))
        msg.param7 = float(params.get("param7", 0.0))
        msg.target_system = self.TARGET_SYSID
        msg.target_component = self.TARGET_COMPID
        msg.source_system = self.MY_SYSID
        msg.source_component = self.MY_COMPID
        msg.from_external = True
        self.command_pub.publish(msg)
        self.publish_status()

    def on_cmd_pose(self, msg):
        self.last_pose = msg

    def on_local_position(self, msg):
        self.local_position = msg
        if self.map_anchor is not None and self.px4_anchor is None:
            self.set_px4_anchor_from_local_position()

    def on_vehicle_status(self, msg):
        self.vehicle_status = msg
        if msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.px4_armed_flag = True
        if msg.failsafe:
            self.get_logger().warn(
                f"PX4 vehicle_status failsafe=true nav_state={msg.nav_state} arming_state={msg.arming_state}",
                throttle_duration_sec=2.0,
            )
        self.publish_status(throttle=True)

    def on_vehicle_control_mode(self, msg):
        self.vehicle_control_mode = msg
        was = (self.px4_offboard_enabled, self.px4_armed_flag)
        self.px4_offboard_enabled = bool(msg.flag_control_offboard_enabled)
        self.px4_armed_flag = bool(msg.flag_armed)
        now = (self.px4_offboard_enabled, self.px4_armed_flag)
        if now != was:
            self.get_logger().info(
                f"PX4 control mode changed: offboard={self.px4_offboard_enabled} armed={self.px4_armed_flag}"
            )
        self.publish_status(throttle=True)

    def on_vehicle_command_ack(self, msg):
        self.last_command_ack = msg
        result_text = self.ack_result_text(msg.result)
        if msg.result in (
            VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED,
        ):
            self.get_logger().warn(
                f"PX4 rejected/failed command={msg.command} result={msg.result}({result_text}) "
                f"result_param1={msg.result_param1} result_param2={msg.result_param2}",
                throttle_duration_sec=2.0,
            )
        else:
            self.get_logger().info(
                f"PX4 command ack command={msg.command} result={msg.result}({result_text})",
                throttle_duration_sec=2.0,
            )
        self.publish_status()

    def on_gimbal_pitch(self, msg):
        self.gimbal_pitch = msg.data
        self.vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL,
            param1=1.0,
            param7=float(msg.data),
        )

    def on_takeoff_origin(self, msg):
        self.map_anchor = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )
        self.set_px4_anchor_from_local_position()
        self.get_logger().info("Received Mission2 takeoff origin")

    def on_land(self, msg):
        if msg.data:
            self.land_requested = True
            self.vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def on_land_detected(self, msg):
        self.land_detected = msg
        if msg.landed:
            if self.landed_since_ns == 0:
                self.landed_since_ns = self.get_clock().now().nanoseconds
        else:
            self.landed_since_ns = 0

    def set_px4_anchor_from_local_position(self):
        if self.local_position is None:
            return False
        if not (self.local_position.xy_valid and self.local_position.z_valid):
            self.get_logger().warn(
                "Waiting for valid /fmu/out/vehicle_local_position before latching PX4 anchor",
                throttle_duration_sec=2.0,
            )
            return False
        self.px4_anchor = (
            float(self.local_position.x),
            float(self.local_position.y),
            float(self.local_position.z),
        )
        self.get_logger().info(
            f"Latched PX4 local NED anchor: x={self.px4_anchor[0]:.2f} "
            f"y={self.px4_anchor[1]:.2f} z={self.px4_anchor[2]:.2f}"
        )
        self.publish_status()
        return True

    def map_pose_to_px4_ned(self, pose):
        target_map = (float(pose.position.x), float(pose.position.y), float(pose.position.z))
        if self.map_anchor is not None and self.px4_anchor is not None:
            dx = target_map[0] - self.map_anchor[0]
            dy = target_map[1] - self.map_anchor[1]
            dz = target_map[2] - self.map_anchor[2]
            if self.get_parameter("enu_to_ned").value:
                return [self.px4_anchor[0] + dy, self.px4_anchor[1] + dx, self.px4_anchor[2] - dz]
            return [self.px4_anchor[0] + dx, self.px4_anchor[1] + dy, self.px4_anchor[2] + dz]

        if self.get_parameter("enu_to_ned").value:
            return [target_map[1], target_map[0], -target_map[2]]
        return list(target_map)

    def current_map_z(self):
        if self.local_position is None or self.map_anchor is None or self.px4_anchor is None:
            return None
        if self.get_parameter("enu_to_ned").value:
            dz = self.px4_anchor[2] - float(self.local_position.z)
        else:
            dz = float(self.local_position.z) - self.px4_anchor[2]
        return self.map_anchor[2] + dz

    def climb_feedforward(self, pose, target_px4):
        velocity = [math.nan, math.nan, math.nan]
        acceleration = [math.nan, math.nan, math.nan]
        current_map_z = self.current_map_z()
        if current_map_z is None or self.local_position is None:
            return velocity, acceleration
        climb_error = float(pose.position.z) - current_map_z
        threshold = float(self.get_parameter("fast_climb_error_threshold_m").value)
        climb_speed = float(self.get_parameter("fast_climb_velocity_feedforward_mps").value)
        climb_accel = float(self.get_parameter("fast_climb_acceleration_feedforward_mps2").value)
        if climb_speed <= 0.0 or climb_error <= threshold:
            return velocity, acceleration
        z_error_px4 = target_px4[2] - float(self.local_position.z)
        direction = math.copysign(1.0, z_error_px4)
        velocity[2] = direction * abs(climb_speed)
        if climb_accel > 0.0:
            acceleration[2] = direction * abs(climb_accel)
        return velocity, acceleration

    def yaw_enu_to_ned(self, yaw_enu):
        if not self.get_parameter("enu_to_ned").value:
            return yaw_enu
        return math.atan2(math.sin((math.pi / 2.0) - yaw_enu), math.cos((math.pi / 2.0) - yaw_enu))

    def px4_ready_for_position_setpoint(self):
        if self.map_anchor is None:
            self.get_logger().warn("Waiting for Mission2 map anchor before PX4 setpoints", throttle_duration_sec=2.0)
            return False
        if self.px4_anchor is None:
            self.get_logger().warn("Waiting for /fmu/out/vehicle_local_position before PX4 setpoints", throttle_duration_sec=2.0)
            return False
        return True

    def px4_in_offboard(self):
        if self.vehicle_control_mode is not None:
            return self.px4_offboard_enabled
        return bool(self.vehicle_status and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def px4_armed(self):
        if self.vehicle_control_mode is not None:
            return self.px4_armed_flag
        return bool(self.vehicle_status and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def ack_result_text(self, result):
        names = {
            VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED: "accepted",
            VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: "temporarily_rejected",
            VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: "denied",
            VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: "unsupported",
            VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: "failed",
            VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS: "in_progress",
            VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED: "cancelled",
        }
        return names.get(int(result), "unknown")

    def should_retry(self, last_request_ns):
        now_ns = self.get_clock().now().nanoseconds
        return last_request_ns == 0 or (now_ns - last_request_ns) > 1_000_000_000

    def publish_status(self, throttle=False):
        now_ns = self.get_clock().now().nanoseconds
        if throttle and (now_ns - self.last_status_pub_ns) < 1_000_000_000:
            return
        self.last_status_pub_ns = now_ns
        local_valid = bool(
            self.local_position
            and self.local_position.xy_valid
            and self.local_position.z_valid
        )
        status = {
            "has_cmd_pose": self.last_pose is not None,
            "has_map_anchor": self.map_anchor is not None,
            "has_px4_anchor": self.px4_anchor is not None,
            "local_position_valid": local_valid,
            "setpoint_counter": self.setpoint_counter,
            "px4_offboard": self.px4_in_offboard(),
            "px4_armed": self.px4_armed(),
            "land_requested": self.land_requested,
            "land_detected_landed": bool(self.land_detected and self.land_detected.landed),
            "land_detected_ground_contact": bool(self.land_detected and self.land_detected.ground_contact),
            "land_detected_at_rest": bool(self.land_detected and self.land_detected.at_rest),
            "disarm_after_land_sent": self.disarm_after_land_sent,
            "nav_state": int(self.vehicle_status.nav_state) if self.vehicle_status else None,
            "arming_state": int(self.vehicle_status.arming_state) if self.vehicle_status else None,
            "last_ack_command": int(self.last_command_ack.command) if self.last_command_ack else None,
            "last_ack_result": int(self.last_command_ack.result) if self.last_command_ack else None,
            "last_ack_result_text": self.ack_result_text(self.last_command_ack.result) if self.last_command_ack else None,
        }
        msg = String()
        msg.data = json.dumps(status, sort_keys=True)
        self.status_pub.publish(msg)

    def tick(self):
        if (
            self.land_requested
            and bool(self.get_parameter("auto_disarm_after_landed").value)
            and not self.disarm_after_land_sent
            and self.px4_armed()
            and self.landed_since_ns > 0
        ):
            delay_ns = int(float(self.get_parameter("landed_disarm_delay_sec").value) * 1e9)
            if self.get_clock().now().nanoseconds - self.landed_since_ns >= delay_ns:
                self.vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
                self.disarm_after_land_sent = True
                self.get_logger().info("Requested PX4 DISARM after vehicle_land_detected.landed=true")

        offboard = OffboardControlMode()
        offboard.timestamp = self.timestamp_us()
        offboard.position = True
        offboard.velocity = False
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False
        self.offboard_pub.publish(offboard)

        if self.last_pose and self.px4_ready_for_position_setpoint():
            pose = self.last_pose.pose
            setpoint = TrajectorySetpoint()
            setpoint.timestamp = self.timestamp_us()
            setpoint.position = self.map_pose_to_px4_ned(pose)
            setpoint.velocity, setpoint.acceleration = self.climb_feedforward(pose, setpoint.position)
            setpoint.yaw = float(self.yaw_enu_to_ned(yaw_from_quaternion(pose.orientation)))
            self.setpoint_pub.publish(setpoint)
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_setpoint_log_ns > 2_000_000_000:
                self.last_setpoint_log_ns = now_ns
                self.get_logger().info(
                    f"Publishing PX4 setpoint NED: x={setpoint.position[0]:.2f} "
                    f"y={setpoint.position[1]:.2f} z={setpoint.position[2]:.2f} yaw={setpoint.yaw:.2f}"
                )
            if self.setpoint_counter < int(self.get_parameter("preoffboard_setpoint_count").value):
                self.setpoint_counter += 1

        ready_for_command = self.last_pose and self.setpoint_counter >= int(self.get_parameter("preoffboard_setpoint_count").value)
        if self.get_parameter("auto_offboard").value and ready_for_command and not self.px4_in_offboard() and self.should_retry(self.last_offboard_request_ns):
            self.vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.last_offboard_request_ns = self.get_clock().now().nanoseconds
            self.offboard_sent = True
            self.get_logger().info("Requested PX4 OFFBOARD mode after setpoint pre-roll")
        if self.get_parameter("auto_arm").value and ready_for_command and not self.px4_armed() and self.should_retry(self.last_arm_request_ns):
            self.vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.last_arm_request_ns = self.get_clock().now().nanoseconds
            self.arm_sent = True
            self.get_logger().info("Requested PX4 ARM after setpoint pre-roll")
        self.publish_status(throttle=True)


def main(args=None):
    rclpy.init(args=args)
    node = Px4OffboardBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
