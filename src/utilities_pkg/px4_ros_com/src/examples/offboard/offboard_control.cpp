// ✅ offboard_control_node.cpp — TF‑based home‑altitude guard (v2)
// 위치 제어·속도 제어를 외부 명령에 따라 전환하며 동작
//   • TF 자료로 홈 고도를 1회 설정 (map → UAV base frame)
//   • 홈+0.5 m 이상 고도에서는 DISARM 거부하고 경고만 출력

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <cmath>
#include <sstream>

// ENU<->NED 및 ROS<->PX4 변환 유틸
#include "px4_ros_com/frame_transforms.h"

using std::placeholders::_1;
using namespace px4_msgs::msg;

enum class ControlMode { POSITION, VELOCITY };

class OffboardControl : public rclcpp::Node
{
public:
  OffboardControl()
  : Node("offboard_control_node"),
    tf_buffer_(this->get_clock())           // TF buffer (uses node clock)
  {
    tf_buffer_.setUsingDedicatedThread(true);
    /* ───── Publishers ───── */
    offboard_control_mode_pub_ = create_publisher<OffboardControlMode>  ("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_   = create_publisher<TrajectorySetpoint>   ("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_       = create_publisher<VehicleCommand>       ("/fmu/in/vehicle_command", 10);
    debug_input_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/debug/offboard/input_pose_enu", 10);
    debug_local_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/debug/offboard/local_pose_enu", 10);
    debug_setpoint_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/debug/offboard/setpoint_pose_ned", 10);
    debug_frame_report_pub_ = create_publisher<std_msgs::msg::String>(
      "/debug/offboard/frame_report", 10);

    /* ───── Subscriptions ───── */
    pose_sub_          = create_subscription<geometry_msgs::msg::PoseStamped>("/command/pose",   10, std::bind(&OffboardControl::pose_callback,   this, _1));
    twist_sub_         = create_subscription<geometry_msgs::msg::Twist>      ("/command/twist",  10, std::bind(&OffboardControl::twist_callback,  this, _1));
    gimbal_pitch_sub_  = create_subscription<std_msgs::msg::Float32>       ("/gimbal_pitch_degree", 10, std::bind(&OffboardControl::gimbal_callback, this, _1));
    disarm_sub_        = create_subscription<std_msgs::msg::Bool>         ("/command/disarm",   10, std::bind(&OffboardControl::disarm_callback, this, _1));
    land_sub_          = create_subscription<std_msgs::msg::Bool>         ("/command/land",     10, std::bind(&OffboardControl::land_callback, this, _1));
    mission_reset_sub_ = create_subscription<std_msgs::msg::Bool>         ("/mission/reset",    10, std::bind(&OffboardControl::mission_reset_callback, this, _1));
    vehicle_control_mode_sub_ = create_subscription<VehicleControlMode>(
      "/fmu/out/vehicle_control_mode", rclcpp::SensorDataQoS(),
      std::bind(&OffboardControl::vehicle_control_mode_callback, this, _1));
    vehicle_status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS(),
      std::bind(&OffboardControl::vehicle_status_callback, this, _1));
    vehicle_command_ack_sub_ = create_subscription<VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", rclcpp::SensorDataQoS(),
      std::bind(&OffboardControl::vehicle_command_ack_callback, this, _1));
    vehicle_local_position_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
      std::bind(&OffboardControl::vehicle_local_position_callback, this, _1));

    /* ───── TF Listener ───── */
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "x500_gimbal_0/base_link");
    declare_parameter<bool>("publish_debug_setpoints", true);
    declare_parameter<bool>("enu_to_ned_enabled", true);
    declare_parameter<bool>("debug_log_target_pose", true);
    declare_parameter<bool>("use_map_origin_offset", true);
    declare_parameter<bool>("auto_set_map_origin_on_first_pose", false);
    declare_parameter<bool>("require_external_origin_for_pose", true);
    declare_parameter<std::string>("external_origin_topic", "/uav/mission2_takeoff_origin");
    declare_parameter<bool>("reject_pose_without_origin", true);
    declare_parameter<bool>("clear_target_on_pose_reject", true);
    declare_parameter<bool>("allow_external_origin_reanchor", false);
    declare_parameter<bool>("clear_origin_on_mission_reset", true);
    declare_parameter<double>("external_origin_duplicate_tolerance_m", 0.2);
    declare_parameter<double>("map_origin_x", 0.0);
    declare_parameter<double>("map_origin_y", 0.0);
    declare_parameter<double>("map_origin_z", 0.0);
    get_parameter("map_frame", map_frame_);
    get_parameter("base_frame", base_frame_);
    get_parameter("publish_debug_setpoints", publish_debug_setpoints_);
    get_parameter("enu_to_ned_enabled", enu_to_ned_enabled_);
    get_parameter("debug_log_target_pose", debug_log_target_pose_);
    get_parameter("use_map_origin_offset", use_map_origin_offset_);
    get_parameter("auto_set_map_origin_on_first_pose", auto_set_map_origin_on_first_pose_);
    get_parameter("require_external_origin_for_pose", require_external_origin_for_pose_);
    get_parameter("external_origin_topic", external_origin_topic_);
    get_parameter("reject_pose_without_origin", reject_pose_without_origin_);
    get_parameter("clear_target_on_pose_reject", clear_target_on_pose_reject_);
    get_parameter("allow_external_origin_reanchor", allow_external_origin_reanchor_);
    get_parameter("clear_origin_on_mission_reset", clear_origin_on_mission_reset_);
    get_parameter("external_origin_duplicate_tolerance_m", external_origin_duplicate_tolerance_m_);
    get_parameter("map_origin_x", map_origin_.x());
    get_parameter("map_origin_y", map_origin_.y());
    get_parameter("map_origin_z", map_origin_.z());
    map_origin_initialized_ =
      !require_external_origin_for_pose_ && !auto_set_map_origin_on_first_pose_;
    origin_source_ = map_origin_initialized_ ? "configured_map_origin" : "none";

    rclcpp::QoS origin_qos(1);
    origin_qos.transient_local().reliable();
    external_origin_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      external_origin_topic_, origin_qos,
      std::bind(&OffboardControl::external_origin_callback, this, _1));

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this, false);
    RCLCPP_INFO(
      get_logger(),
      "offboard_control frames: map_frame=%s base_frame=%s enu_to_ned_enabled=%s use_map_origin_offset=%s external_origin_topic=%s require_external_origin_for_pose=%s allow_external_origin_reanchor=%s",
      map_frame_.c_str(), base_frame_.c_str(), enu_to_ned_enabled_ ? "true" : "false",
      use_map_origin_offset_ ? "true" : "false", external_origin_topic_.c_str(),
      require_external_origin_for_pose_ ? "true" : "false",
      allow_external_origin_reanchor_ ? "true" : "false");

    /* ───── Loop timer (20 Hz) ───── */
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&OffboardControl::timer_callback, this));
  }

private:
  /* ───────── ROS I/F ───────── */
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr   trajectory_setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr       vehicle_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_input_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_local_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_setpoint_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_frame_report_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr        twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           gimbal_pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              disarm_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              land_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              mission_reset_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  external_origin_sub_;
  rclcpp::Subscription<VehicleControlMode>::SharedPtr               vehicle_control_mode_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr                    vehicle_status_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr                vehicle_command_ack_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr             vehicle_local_position_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* ───────── TF ───────── */
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_ = "map";
  std::string base_frame_ = "x500_gimbal_0/base_link";
  bool   home_alt_set_ = false;
  double home_altitude_ = 0.0;
  bool publish_debug_setpoints_{true};
  bool enu_to_ned_enabled_{true};
  bool debug_log_target_pose_{true};
  bool use_map_origin_offset_{true};
  bool auto_set_map_origin_on_first_pose_{false};
  bool require_external_origin_for_pose_{true};
  std::string external_origin_topic_{"/uav/mission2_takeoff_origin"};
  bool reject_pose_without_origin_{true};
  bool clear_target_on_pose_reject_{true};
  bool allow_external_origin_reanchor_{false};
  bool clear_origin_on_mission_reset_{true};
  double external_origin_duplicate_tolerance_m_{0.2};
  bool map_origin_initialized_{false};
  Eigen::Vector3d map_origin_{0.0, 0.0, 0.0};
  Eigen::Vector3d mission2_map_anchor_enu_{0.0, 0.0, 0.0};
  Eigen::Vector3d mission2_px4_anchor_ned_{0.0, 0.0, 0.0};
  Eigen::Vector3d latest_px4_local_ned_{0.0, 0.0, 0.0};
  Eigen::Vector3d last_delta_map_enu_{0.0, 0.0, 0.0};
  Eigen::Vector3d last_final_setpoint_ned_{0.0, 0.0, 0.0};
  bool latest_px4_local_position_valid_{false};
  bool last_origin_msg_ignored_{false};
  uint64_t duplicate_origin_ignored_count_{0};
  rclcpp::Time latest_px4_local_position_time_{0, 0, RCL_ROS_TIME};
  std::string origin_source_{"none"};
  std::string pose_rejected_reason_{"none"};
  geometry_msgs::msg::PoseStamped last_input_pose_enu_{};
  geometry_msgs::msg::PoseStamped last_local_pose_enu_{};
  geometry_msgs::msg::PoseStamped last_setpoint_pose_ned_{};

  /* ───────── State ───────── */
  ControlMode mode_ = ControlMode::POSITION;
  TrajectorySetpoint setpoint_{};
  int   setpoint_counter_ = 0;
  bool  target_command_  = false;   // setpoint 수신 여부
  bool  armed_           = false;   // ARM 여부
  bool  px4_offboard_enabled_ = false;
  bool  px4_armed_ = false;
  rclcpp::Time last_offboard_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_arm_request_time_{0, 0, RCL_ROS_TIME};

  /* ───── PX4 ID 설정 (수정 필요 시) ───── */
  const uint8_t MY_SYSID       = 46;  // 노드(컴패니언)의 sysid
  const uint8_t MY_COMPID      = 47;  // USER1
  const uint8_t TARGET_SYSID   = 1;   // PX4
  const uint8_t TARGET_COMPID  = 1;
  const uint8_t FLAG_GIMBAL    = 12;


  /* ────────────────────────── TIMER LOOP ────────────────────────── */
  void timer_callback()
  {
    /* (0) 홈 고도 1회 설정 */
    if (!home_alt_set_ && tf_buffer_.canTransform(map_frame_, base_frame_, tf2::TimePointZero)) {
      auto tf_uav = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      home_altitude_ = tf_uav.transform.translation.z;
      home_alt_set_  = true;
      RCLCPP_INFO(get_logger(), "Home altitude locked at %.2f m", home_altitude_);
    }

    /* (1) 오프보드 제어모드 heartbeat */
    publish_offboard_control_mode();

    /* (2) 아직 셋포인트가 없으면 trajectory_setpoint를 내보내지 않는다. */
    if (!target_command_) {
      RCLCPP_DEBUG(get_logger(), "Waiting for target command. Not publishing trajectory_setpoint.");
      publish_frame_report();
      return;
    }

    trajectory_setpoint_pub_->publish(setpoint_);

    if (setpoint_counter_ < 20) {
      ++setpoint_counter_;
      return;
    }

    /* (3) PX4 실제 상태 기준으로 OFFBOARD/ARM 재요청 */
    const auto now = this->now();
    if (!px4_offboard_enabled_ &&
        (last_offboard_request_time_.nanoseconds() == 0 ||
         (now - last_offboard_request_time_).seconds() > 1.0)) {
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // OFFBOARD
      last_offboard_request_time_ = now;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Requesting OFFBOARD mode because PX4 is not in offboard.");
    }

    if (!px4_armed_ &&
        (last_arm_request_time_.nanoseconds() == 0 ||
         (now - last_arm_request_time_).seconds() > 1.0)) {
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); // ARM
      last_arm_request_time_ = now;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Requesting ARM because PX4 is not armed.");
    }
  }

  /* ────────────────────────── CALLBACKS ────────────────────────── */
  // ① Pose: 위치 제어
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (reject_pose_without_origin_ && !map_origin_initialized_) {
      reject_pose_command(
        "missing_mission2_map_and_px4_local_anchor",
        "Rejecting /command/pose because Mission2 map/PX4 local anchors are not initialized.");
      return;
    }

    mode_ = ControlMode::POSITION;
    last_input_pose_enu_ = *msg;
    last_input_pose_enu_.header.frame_id = msg->header.frame_id.empty() ? map_frame_ : msg->header.frame_id;

    Eigen::Vector3d target_map_enu(msg->pose.position.x,
                                   msg->pose.position.y,
                                   msg->pose.position.z);

    // /command/pose is a map ENU absolute target. PX4 trajectory setpoints are
    // PX4 local NED absolute positions. Keep the Mission2 PX4 local x/y anchor
    // and only add the map-frame delta from the Mission2 latch pose.
    const Eigen::Vector3d delta_map_enu = target_map_enu - mission2_map_anchor_enu_;
    const Eigen::Vector3d delta_ned = enu_to_ned_enabled_ ?
      px4_ros_com::frame_transforms::enu_to_ned_local_frame(delta_map_enu) :
      delta_map_enu;
    const Eigen::Vector3d p_ned = mission2_px4_anchor_ned_ + delta_ned;

    // ROS/Gazebo map/local frame uses ENU convention:
    //   ENU x = East, y = North, z = Up
    // PX4 local setpoint commonly uses NED convention:
    //   NED x = North, y = East, z = Down
    // Therefore:
    //   ned_x = enu_y
    //   ned_y = enu_x
    //   ned_z = -enu_z
    setpoint_.position[0] = static_cast<float>(p_ned.x());
    setpoint_.position[1] = static_cast<float>(p_ned.y());
    setpoint_.position[2] = static_cast<float>(p_ned.z());

    // 자세(yaw): 기존 ROS(ENU, baselink) -> PX4(NED, aircraft) 변환 유지
    const auto &o = msg->pose.orientation;
    Eigen::Quaterniond q_ros(o.w, o.x, o.y, o.z);
    double roll_ned, pitch_ned, yaw_ned;
    if (enu_to_ned_enabled_) {
      const Eigen::Quaterniond q_px4 = px4_ros_com::frame_transforms::ros_to_px4_orientation(q_ros);
      px4_ros_com::frame_transforms::utils::quaternion::quaternion_to_euler(q_px4, roll_ned, pitch_ned, yaw_ned);
    } else {
      roll_ned = 0.0;
      pitch_ned = 0.0;
      yaw_ned = tf2::getYaw(msg->pose.orientation);
    }
    setpoint_.yaw = static_cast<float>(yaw_ned);
    last_delta_map_enu_ = delta_map_enu;
    last_final_setpoint_ned_ = p_ned;
    last_local_pose_enu_ = make_debug_pose(delta_map_enu, yaw_ned, "mission2_delta_enu");
    last_setpoint_pose_ned_ = make_debug_setpoint_pose(p_ned, yaw_ned);
    publish_debug_setpoints();
    if (!target_command_) {
      setpoint_counter_ = 0;
    }
    target_command_ = true;
    pose_rejected_reason_ = "none";
    if (debug_log_target_pose_) {
      RCLCPP_INFO(
        get_logger(),
        "Target pose received in ROS map ENU: x=%.2f y=%.2f z=%.2f yaw=%.2f",
        target_map_enu.x(), target_map_enu.y(), target_map_enu.z(), tf2::getYaw(msg->pose.orientation));
      RCLCPP_INFO(
        get_logger(),
        "Mission2 delta map ENU: x=%.2f y=%.2f z=%.2f",
        delta_map_enu.x(), delta_map_enu.y(), delta_map_enu.z());
      RCLCPP_INFO(
        get_logger(),
        "Final PX4 local NED setpoint from anchor+delta: x=%.2f y=%.2f z=%.2f yaw=%.2f",
        setpoint_.position[0], setpoint_.position[1], setpoint_.position[2], setpoint_.yaw);
    }
  }

  // ② Twist: 속도 제어
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    mode_ = ControlMode::VELOCITY;
    setpoint_.position[0] = setpoint_.position[1] = setpoint_.position[2] = NAN;
    // 속도: ENU -> NED
    Eigen::Vector3d v_enu(msg->linear.x, msg->linear.y, msg->linear.z);
    const Eigen::Vector3d v_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(v_enu);
    setpoint_.velocity[0] = static_cast<float>(v_ned.x());
    setpoint_.velocity[1] = static_cast<float>(v_ned.y());
    setpoint_.velocity[2] = static_cast<float>(v_ned.z());
    setpoint_.yaw         = NAN;
    // Yaw rate: ENU(+Z up) -> NED(+Z down) → 부호 반전
    setpoint_.yawspeed    = -msg->angular.z;
    if (!target_command_) {
      setpoint_counter_ = 0;
    }
    target_command_ = true;
  }

  void external_origin_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const Eigen::Vector3d received_map_anchor(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);

    if (map_origin_initialized_ && !allow_external_origin_reanchor_) {
      const double origin_delta = (received_map_anchor - mission2_map_anchor_enu_).norm();
      last_origin_msg_ignored_ = true;
      ++duplicate_origin_ignored_count_;
      if (origin_delta <= external_origin_duplicate_tolerance_m_) {
        RCLCPP_WARN(
          get_logger(),
          "Duplicate %s ignored. Existing anchor is kept. duplicate_origin_ignored_count=%lu",
          external_origin_topic_.c_str(),
          static_cast<unsigned long>(duplicate_origin_ignored_count_));
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Received different mission2 origin while anchor already initialized. Ignoring because reanchor is disabled. delta=%.2f duplicate_origin_ignored_count=%lu",
          origin_delta,
          static_cast<unsigned long>(duplicate_origin_ignored_count_));
      }
      publish_frame_report();
      return;
    }

    if (!latest_px4_local_position_valid_) {
      map_origin_initialized_ = false;
      pose_rejected_reason_ = "missing_vehicle_local_position_at_origin_latch";
      origin_source_ = "none";
      last_origin_msg_ignored_ = true;
      RCLCPP_ERROR(
        get_logger(),
        "Received %s but cannot initialize origin: latest /fmu/out/vehicle_local_position is invalid or missing.",
        external_origin_topic_.c_str());
      publish_frame_report();
      return;
    }

    mission2_map_anchor_enu_ = received_map_anchor;
    mission2_px4_anchor_ned_ = latest_px4_local_ned_;

    map_origin_ = mission2_map_anchor_enu_;
    map_origin_initialized_ = true;
    origin_source_ = "mission2_map_and_px4_local_anchor";
    pose_rejected_reason_ = "none";
    last_origin_msg_ignored_ = false;
    RCLCPP_INFO(
      get_logger(),
      "Mission2 anchors set from %s: map_enu=(%.2f, %.2f, %.2f), px4_ned=(%.2f, %.2f, %.2f)",
      external_origin_topic_.c_str(),
      mission2_map_anchor_enu_.x(), mission2_map_anchor_enu_.y(), mission2_map_anchor_enu_.z(),
      mission2_px4_anchor_ned_.x(), mission2_px4_anchor_ned_.y(), mission2_px4_anchor_ned_.z());
    publish_frame_report();
  }

  void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg)
  {
    latest_px4_local_position_valid_ = msg->xy_valid && msg->z_valid;
    latest_px4_local_ned_.x() = msg->x;
    latest_px4_local_ned_.y() = msg->y;
    latest_px4_local_ned_.z() = msg->z;
    latest_px4_local_position_time_ = this->now();
  }

  void mission_reset_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data || !clear_origin_on_mission_reset_) {
      return;
    }
    clear_external_origin_anchor("mission_reset");
  }

  void clear_external_origin_anchor(const std::string & reason)
  {
    map_origin_initialized_ =
      !require_external_origin_for_pose_ && !auto_set_map_origin_on_first_pose_;
    mission2_map_anchor_enu_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    mission2_px4_anchor_ned_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    map_origin_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    origin_source_ = map_origin_initialized_ ? "configured_map_origin" : "none";
    pose_rejected_reason_ = reason;
    target_command_ = false;
    setpoint_counter_ = 0;
    last_origin_msg_ignored_ = false;
    duplicate_origin_ignored_count_ = 0;
    RCLCPP_WARN(get_logger(), "Mission2 external origin anchor cleared: %s", reason.c_str());
    publish_frame_report();
  }

  // ③ Gimbal pitch
  void gimbal_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    send_gimbal_pitch(static_cast<float>(msg->data));
  }

  // ④ Disarm (고도 제한 포함)
  void disarm_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) return;

    if (!home_alt_set_) {
      RCLCPP_WARN(get_logger(), "Home altitude not set yet; cannot evaluate altitude guard. Disarm aborted.");
      return;
    }

    if (!tf_buffer_.canTransform(map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05))) {
      RCLCPP_WARN(get_logger(), "TF unavailable; cannot get current altitude. Disarm aborted.");
      return;
    }

    auto tf_uav = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    double curr_alt = tf_uav.transform.translation.z;

    if (curr_alt > home_altitude_ + 0.5) {
      RCLCPP_WARN(get_logger(), "Disarm denied: current alt %.2f m > home+0.5 m (%.2f m)",
                  curr_alt, home_altitude_ + 0.5);
      return;
    }

    RCLCPP_INFO(get_logger(), "DISARM requested at %.2f m (≤limit). Sending command 400", curr_alt);
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f, 21196.0f);
    armed_ = false;
    px4_armed_ = false;
    px4_offboard_enabled_ = false;
    target_command_ = false;
    setpoint_counter_ = 0;
  }

  void land_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }
    RCLCPP_WARN(get_logger(), "LAND requested. Sending VEHICLE_CMD_NAV_LAND.");
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
  }

  void vehicle_control_mode_callback(const VehicleControlMode::SharedPtr msg)
  {
    px4_offboard_enabled_ = msg->flag_control_offboard_enabled;
    px4_armed_ = msg->flag_armed;
    armed_ = px4_armed_;
  }

  void vehicle_status_callback(const VehicleStatus::SharedPtr msg)
  {
    px4_armed_ = (msg->arming_state == VehicleStatus::ARMING_STATE_ARMED);
    armed_ = px4_armed_;
    if (msg->failsafe) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "PX4 vehicle_status reports failsafe=true. nav_state=%u arming_state=%u",
        static_cast<unsigned>(msg->nav_state), static_cast<unsigned>(msg->arming_state));
    }
  }

  void vehicle_command_ack_callback(const VehicleCommandAck::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "VehicleCommandAck command=%u result=%u",
      static_cast<unsigned>(msg->command), static_cast<unsigned>(msg->result));

    if (msg->result == VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED ||
        msg->result == VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED ||
        msg->result == VehicleCommandAck::VEHICLE_CMD_RESULT_FAILED) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "PX4 rejected/failed VehicleCommand command=%u result=%u result_param1=%u result_param2=%d",
        static_cast<unsigned>(msg->command), static_cast<unsigned>(msg->result),
        static_cast<unsigned>(msg->result_param1), msg->result_param2);
    }
  }

  /* ────────────────────────── HELPER FNs ────────────────────────── */
  void publish_offboard_control_mode()
  {
    OffboardControlMode msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position        = (mode_ == ControlMode::POSITION);
    msg.velocity        = (mode_ == ControlMode::VELOCITY);
    msg.acceleration    = false;
    msg.attitude        = false;
    msg.body_rate       = false;
    offboard_control_mode_pub_->publish(msg);
  }

  void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    VehicleCommand cmd{};
    cmd.timestamp       = this->get_clock()->now().nanoseconds() / 1000;
    cmd.param1          = param1;
    cmd.param2          = param2;
    cmd.command         = command;
    cmd.target_system   = TARGET_SYSID;
    cmd.target_component= TARGET_COMPID;
    cmd.source_system   = MY_SYSID;
    cmd.source_component= MY_COMPID;
    cmd.from_external   = true;
    vehicle_command_pub_->publish(cmd);
  }

  void reject_pose_command(const std::string & reason, const std::string & log_message)
  {
    pose_rejected_reason_ = reason;
    if (clear_target_on_pose_reject_) {
      target_command_ = false;
      setpoint_counter_ = 0;
    }
    RCLCPP_ERROR(get_logger(), "%s", log_message.c_str());
    publish_frame_report();
  }

  void ensure_map_origin_initialized(const Eigen::Vector3d & first_pose_map_enu)
  {
    if (!use_map_origin_offset_ || map_origin_initialized_) {
      return;
    }

    if (require_external_origin_for_pose_) {
      return;
    }

    if (tf_buffer_.canTransform(map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05))) {
      const auto tf_uav = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      map_origin_.x() = tf_uav.transform.translation.x;
      map_origin_.y() = tf_uav.transform.translation.y;
      map_origin_.z() = tf_uav.transform.translation.z;
      map_origin_initialized_ = true;
      origin_source_ = "tf";
      RCLCPP_INFO(
        get_logger(),
        "Map origin for PX4 local conversion set from TF: x=%.2f, y=%.2f, z=%.2f",
        map_origin_.x(), map_origin_.y(), map_origin_.z());
      return;
    }

    if (!auto_set_map_origin_on_first_pose_) {
      return;
    }

    map_origin_ = first_pose_map_enu;
    map_origin_initialized_ = true;
    origin_source_ = "first_pose";
    RCLCPP_WARN(
      get_logger(),
      "Failed to lookup UAV TF for map origin. Using first /command/pose as origin.");
  }

  geometry_msgs::msg::PoseStamped make_debug_pose(
    const Eigen::Vector3d & position,
    double yaw,
    const std::string & frame_id) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = frame_id;
    pose.pose.position.x = position.x();
    pose.pose.position.y = position.y();
    pose.pose.position.z = position.z();
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    return pose;
  }

  geometry_msgs::msg::PoseStamped make_debug_setpoint_pose(
    const Eigen::Vector3d & position,
    double yaw) const
  {
    return make_debug_pose(position, yaw, "px4_ned");
  }

  void publish_debug_setpoints()
  {
    if (!publish_debug_setpoints_) {
      return;
    }
    debug_input_pose_pub_->publish(last_input_pose_enu_);
    debug_local_pose_pub_->publish(last_local_pose_enu_);
    debug_setpoint_pose_pub_->publish(last_setpoint_pose_ned_);
    publish_frame_report();
  }

  void publish_frame_report()
  {
    std_msgs::msg::String report;
    std::ostringstream stream;
    stream << "map_frame=" << map_frame_
           << ", base_frame=" << base_frame_
           << ", enu_to_ned_enabled=" << (enu_to_ned_enabled_ ? "true" : "false")
           << ", use_map_origin_offset=" << (use_map_origin_offset_ ? "true" : "false")
           << ", origin_initialized=" << (map_origin_initialized_ ? "true" : "false")
           << ", origin_source=" << origin_source_
           << ", pose_rejected_reason=" << pose_rejected_reason_
           << ", target_command=" << (target_command_ ? "true" : "false")
           << ", allow_external_origin_reanchor=" << (allow_external_origin_reanchor_ ? "true" : "false")
           << ", last_origin_msg_ignored=" << (last_origin_msg_ignored_ ? "true" : "false")
           << ", duplicate_origin_ignored_count=" << duplicate_origin_ignored_count_
           << ", map_origin=("
           << map_origin_.x() << ","
           << map_origin_.y() << ","
           << map_origin_.z() << ")"
           << ", mission2_map_anchor_enu=("
           << mission2_map_anchor_enu_.x() << ","
           << mission2_map_anchor_enu_.y() << ","
           << mission2_map_anchor_enu_.z() << ")"
           << ", mission2_px4_anchor_ned=("
           << mission2_px4_anchor_ned_.x() << ","
           << mission2_px4_anchor_ned_.y() << ","
           << mission2_px4_anchor_ned_.z() << ")"
           << ", latest_px4_local_ned=("
           << latest_px4_local_ned_.x() << ","
           << latest_px4_local_ned_.y() << ","
           << latest_px4_local_ned_.z() << ")"
           << ", input_enu=("
           << last_input_pose_enu_.pose.position.x << ","
           << last_input_pose_enu_.pose.position.y << ","
           << last_input_pose_enu_.pose.position.z << ")"
           << ", delta_map_enu=("
           << last_delta_map_enu_.x() << ","
           << last_delta_map_enu_.y() << ","
           << last_delta_map_enu_.z() << ")"
           << ", local_pose_enu_debug=("
           << last_local_pose_enu_.pose.position.x << ","
           << last_local_pose_enu_.pose.position.y << ","
           << last_local_pose_enu_.pose.position.z << ")"
           << ", final_setpoint_ned=("
           << last_final_setpoint_ned_.x() << ","
           << last_final_setpoint_ned_.y() << ","
           << last_final_setpoint_ned_.z() << ")"
           << ", setpoint_pose_ned_debug=("
           << last_setpoint_pose_ned_.pose.position.x << ","
           << last_setpoint_pose_ned_.pose.position.y << ","
           << last_setpoint_pose_ned_.pose.position.z << ")";
    report.data = stream.str();
    debug_frame_report_pub_->publish(report);
  }

  /* ─── Gimbal pitch helper ─── */
  void take_gimbal_control()
  {
    VehicleCommand cmd{};
    cmd.timestamp       = this->get_clock()->now().nanoseconds() / 1000;
    cmd.command         = VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;   // 1001
    cmd.param1          = MY_SYSID;
    cmd.param2          = MY_COMPID;
    cmd.param3 = cmd.param4 = 0;   // no secondary control
    cmd.param5          = FLAG_GIMBAL;    // flags
    cmd.param7          = 1;              // gimbal device id
    cmd.target_system   = TARGET_SYSID;
    cmd.target_component= TARGET_COMPID;
    cmd.source_system   = MY_SYSID;
    cmd.source_component= MY_COMPID;
    cmd.from_external   = true;
    vehicle_command_pub_->publish(cmd);
  }

  void send_gimbal_pitch(float pitch_deg)
  {
    take_gimbal_control();
    VehicleCommand cmd{};
    cmd.timestamp       = this->get_clock()->now().nanoseconds() / 1000;
    cmd.command         = VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW; // 1000
    cmd.param1          = pitch_deg;   // pitch (+up, -down)
    cmd.param2          = 0.0f;        // yaw hold
    cmd.param3 = cmd.param4 = NAN;     // rates
    cmd.param5          = FLAG_GIMBAL;
    cmd.param7          = 1;           // device id
    cmd.target_system   = TARGET_SYSID;
    cmd.target_component= TARGET_COMPID;
    cmd.source_system   = MY_SYSID;
    cmd.source_component= MY_COMPID;
    cmd.from_external   = true;
    vehicle_command_pub_->publish(cmd);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());
  rclcpp::shutdown();
  return 0;
}
