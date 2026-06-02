// ✅ offboard_control_node.cpp — TF‑based home‑altitude guard (v2)
// 위치 제어·속도 제어를 외부 명령에 따라 전환하며 동작
//   • TF 자료로 홈 고도를 1회 설정 (map → base_link)
//   • 홈+0.5 m 이상 고도에서는 DISARM 거부하고 경고만 출력

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <cmath>

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

    /* ───── Subscriptions ───── */
    pose_sub_          = create_subscription<geometry_msgs::msg::PoseStamped>("/command/pose",   10, std::bind(&OffboardControl::pose_callback,   this, _1));
    twist_sub_         = create_subscription<geometry_msgs::msg::Twist>      ("/command/twist",  10, std::bind(&OffboardControl::twist_callback,  this, _1));
    gimbal_pitch_sub_  = create_subscription<std_msgs::msg::Float32>       ("/gimbal_pitch_degree", 10, std::bind(&OffboardControl::gimbal_callback, this, _1));
    disarm_sub_        = create_subscription<std_msgs::msg::Bool>         ("/command/disarm",   10, std::bind(&OffboardControl::disarm_callback, this, _1));

    /* ───── TF Listener ───── */
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this, false);

    /* ───── Loop timer (20 Hz) ───── */
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&OffboardControl::timer_callback, this));
  }

private:
  /* ───────── ROS I/F ───────── */
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr   trajectory_setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr       vehicle_command_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr        twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           gimbal_pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              disarm_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* ───────── TF ───────── */
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  const std::string map_frame_ = "map";
  const std::string uav_frame_ = "base_link";   // ← 필요 시 파라미터화
  bool   home_alt_set_ = false;
  double home_altitude_ = 0.0;

  /* ───────── State ───────── */
  ControlMode mode_ = ControlMode::POSITION;
  TrajectorySetpoint setpoint_{};
  int   setpoint_counter_ = 0;
  bool  target_command_  = false;   // setpoint 수신 여부
  bool  armed_           = false;   // ARM 여부

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
    if (!home_alt_set_ && tf_buffer_.canTransform(map_frame_, uav_frame_, tf2::TimePointZero)) {
      auto tf_uav = tf_buffer_.lookupTransform(map_frame_, uav_frame_, tf2::TimePointZero);
      home_altitude_ = tf_uav.transform.translation.z;
      home_alt_set_  = true;
      RCLCPP_INFO(get_logger(), "Home altitude locked at %.2f m", home_altitude_);
    }

    /* (1) 오프보드 제어모드 & 셋포인트 송출 */
    publish_offboard_control_mode();
    trajectory_setpoint_pub_->publish(setpoint_);

    /* (2) 아직 셋포인트가 없으면 대기 */
    if (!target_command_) {
      RCLCPP_DEBUG(get_logger(), "Waiting for target command.");
      return;
    }

    /* (3) 첫 셋포인트 수신 후 10회(≈1 s) 연속 송출 → OFFBOARD 모드 + ARM */
    if (!armed_ && ++setpoint_counter_ >= 20) {
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);          // OFFBOARD
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); // ARM
      armed_ = true;
      RCLCPP_INFO(get_logger(), "Set OFFBOARD mode & ARM (after pose received)");
    }
  }

  /* ────────────────────────── CALLBACKS ────────────────────────── */
  // ① Pose: 위치 제어
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    mode_ = ControlMode::POSITION;
    // 위치: ENU -> NED
    Eigen::Vector3d p_enu(msg->pose.position.x,
                          msg->pose.position.y,
                          msg->pose.position.z);
    const Eigen::Vector3d p_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(p_enu);
    setpoint_.position[0] = static_cast<float>(p_ned.x());
    setpoint_.position[1] = static_cast<float>(p_ned.y());
    setpoint_.position[2] = static_cast<float>(p_ned.z());

    // 자세(yaw): ROS(ENU, baselink) -> PX4(NED, aircraft)
    const auto &o = msg->pose.orientation;
    Eigen::Quaterniond q_ros(o.w, o.x, o.y, o.z);
    const Eigen::Quaterniond q_px4 = px4_ros_com::frame_transforms::ros_to_px4_orientation(q_ros);
    double roll_ned, pitch_ned, yaw_ned;
    px4_ros_com::frame_transforms::utils::quaternion::quaternion_to_euler(q_px4, roll_ned, pitch_ned, yaw_ned);
    setpoint_.yaw = static_cast<float>(yaw_ned);
    target_command_ = true;
    RCLCPP_INFO(get_logger(), "Target pose arrived. X: %.2f Y: %.2f Z: %.2f Yaw: %.2f",
                setpoint_.position[0], setpoint_.position[1], setpoint_.position[2], setpoint_.yaw);
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
    target_command_ = true;
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

    if (!tf_buffer_.canTransform(map_frame_, uav_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05))) {
      RCLCPP_WARN(get_logger(), "TF unavailable; cannot get current altitude. Disarm aborted.");
      return;
    }

    auto tf_uav = tf_buffer_.lookupTransform(map_frame_, uav_frame_, tf2::TimePointZero);
    double curr_alt = tf_uav.transform.translation.z;

    if (curr_alt > home_altitude_ + 0.5) {
      RCLCPP_WARN(get_logger(), "Disarm denied: current alt %.2f m > home+0.5 m (%.2f m)",
                  curr_alt, home_altitude_ + 0.5);
      return;
    }

    RCLCPP_INFO(get_logger(), "DISARM requested at %.2f m (≤limit). Sending command 400", curr_alt);
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f, 21196.0f);
    armed_ = false;
    target_command_ = false;
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
