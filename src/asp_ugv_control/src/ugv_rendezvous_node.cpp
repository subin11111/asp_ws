#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

struct RendezvousWaypoint
{
  double x{};
  double y{};
  double target_speed{};
};

class UgvRendezvousNode : public rclcpp::Node
{
public:
  UgvRendezvousNode()
  : Node("ugv_rendezvous_node"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    declare_parameters();
    read_parameters();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    reached_pub_ = create_publisher<std_msgs::msg::Bool>(rendezvous_reached_topic_, 10);
    event_pub_ = create_publisher<std_msgs::msg::String>("/ugv/mission_event", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/ugv/rendezvous_state", 10);

    start_sub_ = create_subscription<std_msgs::msg::Bool>(
      start_topic_, 10, std::bind(&UgvRendezvousNode::start_callback, this, std::placeholders::_1));

    path_loaded_ = load_path(path_csv_);
    publish_state(path_loaded_ ? "IDLE" : "ERROR:NO_PATH");

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&UgvRendezvousNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "UGV rendezvous waits for %s", start_topic_.c_str());
    RCLCPP_INFO(get_logger(), "UGV rendezvous command topic: %s", cmd_vel_topic_.c_str());
  }

  ~UgvRendezvousNode() override
  {
    publish_zero_twist();
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("start_topic", "/ugv/rendezvous_start");
    declare_parameter<std::string>("rendezvous_reached_topic", "/ugv/rendezvous_reached");
    declare_parameter<std::string>("path_csv", "");
    declare_parameter<std::string>("cmd_vel_topic", "/command/ugv_cmd_vel");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "X1_asp/base_link");
    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("waypoint_tolerance", 0.9);
    declare_parameter<double>("final_tolerance", 1.0);
    declare_parameter<double>("lookahead_distance", 3.5);
    declare_parameter<double>("max_linear_speed", 1.4);
    declare_parameter<double>("cruise_speed", 1.1);
    declare_parameter<double>("min_linear_speed", 0.35);
    declare_parameter<double>("max_angular_speed", 1.4);
    declare_parameter<double>("kp_heading", 1.6);
    declare_parameter<double>("kp_distance", 0.8);
    declare_parameter<double>("slow_down_distance", 2.0);
    declare_parameter<double>("heading_slowdown_cos_threshold", 0.2);
    declare_parameter<bool>("allow_reverse", false);
    declare_parameter<double>("stuck_timeout_sec", 20.0);
    declare_parameter<double>("progress_epsilon_m", 0.4);
    declare_parameter<int>("zero_publish_after_complete_count", 5);
  }

  void read_parameters()
  {
    get_parameter("start_topic", start_topic_);
    get_parameter("rendezvous_reached_topic", rendezvous_reached_topic_);
    get_parameter("path_csv", path_csv_);
    get_parameter("cmd_vel_topic", cmd_vel_topic_);
    get_parameter("map_frame", map_frame_);
    get_parameter("base_frame", base_frame_);
    get_parameter("control_rate_hz", control_rate_hz_);
    get_parameter("waypoint_tolerance", waypoint_tolerance_);
    get_parameter("final_tolerance", final_tolerance_);
    get_parameter("lookahead_distance", lookahead_distance_);
    get_parameter("max_linear_speed", max_linear_speed_);
    get_parameter("cruise_speed", cruise_speed_);
    get_parameter("min_linear_speed", min_linear_speed_);
    get_parameter("max_angular_speed", max_angular_speed_);
    get_parameter("kp_heading", kp_heading_);
    get_parameter("kp_distance", kp_distance_);
    get_parameter("slow_down_distance", slow_down_distance_);
    get_parameter("heading_slowdown_cos_threshold", heading_slowdown_cos_threshold_);
    get_parameter("allow_reverse", allow_reverse_);
    get_parameter("stuck_timeout_sec", stuck_timeout_sec_);
    get_parameter("progress_epsilon_m", progress_epsilon_m_);
    get_parameter("zero_publish_after_complete_count", zero_publish_after_complete_count_);
    control_rate_hz_ = std::max(1.0, control_rate_hz_);
    waypoint_tolerance_ = std::max(0.05, waypoint_tolerance_);
    final_tolerance_ = std::max(0.05, final_tolerance_);
    lookahead_distance_ = std::max(0.0, lookahead_distance_);
    max_linear_speed_ = std::max(0.0, max_linear_speed_);
    cruise_speed_ = clamp(cruise_speed_, 0.0, max_linear_speed_);
    min_linear_speed_ = clamp(min_linear_speed_, 0.0, max_linear_speed_);
    max_angular_speed_ = std::max(0.0, max_angular_speed_);
    slow_down_distance_ = std::max(0.1, slow_down_distance_);
    heading_slowdown_cos_threshold_ = clamp(heading_slowdown_cos_threshold_, -1.0, 1.0);
    stuck_timeout_sec_ = std::max(0.0, stuck_timeout_sec_);
    progress_epsilon_m_ = std::max(0.0, progress_epsilon_m_);
    zero_publish_after_complete_count_ = std::max(0, zero_publish_after_complete_count_);
  }

  bool load_path(const std::string & path)
  {
    if (path.empty()) {
      RCLCPP_ERROR(get_logger(), "rendezvous path_csv is empty.");
      return false;
    }
    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open rendezvous path: %s", path.c_str());
      return false;
    }
    std::string line;
    size_t line_number = 0;
    while (std::getline(file, line)) {
      ++line_number;
      if (line.empty() || line[0] == '#') {
        continue;
      }
      std::stringstream ss(line);
      std::string x_str;
      std::string y_str;
      std::string speed_str;
      if (!std::getline(ss, x_str, ',') ||
          !std::getline(ss, y_str, ',') ||
          !std::getline(ss, speed_str, ',')) {
        RCLCPP_WARN(get_logger(), "Skipping malformed rendezvous line %zu.", line_number);
        continue;
      }
      try {
        RendezvousWaypoint waypoint;
        waypoint.x = std::stod(x_str);
        waypoint.y = std::stod(y_str);
        waypoint.target_speed = std::stod(speed_str);
        if (std::isfinite(waypoint.x) && std::isfinite(waypoint.y)) {
          waypoint.target_speed = clamp(waypoint.target_speed, 0.0, max_linear_speed_);
          waypoints_.push_back(waypoint);
        }
      } catch (const std::exception &) {
        if (line_number != 1) {
          RCLCPP_WARN(get_logger(), "Skipping invalid rendezvous CSV line %zu: %s", line_number, line.c_str());
        }
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu rendezvous waypoints.", waypoints_.size());
    return !waypoints_.empty();
  }

  void start_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data || !path_loaded_ || completed_) {
      return;
    }
    if (!active_) {
      active_ = true;
      current_index_ = 0;
      current_waypoint_started_at_ = now();
      last_progress_time_ = now();
      best_distance_to_waypoint_ = std::numeric_limits<double>::infinity();
      publish_event("RENDEZVOUS_STARTED");
      publish_state("RUNNING");
    }
  }

  void control_loop()
  {
    if (!active_ || completed_) {
      publish_state(completed_ ? "COMPLETE" : "IDLE");
      return;
    }
    if (!path_loaded_ || waypoints_.empty()) {
      publish_zero_twist();
      publish_state("ERROR:NO_PATH");
      return;
    }

    geometry_msgs::msg::TransformStamped pose;
    try {
      pose = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      publish_zero_twist();
      publish_state("WAITING_FOR_TF");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return;
    }

    const double current_x = pose.transform.translation.x;
    const double current_y = pose.transform.translation.y;
    const double current_yaw = yaw_from_quaternion(pose.transform.rotation);

    advance_lookahead(current_x, current_y);

    const auto & target = waypoints_[current_index_];
    const double distance = std::hypot(target.x - current_x, target.y - current_y);
    update_progress(distance);
    if (stuck_timeout_sec_ > 0.0 &&
        last_progress_time_.nanoseconds() > 0 &&
        (now() - last_progress_time_).seconds() > stuck_timeout_sec_) {
      publish_zero_twist();
      publish_state("STUCK");
      publish_event("RENDEZVOUS_STUCK");
      active_ = false;
      return;
    }

    const bool final_waypoint = current_index_ + 1 >= waypoints_.size();
    const double tolerance = final_waypoint ? final_tolerance_ : waypoint_tolerance_;

    if (distance <= tolerance) {
      RCLCPP_INFO(get_logger(), "Reached rendezvous waypoint %zu.", current_index_);
      if (final_waypoint) {
        complete_rendezvous();
        return;
      }
      ++current_index_;
      current_waypoint_started_at_ = now();
      last_progress_time_ = now();
      best_distance_to_waypoint_ = std::numeric_limits<double>::infinity();
      publish_state("NEXT_WAYPOINT");
      return;
    }

    publish_state("RUNNING");
    publish_drive_command(current_x, current_y, current_yaw, target, final_waypoint);
  }

  void advance_lookahead(double current_x, double current_y)
  {
    while (current_index_ + 1 < waypoints_.size()) {
      const auto & waypoint = waypoints_[current_index_];
      const double distance = std::hypot(waypoint.x - current_x, waypoint.y - current_y);
      if (distance > lookahead_distance_) {
        return;
      }
      ++current_index_;
      current_waypoint_started_at_ = now();
      last_progress_time_ = now();
      best_distance_to_waypoint_ = std::numeric_limits<double>::infinity();
      publish_event("RENDEZVOUS_LOOKAHEAD_SKIP");
    }
  }

  void update_progress(double distance)
  {
    if (distance < best_distance_to_waypoint_ - progress_epsilon_m_) {
      best_distance_to_waypoint_ = distance;
      last_progress_time_ = now();
    }
  }

  void publish_drive_command(
    double current_x,
    double current_y,
    double current_yaw,
    const RendezvousWaypoint & target,
    bool final_waypoint)
  {
    const double dx = target.x - current_x;
    const double dy = target.y - current_y;
    const double distance = std::hypot(dx, dy);
    const double target_heading = std::atan2(dy, dx);
    const double heading_error = normalize_angle(target_heading - current_yaw);

    const double base_speed = target.target_speed > 0.0 ? target.target_speed : cruise_speed_;
    double linear_x = std::min(base_speed, kp_distance_ * distance);
    if (final_waypoint && distance < slow_down_distance_) {
      linear_x *= clamp(distance / slow_down_distance_, 0.0, 1.0);
    }
    const double heading_cos = std::cos(heading_error);
    double heading_scale = 1.0;
    if (!allow_reverse_) {
      if (heading_cos <= heading_slowdown_cos_threshold_) {
        heading_scale = 0.0;
      } else {
        heading_scale = clamp(
          (heading_cos - heading_slowdown_cos_threshold_) /
          (1.0 - heading_slowdown_cos_threshold_),
          0.0,
          1.0);
      }
    }
    linear_x *= heading_scale;
    if (linear_x > 1e-3) {
      linear_x = std::max(linear_x, min_linear_speed_);
    }
    linear_x = clamp(linear_x, 0.0, max_linear_speed_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.angular.z = clamp(kp_heading_ * heading_error, -max_angular_speed_, max_angular_speed_);
    cmd_pub_->publish(cmd);
  }

  void complete_rendezvous()
  {
    publish_zero_twist_burst(zero_publish_after_complete_count_);
    active_ = false;
    completed_ = true;
    publish_state("COMPLETE");
    publish_event("RENDEZVOUS_REACHED");
    std_msgs::msg::Bool reached;
    reached.data = true;
    reached_pub_->publish(reached);
  }

  void publish_zero_twist()
  {
    if (!cmd_pub_) {
      return;
    }
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
  }

  void publish_zero_twist_burst(int count)
  {
    for (int index = 0; index < count; ++index) {
      publish_zero_twist();
    }
  }

  void publish_event(const std::string & event)
  {
    std_msgs::msg::String msg;
    msg.data = event;
    event_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", event.c_str());
  }

  void publish_state(const std::string & state)
  {
    std_msgs::msg::String msg;
    msg.data = state;
    state_pub_->publish(msg);
  }

  std::string start_topic_;
  std::string rendezvous_reached_topic_;
  std::string path_csv_;
  std::string cmd_vel_topic_;
  std::string map_frame_;
  std::string base_frame_;
  double control_rate_hz_{20.0};
  double waypoint_tolerance_{0.9};
  double final_tolerance_{1.0};
  double lookahead_distance_{3.5};
  double max_linear_speed_{1.4};
  double cruise_speed_{1.1};
  double min_linear_speed_{0.35};
  double max_angular_speed_{1.4};
  double kp_heading_{1.6};
  double kp_distance_{0.8};
  double slow_down_distance_{2.0};
  double heading_slowdown_cos_threshold_{0.2};
  bool allow_reverse_{false};
  double stuck_timeout_sec_{20.0};
  double progress_epsilon_m_{0.4};
  int zero_publish_after_complete_count_{5};
  bool path_loaded_{false};
  bool active_{false};
  bool completed_{false};
  size_t current_index_{0};
  rclcpp::Time current_waypoint_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};
  double best_distance_to_waypoint_{std::numeric_limits<double>::infinity()};
  std::vector<RendezvousWaypoint> waypoints_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UgvRendezvousNode>());
  rclcpp::shutdown();
  return 0;
}
