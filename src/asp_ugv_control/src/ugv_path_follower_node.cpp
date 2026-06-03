#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

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

struct Waypoint
{
  double x{};
  double y{};
  int mission_type{};
  double target_speed{};
};

class UgvPathFollowerNode : public rclcpp::Node
{
public:
  UgvPathFollowerNode()
  : Node("ugv_path_follower_node"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    declare_parameters();
    read_parameters();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/ugv/state", 10);
    event_pub_ = create_publisher<std_msgs::msg::String>("/ugv/mission_event", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/ugv/target_marker", 10);

    mission_loaded_ = load_mission_csv(mission_csv_);
    publish_state(mission_loaded_ ? "LOADED_MISSION" : "STOPPED");

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&UgvPathFollowerNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "UGV path follower output topic: %s", cmd_vel_topic_.c_str());
    RCLCPP_INFO(get_logger(), "TF lookup: %s -> %s", map_frame_.c_str(), base_frame_.c_str());
  }

  ~UgvPathFollowerNode() override
  {
    publish_zero_twist();
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("mission_csv", "");
    declare_parameter<std::string>("cmd_vel_topic", "/command/ugv_cmd_vel");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "X1_asp/base_link");
    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("waypoint_tolerance", 0.8);
    declare_parameter<double>("stop_tolerance", 0.5);
    declare_parameter<double>("lookahead_distance", 1.5);
    declare_parameter<double>("max_linear_speed", 0.8);
    declare_parameter<double>("min_linear_speed", 0.2);
    declare_parameter<double>("max_angular_speed", 0.8);
    declare_parameter<double>("kp_heading", 1.2);
    declare_parameter<double>("kp_distance", 0.5);
    declare_parameter<double>("slow_down_distance", 2.0);
    declare_parameter<int>("final_stop_mission_type", 2);
    declare_parameter<int>("zero_publish_after_stop_count", 5);
    declare_parameter<bool>("disable_cmd_after_stop", true);
  }

  void read_parameters()
  {
    get_parameter("mission_csv", mission_csv_);
    get_parameter("cmd_vel_topic", cmd_vel_topic_);
    get_parameter("map_frame", map_frame_);
    get_parameter("base_frame", base_frame_);
    get_parameter("control_rate_hz", control_rate_hz_);
    get_parameter("waypoint_tolerance", waypoint_tolerance_);
    get_parameter("stop_tolerance", stop_tolerance_);
    get_parameter("lookahead_distance", lookahead_distance_);
    get_parameter("max_linear_speed", max_linear_speed_);
    get_parameter("min_linear_speed", min_linear_speed_);
    get_parameter("max_angular_speed", max_angular_speed_);
    get_parameter("kp_heading", kp_heading_);
    get_parameter("kp_distance", kp_distance_);
    get_parameter("slow_down_distance", slow_down_distance_);
    get_parameter("final_stop_mission_type", final_stop_mission_type_);
    get_parameter("zero_publish_after_stop_count", zero_publish_after_stop_count_);
    get_parameter("disable_cmd_after_stop", disable_cmd_after_stop_);

    control_rate_hz_ = std::max(1.0, control_rate_hz_);
    waypoint_tolerance_ = std::max(0.05, waypoint_tolerance_);
    stop_tolerance_ = std::max(0.05, stop_tolerance_);
    lookahead_distance_ = std::max(0.0, lookahead_distance_);
    max_linear_speed_ = std::max(0.0, max_linear_speed_);
    min_linear_speed_ = clamp(min_linear_speed_, 0.0, max_linear_speed_);
    max_angular_speed_ = std::max(0.0, max_angular_speed_);
    slow_down_distance_ = std::max(0.1, slow_down_distance_);
    zero_publish_after_stop_count_ = std::max(0, zero_publish_after_stop_count_);
  }

  bool load_mission_csv(const std::string & path)
  {
    waypoints_.clear();
    current_index_ = 0;

    if (path.empty()) {
      RCLCPP_ERROR(get_logger(), "mission_csv parameter is empty.");
      return false;
    }

    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open mission_csv: %s", path.c_str());
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
      std::string mission_type_str;
      std::string target_speed_str;

      if (!std::getline(ss, x_str, ',') ||
          !std::getline(ss, y_str, ',') ||
          !std::getline(ss, mission_type_str, ',') ||
          !std::getline(ss, target_speed_str, ',')) {
        RCLCPP_WARN(get_logger(), "Skipping malformed CSV line %zu: %s", line_number, line.c_str());
        continue;
      }

      try {
        Waypoint waypoint;
        waypoint.x = std::stod(x_str);
        waypoint.y = std::stod(y_str);
        waypoint.mission_type = std::stoi(mission_type_str);
        waypoint.target_speed = std::stod(target_speed_str);

        if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y) ||
            !std::isfinite(waypoint.target_speed)) {
          RCLCPP_WARN(get_logger(), "Skipping non-finite CSV line %zu.", line_number);
          continue;
        }

        waypoints_.push_back(waypoint);
      } catch (const std::exception &) {
        if (line_number == 1) {
          RCLCPP_INFO(get_logger(), "Skipping CSV header: %s", line.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "Skipping invalid CSV line %zu: %s", line_number, line.c_str());
        }
      }
    }

    if (waypoints_.empty()) {
      RCLCPP_ERROR(get_logger(), "No valid waypoints loaded from %s.", path.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from %s.", waypoints_.size(), path.c_str());
    return true;
  }

  void control_loop()
  {
    if (stopped_) {
      if (!disable_cmd_after_stop_) {
        publish_zero_twist();
      }
      publish_state("STOPPED");
      return;
    }

    if (!mission_loaded_ || waypoints_.empty()) {
      publish_zero_twist();
      publish_state("STOPPED");
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

    if (!std::isfinite(current_x) || !std::isfinite(current_y) || !std::isfinite(current_yaw)) {
      publish_zero_twist();
      publish_state("WAITING_FOR_TF");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF contains non-finite values.");
      return;
    }

    advance_lookahead(current_x, current_y);

    if (current_index_ >= waypoints_.size()) {
      stop_mission("MISSION2_START_REACHED");
      return;
    }

    const auto & target = waypoints_[current_index_];
    const double distance = std::hypot(target.x - current_x, target.y - current_y);
    const double tolerance =
      target.mission_type == final_stop_mission_type_ ? stop_tolerance_ : waypoint_tolerance_;

    if (distance <= tolerance) {
      publish_state("WAYPOINT_REACHED");
      RCLCPP_INFO(
        get_logger(), "Reached waypoint %zu (mission_type=%d).",
        current_index_, target.mission_type);

      if (target.mission_type == final_stop_mission_type_) {
        stop_mission("MISSION2_START_REACHED");
        return;
      }

      ++current_index_;
      if (current_index_ >= waypoints_.size()) {
        stop_mission("MISSION2_START_REACHED");
        return;
      }
    }

    publish_state("FOLLOWING_PATH");
    publish_drive_command(current_x, current_y, current_yaw, waypoints_[current_index_]);
    publish_target_marker(waypoints_[current_index_]);
  }

  void advance_lookahead(double current_x, double current_y)
  {
    while (current_index_ + 1 < waypoints_.size()) {
      const auto & waypoint = waypoints_[current_index_];
      if (waypoint.mission_type == final_stop_mission_type_) {
        return;
      }

      const double distance = std::hypot(waypoint.x - current_x, waypoint.y - current_y);
      if (distance > lookahead_distance_) {
        return;
      }

      ++current_index_;
    }
  }

  void publish_drive_command(
    double current_x,
    double current_y,
    double current_yaw,
    const Waypoint & target)
  {
    geometry_msgs::msg::Twist cmd;
    const double dx = target.x - current_x;
    const double dy = target.y - current_y;
    const double distance = std::hypot(dx, dy);
    const double target_heading = std::atan2(dy, dx);
    const double heading_error = normalize_angle(target_heading - current_yaw);

    double desired_speed = target.target_speed > 0.0 ? target.target_speed : max_linear_speed_;
    desired_speed = clamp(desired_speed, 0.0, max_linear_speed_);
    desired_speed = std::min(desired_speed, kp_distance_ * distance);

    if (distance < slow_down_distance_) {
      desired_speed *= clamp(distance / slow_down_distance_, 0.0, 1.0);
    }

    const double heading_scale = clamp(std::cos(heading_error), 0.0, 1.0);
    double linear_x = desired_speed * heading_scale;
    if (linear_x > 1e-3) {
      linear_x = std::max(linear_x, min_linear_speed_);
    }
    linear_x = clamp(linear_x, 0.0, max_linear_speed_);

    const double angular_z = clamp(kp_heading_ * heading_error, -max_angular_speed_, max_angular_speed_);

    if (!std::isfinite(linear_x) || !std::isfinite(angular_z)) {
      publish_zero_twist();
      publish_state("STOPPED");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Command calculation produced NaN/Inf.");
      return;
    }

    cmd.linear.x = linear_x;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = angular_z;
    cmd_pub_->publish(cmd);
  }

  void stop_mission(const std::string & event)
  {
    if (stopped_) {
      return;
    }
    stopped_ = true;
    publish_zero_twist_burst(zero_publish_after_stop_count_);
    publish_state("STOPPED");
    publish_event(event);
    if (disable_cmd_after_stop_) {
      RCLCPP_INFO(
        get_logger(),
        "Mission1 stopped; released %s after %d zero commands.",
        cmd_vel_topic_.c_str(), zero_publish_after_stop_count_);
    } else {
      RCLCPP_INFO(get_logger(), "Mission1 stopped with event: %s", event.c_str());
    }
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

  void publish_state(const std::string & state)
  {
    if (state == last_state_ || !state_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = state;
    state_pub_->publish(msg);
    last_state_ = state;
  }

  void publish_event(const std::string & event)
  {
    if (!event_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = event;
    event_pub_->publish(msg);
  }

  void publish_target_marker(const Waypoint & target)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = map_frame_;
    marker.ns = "ugv_path_follower";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = target.x;
    marker.pose.position.y = target.y;
    marker.pose.position.z = 0.3;
    marker.scale.x = 0.6;
    marker.scale.y = 0.6;
    marker.scale.z = 0.6;
    marker.color.a = 1.0;
    marker.color.r = 0.1;
    marker.color.g = 0.9;
    marker.color.b = 0.2;
    marker_pub_->publish(marker);
  }

  std::string mission_csv_;
  std::string cmd_vel_topic_;
  std::string map_frame_;
  std::string base_frame_;
  double control_rate_hz_{20.0};
  double waypoint_tolerance_{0.8};
  double stop_tolerance_{0.5};
  double lookahead_distance_{1.5};
  double max_linear_speed_{0.8};
  double min_linear_speed_{0.2};
  double max_angular_speed_{0.8};
  double kp_heading_{1.2};
  double kp_distance_{0.5};
  double slow_down_distance_{2.0};
  int final_stop_mission_type_{2};
  int zero_publish_after_stop_count_{5};
  bool disable_cmd_after_stop_{true};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<Waypoint> waypoints_;
  size_t current_index_{0};
  bool mission_loaded_{false};
  bool stopped_{false};
  std::string last_state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UgvPathFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
