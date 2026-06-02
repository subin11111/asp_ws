// pose_tf_broadcaster.cpp
//
// Gazebo model pose topics -> ROS2 TF broadcaster
//
// 목적:
// Gazebo pose bridge로 들어오는 다음 토픽을 구독한다.
//   /model/X1_asp/pose
//   /model/X1_asp/pose_static
//   /model/x500_gimbal_0/pose
//   /model/x500_gimbal_0/pose_static
//
// 수신한 Pose_V 안의 transform들을 TF로 발행한다.
// parent frame이 "default"이면 "map"으로 바꾼다.
// child frame은 Gazebo가 제공한 이름을 그대로 유지한다.
//
// 기대 TF:
//   map -> X1_asp/base_link
//   map -> x500_gimbal_0/base_link
//
// 실행:
//   ros2 run gazebo_env_setup pose_tf_broadcaster

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <memory>
#include <string>
#include <vector>

class PoseTfBroadcaster : public rclcpp::Node
{
public:
  PoseTfBroadcaster()
  : Node("pose_tf_broadcaster")
  {
    dynamic_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    x1_pose_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/model/X1_asp/pose",
      qos,
      std::bind(&PoseTfBroadcaster::dynamicPoseCallback, this, std::placeholders::_1)
    );

    x1_pose_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/model/X1_asp/pose_static",
      qos,
      std::bind(&PoseTfBroadcaster::staticPoseCallback, this, std::placeholders::_1)
    );

    x500_pose_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/model/x500_gimbal_0/pose",
      qos,
      std::bind(&PoseTfBroadcaster::dynamicPoseCallback, this, std::placeholders::_1)
    );

    x500_pose_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/model/x500_gimbal_0/pose_static",
      qos,
      std::bind(&PoseTfBroadcaster::staticPoseCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "pose_tf_broadcaster started.");
    RCLCPP_INFO(get_logger(), "Subscribing:");
    RCLCPP_INFO(get_logger(), "  /model/X1_asp/pose");
    RCLCPP_INFO(get_logger(), "  /model/X1_asp/pose_static");
    RCLCPP_INFO(get_logger(), "  /model/x500_gimbal_0/pose");
    RCLCPP_INFO(get_logger(), "  /model/x500_gimbal_0/pose_static");
  }

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr x1_pose_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr x1_pose_static_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr x500_pose_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr x500_pose_static_sub_;

  geometry_msgs::msg::TransformStamped normalizeTransform(
    const geometry_msgs::msg::TransformStamped & input)
  {
    auto output = input;

    // Gazebo world frame "default"를 ROS/RViz 기준 "map"으로 통일
    if (output.header.frame_id == "default" || output.header.frame_id.empty()) {
      output.header.frame_id = "map";
    }

    // child frame이 비어 있으면 TF에 넣으면 안 됨
    // 빈 child_frame_id는 callback에서 필터링한다.
    output.header.stamp = this->now();

    return output;
  }

  bool isValidTransform(const geometry_msgs::msg::TransformStamped & tf)
  {
    if (tf.header.frame_id.empty()) {
      return false;
    }

    if (tf.child_frame_id.empty()) {
      return false;
    }

    if (tf.header.frame_id == tf.child_frame_id) {
      return false;
    }

    return true;
  }

  void dynamicPoseCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    std::vector<geometry_msgs::msg::TransformStamped> valid_transforms;
    valid_transforms.reserve(msg->transforms.size());

    for (const auto & tf_in : msg->transforms) {
      auto tf_out = normalizeTransform(tf_in);

      if (!isValidTransform(tf_out)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Ignored invalid dynamic TF. parent='%s', child='%s'",
          tf_out.header.frame_id.c_str(),
          tf_out.child_frame_id.c_str()
        );
        continue;
      }

      valid_transforms.push_back(tf_out);
    }

    if (!valid_transforms.empty()) {
      dynamic_broadcaster_->sendTransform(valid_transforms);
    }
  }

  void staticPoseCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    std::vector<geometry_msgs::msg::TransformStamped> valid_transforms;
    valid_transforms.reserve(msg->transforms.size());

    for (const auto & tf_in : msg->transforms) {
      auto tf_out = normalizeTransform(tf_in);

      if (!isValidTransform(tf_out)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Ignored invalid static TF. parent='%s', child='%s'",
          tf_out.header.frame_id.c_str(),
          tf_out.child_frame_id.c_str()
        );
        continue;
      }

      valid_transforms.push_back(tf_out);
    }

    if (!valid_transforms.empty()) {
      static_broadcaster_->sendTransform(valid_transforms);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
