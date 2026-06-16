// mission_timer_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MissionTimerNode : public rclcpp::Node
{
public:
  MissionTimerNode()
  : Node("mission_timer_node"),
    world_frame_("map"),
    robot_frame_("X1_asp"),
    gimbal_frame_("x500_gimbal_0"),
    start_threshold_(0.1),     // [m]
    stop_threshold_(1.0)       // [m]
  {
    // ── TF2 준비 ──────────────────────────────────────────────
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);          // 전용 쓰레드
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ── disarm 토픽 구독 ─────────────────────────────────────
    disarm_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/command/disarm", 10,
      std::bind(&MissionTimerNode::disarmCb, this, std::placeholders::_1));

    // ── 주기적 검사 타이머(20 Hz) ───────────────────────────
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&MissionTimerNode::timerCb, this));

    RCLCPP_INFO(get_logger(), "MissionTimerNode ready");
  }

private:
  // ── 콜백: 주기적 거리 확인 ─────────────────────────────────
  void timerCb()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        world_frame_, robot_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.05));   // 50 ms timeout
    } catch (const tf2::TransformException & e) {
      RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", e.what());
      return;
    }

    const auto & p = tf.transform.translation;

    // ① 최초 위치 저장
    if (!init_pose_set_) {
      init_x_ = p.x;  init_y_ = p.y;  init_z_ = p.z;
      init_pose_set_ = true;
      return;
    }

    // ② 미션 시작 판정
    if (!mission_started_) {
      double dist = euclid(p.x, p.y, p.z, init_x_, init_y_, init_z_);
      if (dist >= start_threshold_) {
        mission_started_ = true;
        start_time_ = this->now();
        RCLCPP_INFO(get_logger(),
          "📍 Mission started (%.2f m from origin)", dist);
      }
    }
  }

  // ── 콜백: disarm 수신 시 종료 조건 확인 ────────────────────
  void disarmCb(const std_msgs::msg::Bool::SharedPtr /*msg*/)
  {
    if (!mission_started_ || mission_finished_) return;

    geometry_msgs::msg::TransformStamped tf_robot, tf_gimbal;
    try {
      tf_robot  = tf_buffer_->lookupTransform(world_frame_, robot_frame_,  tf2::TimePointZero,
                                              tf2::durationFromSec(0.05));
      tf_gimbal = tf_buffer_->lookupTransform(world_frame_, gimbal_frame_, tf2::TimePointZero,
                                              tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "TF lookup (end) failed: %s", e.what());
      return;
    }

    const auto & pr = tf_robot.transform.translation;
    const auto & pg = tf_gimbal.transform.translation;

    double dist = euclid(pr.x, pr.y, pr.z, pg.x, pg.y, pg.z);

    if (dist <= stop_threshold_) {
      mission_finished_ = true;
      auto end_time = this->now();
      double elapsed = (end_time - start_time_).seconds();

      RCLCPP_INFO(get_logger(),
        "✅ Mission finished (dist=%.2f m). Total time: %.2f s", dist, elapsed);
    }
  }

  // ── 유틸 ───────────────────────────────────────────────────
  static double euclid(double x1,double y1,double z1,
                       double x2,double y2,double z2)
  {
    return std::hypot(std::hypot(x1 - x2, y1 - y2), z1 - z2);
  }

  // ── 멤버 변수 ──────────────────────────────────────────────
  std::string world_frame_, robot_frame_, gimbal_frame_;
  double start_threshold_, stop_threshold_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr disarm_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 상태 플래그
  bool init_pose_set_{false};
  bool mission_started_{false};
  bool mission_finished_{false};

  // 좌표 & 시간
  double init_x_, init_y_, init_z_;
  rclcpp::Time start_time_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionTimerNode>());
  rclcpp::shutdown();
  return 0;
}
