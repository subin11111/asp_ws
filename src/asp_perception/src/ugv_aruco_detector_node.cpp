#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <exception>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

class UgvArucoDetectorNode : public rclcpp::Node
{
public:
  UgvArucoDetectorNode()
  : Node("ugv_aruco_detector_node")
  {
    declare_parameters();
    read_parameters();
    configure_dictionary();

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&UgvArucoDetectorNode::image_callback, this, std::placeholders::_1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&UgvArucoDetectorNode::camera_info_callback, this, std::placeholders::_1));

    mission_event_sub_ = create_subscription<std_msgs::msg::String>(
      ugv_mission_event_topic_, 10,
      std::bind(&UgvArucoDetectorNode::mission_event_callback, this, std::placeholders::_1));

    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(
      annotated_image_topic_, rclcpp::SensorDataQoS());
    marker_id_pub_ = create_publisher<std_msgs::msg::Int32>(marker_id_topic_, 10);
    marker_detections_pub_ = create_publisher<std_msgs::msg::String>(marker_detections_topic_, 10);
    mission2_trigger_pub_ = create_publisher<std_msgs::msg::Bool>(mission2_trigger_topic_, 10);
    if (publish_manager_trigger_) {
      mission2_manager_trigger_pub_ =
        create_publisher<std_msgs::msg::Bool>(mission2_manager_trigger_topic_, 10);
    }

    RCLCPP_INFO(get_logger(), "UGV ArUco detector started.");
    RCLCPP_INFO(get_logger(), "image_topic: %s", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "camera_info_topic: %s", camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "annotated_image_topic: %s", annotated_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "target_marker_id: %d", target_marker_id_);
    RCLCPP_INFO(get_logger(), "dictionary: %s", dictionary_name_.c_str());
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>(
      "image_topic", "/world/default/model/X1_asp/link/base_link/sensor/camera_front/image");
    declare_parameter<std::string>(
      "camera_info_topic",
      "/world/default/model/X1_asp/link/base_link/sensor/camera_front/camera_info");
    declare_parameter<std::string>("annotated_image_topic", "/perception/aruco/annotated");
    declare_parameter<std::string>("marker_id_topic", "/perception/marker_id");
    declare_parameter<std::string>("marker_detections_topic", "/perception/marker_detections");
    declare_parameter<std::string>("mission2_trigger_topic", "/perception/mission2_trigger");
    declare_parameter<std::string>(
      "mission2_manager_trigger_topic", "/mission/mission2_trigger");
    declare_parameter<bool>("publish_manager_trigger", false);
    declare_parameter<std::string>("ugv_mission_event_topic", "/ugv/mission_event");
    declare_parameter<std::string>("required_ugv_event", "MISSION2_START_REACHED");
    declare_parameter<bool>("require_ugv_event", false);
    declare_parameter<int>("target_marker_id", 0);
    declare_parameter<double>("marker_size_m", 0.5);
    declare_parameter<std::string>("dictionary", "DICT_4X4_50");
    declare_parameter<int>("consecutive_detection_threshold", 3);
    declare_parameter<bool>("trigger_once", true);
    declare_parameter<bool>("publish_debug_image", true);
    declare_parameter<bool>("use_camera_info", true);
  }

  void read_parameters()
  {
    get_parameter("image_topic", image_topic_);
    get_parameter("camera_info_topic", camera_info_topic_);
    get_parameter("annotated_image_topic", annotated_image_topic_);
    get_parameter("marker_id_topic", marker_id_topic_);
    get_parameter("marker_detections_topic", marker_detections_topic_);
    get_parameter("mission2_trigger_topic", mission2_trigger_topic_);
    get_parameter("mission2_manager_trigger_topic", mission2_manager_trigger_topic_);
    get_parameter("publish_manager_trigger", publish_manager_trigger_);
    get_parameter("ugv_mission_event_topic", ugv_mission_event_topic_);
    get_parameter("required_ugv_event", required_ugv_event_);
    get_parameter("require_ugv_event", require_ugv_event_);
    get_parameter("target_marker_id", target_marker_id_);
    get_parameter("marker_size_m", marker_size_m_);
    get_parameter("dictionary", dictionary_name_);
    get_parameter("consecutive_detection_threshold", consecutive_detection_threshold_);
    get_parameter("trigger_once", trigger_once_);
    get_parameter("publish_debug_image", publish_debug_image_);
    get_parameter("use_camera_info", use_camera_info_);

    consecutive_detection_threshold_ = std::max(1, consecutive_detection_threshold_);
    marker_size_m_ = std::max(0.0, marker_size_m_);
  }

  void configure_dictionary()
  {
    static const std::map<std::string, int> dictionaries = {
      {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
      {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
      {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
      {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
      {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
      {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    };

    const auto iter = dictionaries.find(dictionary_name_);
    const int dictionary_id =
      iter == dictionaries.end() ? cv::aruco::DICT_4X4_50 : iter->second;
    if (iter == dictionaries.end()) {
      RCLCPP_WARN(
        get_logger(), "Unknown dictionary '%s'. Falling back to DICT_4X4_50.",
        dictionary_name_.c_str());
      dictionary_name_ = "DICT_4X4_50";
    }

    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id);
    detector_params_ = cv::aruco::DetectorParameters::create();
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!use_camera_info_) {
      return;
    }
    latest_camera_info_ = *msg;
    camera_info_received_ = true;
  }

  void mission_event_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == required_ugv_event_) {
      ugv_event_ready_ = true;
      RCLCPP_INFO(get_logger(), "Required UGV mission event received: %s", msg->data.c_str());
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!msg || msg->data.empty() || msg->width == 0 || msg->height == 0) {
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "cv_bridge conversion failed: %s", ex.what());
      return;
    }

    if (cv_ptr->image.empty()) {
      return;
    }

    cv::Mat annotated;
    cv::Mat gray;
    prepare_images(cv_ptr->image, msg->encoding, annotated, gray);

    if (gray.empty() || annotated.empty()) {
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_);

    const bool has_target = process_detections(ids);

    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(annotated, corners, ids);
    }

    if (has_target) {
      cv::putText(
        annotated, "MISSION2 TRIGGER", cv::Point(24, 48), cv::FONT_HERSHEY_SIMPLEX,
        1.0, cv::Scalar(0, 0, 255), 2);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "Target marker %d detected (%d/%d).",
        target_marker_id_, consecutive_target_count_, consecutive_detection_threshold_);
    }

    if (publish_debug_image_) {
      publish_annotated_image(msg->header, annotated);
    }
  }

  void prepare_images(
    const cv::Mat & input,
    const std::string & encoding,
    cv::Mat & annotated,
    cv::Mat & gray) const
  {
    if (input.channels() == 1) {
      gray = input.clone();
      cv::cvtColor(input, annotated, cv::COLOR_GRAY2BGR);
      return;
    }

    if (input.channels() == 3) {
      if (encoding == sensor_msgs::image_encodings::RGB8) {
        cv::cvtColor(input, annotated, cv::COLOR_RGB2BGR);
      } else {
        annotated = input.clone();
      }
      cv::cvtColor(annotated, gray, cv::COLOR_BGR2GRAY);
      return;
    }

    if (input.channels() == 4) {
      if (encoding == sensor_msgs::image_encodings::RGBA8) {
        cv::cvtColor(input, annotated, cv::COLOR_RGBA2BGR);
      } else {
        cv::cvtColor(input, annotated, cv::COLOR_BGRA2BGR);
      }
      cv::cvtColor(annotated, gray, cv::COLOR_BGR2GRAY);
    }
  }

  bool process_detections(const std::vector<int> & ids)
  {
    bool has_target = false;
    std::ostringstream detection_text;
    detection_text << "ids=[";

    for (size_t i = 0; i < ids.size(); ++i) {
      if (i > 0) {
        detection_text << ",";
      }
      detection_text << ids[i];

      std_msgs::msg::Int32 marker_id_msg;
      marker_id_msg.data = ids[i];
      marker_id_pub_->publish(marker_id_msg);

      if (ids[i] == target_marker_id_) {
        has_target = true;
      }
    }

    detection_text << "]";
    if (!ids.empty()) {
      std_msgs::msg::String msg;
      msg.data = detection_text.str();
      marker_detections_pub_->publish(msg);
    }

    if (!has_target) {
      consecutive_target_count_ = 0;
      return false;
    }

    ++consecutive_target_count_;
    if (consecutive_target_count_ >= consecutive_detection_threshold_) {
      maybe_publish_trigger();
    }

    return true;
  }

  void maybe_publish_trigger()
  {
    if (trigger_once_ && trigger_published_) {
      return;
    }

    if (require_ugv_event_ && !ugv_event_ready_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Target marker detected, waiting for required UGV event '%s'.",
        required_ugv_event_.c_str());
      return;
    }

    std_msgs::msg::Bool trigger_msg;
    trigger_msg.data = true;
    mission2_trigger_pub_->publish(trigger_msg);
    if (publish_manager_trigger_ && mission2_manager_trigger_pub_) {
      mission2_manager_trigger_pub_->publish(trigger_msg);
    }
    trigger_published_ = true;

    RCLCPP_WARN(
      get_logger(), "MISSION2 TRIGGER published by marker id %d.", target_marker_id_);
  }

  void publish_annotated_image(const std_msgs::msg::Header & header, const cv::Mat & annotated)
  {
    cv_bridge::CvImage out_msg;
    out_msg.header = header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = annotated;
    annotated_pub_->publish(*out_msg.toImageMsg());
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string annotated_image_topic_;
  std::string marker_id_topic_;
  std::string marker_detections_topic_;
  std::string mission2_trigger_topic_;
  std::string mission2_manager_trigger_topic_;
  bool publish_manager_trigger_{false};
  std::string ugv_mission_event_topic_;
  std::string required_ugv_event_;
  bool require_ugv_event_{false};
  int target_marker_id_{0};
  double marker_size_m_{0.5};
  std::string dictionary_name_{"DICT_4X4_50"};
  int consecutive_detection_threshold_{3};
  bool trigger_once_{true};
  bool publish_debug_image_{true};
  bool use_camera_info_{true};

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_event_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr marker_id_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr marker_detections_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission2_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission2_manager_trigger_pub_;

  sensor_msgs::msg::CameraInfo latest_camera_info_;
  bool camera_info_received_{false};
  bool ugv_event_ready_{false};
  bool trigger_published_{false};
  int consecutive_target_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UgvArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
