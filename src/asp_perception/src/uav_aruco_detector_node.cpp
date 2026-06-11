#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>

struct MarkerRecord
{
  int id{};
  double stamp_sec{};
  double map_x{std::numeric_limits<double>::quiet_NaN()};
  double map_y{std::numeric_limits<double>::quiet_NaN()};
  double map_z{std::numeric_limits<double>::quiet_NaN()};
};

class UavArucoDetectorNode : public rclcpp::Node
{
public:
  UavArucoDetectorNode()
  : Node("uav_aruco_detector_node"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    declare_parameters();
    read_parameters();
    configure_dictionary();

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&UavArucoDetectorNode::image_callback, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&UavArucoDetectorNode::camera_info_callback, this, std::placeholders::_1));

    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(
      annotated_image_topic_, rclcpp::SensorDataQoS());
    marker_id_pub_ = create_publisher<std_msgs::msg::Int32>(marker_id_topic_, 10);
    detections_pub_ = create_publisher<std_msgs::msg::String>(marker_detections_topic_, 10);
    marker_array_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(marker_map_points_topic_, 10);

    prepare_csv();

    RCLCPP_INFO(get_logger(), "UAV ArUco detector started.");
    RCLCPP_INFO(get_logger(), "image_topic: %s", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "camera_info_topic: %s", camera_info_topic_.c_str());
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>(
      "image_topic", "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image");
    declare_parameter<std::string>(
      "camera_info_topic",
      "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info");
    declare_parameter<std::string>("annotated_image_topic", "/perception/uav/aruco/annotated");
    declare_parameter<std::string>("marker_id_topic", "/perception/uav/marker_id");
    declare_parameter<std::string>("marker_detections_topic", "/perception/uav/marker_detections");
    declare_parameter<std::string>("marker_map_points_topic", "/perception/uav/marker_map_points");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("camera_frame", "x500_gimbal_0/camera_link");
    declare_parameter<std::string>("dictionary", "DICT_4X4_50");
    declare_parameter<double>("marker_size_m", 0.5);
    declare_parameter<bool>("publish_debug_image", true);
    declare_parameter<bool>("write_csv", true);
    declare_parameter<bool>("publish_detections_without_map", true);
    declare_parameter<int>("min_marker_id", 0);
    declare_parameter<int>("max_marker_id", 49);
    declare_parameter<std::string>(
      "csv_path", "/home/desktop1/ros2_ws/mission_logs/uav_marker_detections.csv");
    declare_parameter<double>("duplicate_suppression_distance_m", 0.5);
    declare_parameter<double>("duplicate_suppression_time_sec", 2.0);
  }

  void read_parameters()
  {
    get_parameter("image_topic", image_topic_);
    get_parameter("camera_info_topic", camera_info_topic_);
    get_parameter("annotated_image_topic", annotated_image_topic_);
    get_parameter("marker_id_topic", marker_id_topic_);
    get_parameter("marker_detections_topic", marker_detections_topic_);
    get_parameter("marker_map_points_topic", marker_map_points_topic_);
    get_parameter("map_frame", map_frame_);
    get_parameter("camera_frame", camera_frame_);
    get_parameter("dictionary", dictionary_name_);
    get_parameter("marker_size_m", marker_size_m_);
    get_parameter("publish_debug_image", publish_debug_image_);
    get_parameter("write_csv", write_csv_);
    get_parameter("publish_detections_without_map", publish_detections_without_map_);
    get_parameter("min_marker_id", min_marker_id_);
    get_parameter("max_marker_id", max_marker_id_);
    get_parameter("csv_path", csv_path_);
    get_parameter("duplicate_suppression_distance_m", duplicate_distance_);
    get_parameter("duplicate_suppression_time_sec", duplicate_time_sec_);
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
    dictionary_ = cv::aruco::getPredefinedDictionary(
      iter == dictionaries.end() ? cv::aruco::DICT_4X4_50 : iter->second);
    detector_params_ = cv::aruco::DetectorParameters::create();
  }

  void prepare_csv()
  {
    if (!write_csv_) {
      return;
    }
    const auto slash_pos = csv_path_.find_last_of('/');
    if (slash_pos != std::string::npos) {
      const std::string directory = csv_path_.substr(0, slash_pos);
      mkdir(directory.c_str(), 0755);
    }
    std::ifstream existing(csv_path_);
    if (!existing.good()) {
      std::ofstream file(csv_path_, std::ios::app);
      file << "stamp,marker_id,source,camera_x,camera_y,camera_z,map_x,map_y,map_z\n";
    }
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    camera_info_ = *msg;
    camera_info_received_ = true;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!msg || msg->data.empty()) {
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge failed: %s", ex.what());
      return;
    }

    cv::Mat annotated;
    cv::Mat gray;
    prepare_images(cv_ptr->image, msg->encoding, annotated, gray);
    if (gray.empty()) {
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_);

    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    estimate_pose(corners, rvecs, tvecs);

    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(annotated, corners, ids);
    }

    visualization_msgs::msg::MarkerArray marker_array;
    const double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();

    for (size_t i = 0; i < ids.size(); ++i) {
      const cv::Vec3d tvec = i < tvecs.size() ?
        tvecs[i] :
        cv::Vec3d(
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN());

      auto map_point = transform_to_map(tvec, msg->header.stamp);
      if (!is_marker_id_in_range(ids[i])) {
        continue;
      }
      const bool has_map = point_has_map(map_point);
      if (!has_map && !publish_detections_without_map_) {
        continue;
      }
      const cv::Point2f center = marker_center(corners[i]);
      std_msgs::msg::String text_msg;
      text_msg.data = make_detection_json(
        ids[i], stamp_sec, center, msg->width, msg->height, tvec, map_point, has_map);
      detections_pub_->publish(text_msg);

      publish_marker_id(ids[i]);
      add_marker(marker_array, ids[i], map_point, msg->header.stamp);
      maybe_write_csv(stamp_sec, ids[i], tvec, map_point);
    }

    if (!ids.empty()) {
      marker_array_pub_->publish(marker_array);
    }

    if (publish_debug_image_) {
      publish_annotated(msg->header, annotated);
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
    } else if (input.channels() == 3) {
      if (encoding == sensor_msgs::image_encodings::RGB8) {
        cv::cvtColor(input, annotated, cv::COLOR_RGB2BGR);
      } else {
        annotated = input.clone();
      }
      cv::cvtColor(annotated, gray, cv::COLOR_BGR2GRAY);
    } else if (input.channels() == 4) {
      cv::cvtColor(input, annotated, cv::COLOR_BGRA2BGR);
      cv::cvtColor(annotated, gray, cv::COLOR_BGR2GRAY);
    }
  }

  void estimate_pose(
    const std::vector<std::vector<cv::Point2f>> & corners,
    std::vector<cv::Vec3d> & rvecs,
    std::vector<cv::Vec3d> & tvecs)
  {
    if (!camera_info_received_ || corners.empty() || marker_size_m_ <= 0.0) {
      return;
    }

    cv::Mat camera_matrix(3, 3, CV_64F, const_cast<double *>(camera_info_.k.data()));
    cv::Mat dist_coeffs(camera_info_.d, true);
    try {
      cv::aruco::estimatePoseSingleMarkers(
        corners, marker_size_m_, camera_matrix.clone(), dist_coeffs.clone(), rvecs, tvecs);
    } catch (const cv::Exception & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "pose estimation failed: %s", ex.what());
    }
  }

  geometry_msgs::msg::PointStamped transform_to_map(
    const cv::Vec3d & tvec,
    const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::PointStamped camera_point;
    camera_point.header.stamp = stamp;
    camera_point.header.frame_id = camera_frame_;
    camera_point.point.x = tvec[0];
    camera_point.point.y = tvec[1];
    camera_point.point.z = tvec[2];

    geometry_msgs::msg::PointStamped map_point = camera_point;
    map_point.header.frame_id = map_frame_;
    if (!std::isfinite(tvec[0]) || !std::isfinite(tvec[1]) || !std::isfinite(tvec[2])) {
      map_point.point.x = std::numeric_limits<double>::quiet_NaN();
      map_point.point.y = std::numeric_limits<double>::quiet_NaN();
      map_point.point.z = std::numeric_limits<double>::quiet_NaN();
      return map_point;
    }
    try {
      map_point = tf_buffer_->transform(camera_point, map_frame_, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "marker map transform failed: %s", ex.what());
      map_point.point.x = std::numeric_limits<double>::quiet_NaN();
      map_point.point.y = std::numeric_limits<double>::quiet_NaN();
      map_point.point.z = std::numeric_limits<double>::quiet_NaN();
    }
    return map_point;
  }

  void publish_marker_id(int id)
  {
    if (!is_marker_id_in_range(id)) {
      return;
    }
    std_msgs::msg::Int32 msg;
    msg.data = id;
    marker_id_pub_->publish(msg);
  }

  bool is_marker_id_in_range(int id) const
  {
    return min_marker_id_ <= id && id <= max_marker_id_;
  }

  bool point_has_map(const geometry_msgs::msg::PointStamped & point) const
  {
    return std::isfinite(point.point.x) &&
      std::isfinite(point.point.y) &&
      std::isfinite(point.point.z);
  }

  cv::Point2f marker_center(const std::vector<cv::Point2f> & marker_corners) const
  {
    cv::Point2f center(0.0f, 0.0f);
    if (marker_corners.empty()) {
      return center;
    }
    for (const auto & corner : marker_corners) {
      center += corner;
    }
    center.x /= static_cast<float>(marker_corners.size());
    center.y /= static_cast<float>(marker_corners.size());
    return center;
  }

  std::string json_number_or_null(double value) const
  {
    if (!std::isfinite(value)) {
      return "null";
    }
    std::ostringstream stream;
    stream << value;
    return stream.str();
  }

  std::string make_detection_json(
    int id,
    double stamp_sec,
    const cv::Point2f & center,
    uint32_t image_width,
    uint32_t image_height,
    const cv::Vec3d & camera_point,
    const geometry_msgs::msg::PointStamped & map_point,
    bool has_map) const
  {
    std::ostringstream stream;
    stream << "{"
           << "\"marker_id\":" << id << ","
           << "\"source\":\"uav_camera\","
           << "\"stamp\":" << stamp_sec << ","
           << "\"center_u\":" << center.x << ","
           << "\"center_v\":" << center.y << ","
           << "\"image_width\":" << image_width << ","
           << "\"image_height\":" << image_height << ","
           << "\"camera_x\":" << json_number_or_null(camera_point[0]) << ","
           << "\"camera_y\":" << json_number_or_null(camera_point[1]) << ","
           << "\"camera_z\":" << json_number_or_null(camera_point[2]) << ","
           << "\"map_x\":" << json_number_or_null(map_point.point.x) << ","
           << "\"map_y\":" << json_number_or_null(map_point.point.y) << ","
           << "\"map_z\":" << json_number_or_null(map_point.point.z) << ","
           << "\"has_map\":" << (has_map ? "true" : "false")
           << "}";
    return stream.str();
  }

  void add_marker(
    visualization_msgs::msg::MarkerArray & marker_array,
    int id,
    const geometry_msgs::msg::PointStamped & point,
    const builtin_interfaces::msg::Time & stamp)
  {
    if (!std::isfinite(point.point.x) || !std::isfinite(point.point.y) || !std::isfinite(point.point.z)) {
      return;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = map_frame_;
    marker.ns = "uav_aruco";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = point.point;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 0.1;
    marker.color.g = 0.4;
    marker.color.b = 1.0;
    marker_array.markers.push_back(marker);
  }

  void maybe_write_csv(
    double stamp_sec,
    int id,
    const cv::Vec3d & camera_point,
    const geometry_msgs::msg::PointStamped & map_point)
  {
    if (!write_csv_ || is_duplicate(stamp_sec, id, map_point)) {
      return;
    }
    MarkerRecord record;
    record.id = id;
    record.stamp_sec = stamp_sec;
    record.map_x = map_point.point.x;
    record.map_y = map_point.point.y;
    record.map_z = map_point.point.z;
    records_.push_back(record);

    std::ofstream file(csv_path_, std::ios::app);
    file << stamp_sec << "," << id << ",uav,"
         << camera_point[0] << "," << camera_point[1] << "," << camera_point[2] << ","
         << map_point.point.x << "," << map_point.point.y << "," << map_point.point.z << "\n";
  }

  bool is_duplicate(double stamp_sec, int id, const geometry_msgs::msg::PointStamped & point) const
  {
    for (const auto & record : records_) {
      if (record.id != id) {
        continue;
      }
      if (std::abs(stamp_sec - record.stamp_sec) > duplicate_time_sec_) {
        continue;
      }
      if (!std::isfinite(point.point.x) || !std::isfinite(record.map_x)) {
        return true;
      }
      const double dx = point.point.x - record.map_x;
      const double dy = point.point.y - record.map_y;
      const double dz = point.point.z - record.map_z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) <= duplicate_distance_) {
        return true;
      }
    }
    return false;
  }

  void publish_annotated(const std_msgs::msg::Header & header, const cv::Mat & annotated)
  {
    cv_bridge::CvImage out;
    out.header = header;
    out.encoding = sensor_msgs::image_encodings::BGR8;
    out.image = annotated;
    annotated_pub_->publish(*out.toImageMsg());
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string annotated_image_topic_;
  std::string marker_id_topic_;
  std::string marker_detections_topic_;
  std::string marker_map_points_topic_;
  std::string map_frame_;
  std::string camera_frame_;
  std::string dictionary_name_;
  double marker_size_m_{0.5};
  bool publish_debug_image_{true};
  bool write_csv_{true};
  bool publish_detections_without_map_{true};
  int min_marker_id_{0};
  int max_marker_id_{49};
  std::string csv_path_;
  double duplicate_distance_{0.5};
  double duplicate_time_sec_{2.0};

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  sensor_msgs::msg::CameraInfo camera_info_;
  bool camera_info_received_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr marker_id_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detections_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  std::vector<MarkerRecord> records_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UavArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
