#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include <cmath>
#include <fstream>
#include <filesystem>
#include <string>
#include <vector>

/*
 * UAV ArUco Detector Node
 *
 * Subscribe:
 *   /uav/camera/image_raw
 *   /uav/camera/camera_info
 *   /mission_state
 *
 * Publish:
 *   /aruco/marker_pose       camera 기준 상대좌표
 *   /aruco/marker_pose_map   map 기준 절대좌표
 *   /aruco/marker_id         marker ID
 *   /offboard_control/image_proc  ArUco 검출 표시 이미지
 */

class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode()
  : Node("aruco_detector")
  {
    declareParameters();
    loadParameters();
    initializeArucoDictionary();
    initializeTf();
    initializeCsvLog();
    initializeRosInterfaces();

    RCLCPP_INFO(get_logger(), "======================================");
    RCLCPP_INFO(get_logger(), " UAV ArUco Detector Node Started");
    RCLCPP_INFO(get_logger(), " pose topic        : /aruco/marker_pose");
    RCLCPP_INFO(get_logger(), " map pose topic    : /aruco/marker_pose_map");
    RCLCPP_INFO(get_logger(), " marker id topic   : /aruco/marker_id");
    RCLCPP_INFO(get_logger(), " image proc topic  : %s", output_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), " target frame      : %s", target_frame_.c_str());
    RCLCPP_INFO(get_logger(), " camera frame      : %s", camera_frame_id_.c_str());
    RCLCPP_INFO(get_logger(), " marker_size       : %.3f m", marker_size_);
    RCLCPP_INFO(get_logger(), " active_state      : %d", active_mission_state_);
    RCLCPP_INFO(get_logger(), "======================================");
  }

  ~ArucoDetectorNode() override
  {
    if (csv_file_.is_open()) {
      csv_file_.close();
    }
  }

private:
  double marker_size_{0.5};
  int active_mission_state_{1};
  bool detect_only_active_state_{true};
  int target_marker_id_{-1};

  std::string dictionary_name_{"DICT_4X4_50"};
  std::string default_camera_frame_id_{"x500_gimbal_0/camera_link"};
  std::string camera_frame_id_;
  std::string target_frame_{"map"};
  std::string output_image_topic_{"/offboard_control/image_proc"};

  bool publish_map_pose_{true};
  bool write_csv_{true};
  std::string csv_file_path_{""};
  bool convert_opencv_to_ros_{true};

  int mission_state_{0};
  bool camera_info_received_{false};

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::ofstream csv_file_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mission_state_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_map_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr marker_id_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_image_pub_;

  void declareParameters()
  {
    declare_parameter<double>("marker_size", 0.5);
    declare_parameter<int>("active_mission_state", 1);
    declare_parameter<bool>("detect_only_active_state", true);
    declare_parameter<int>("target_marker_id", -1);
    declare_parameter<std::string>("dictionary", "DICT_4X4_50");

    // 최종 성공한 TF frame 기준
    declare_parameter<std::string>("default_camera_frame_id", "x500_gimbal_0/camera_link");

    declare_parameter<std::string>("target_frame", "map");
    declare_parameter<bool>("publish_map_pose", true);
    declare_parameter<bool>("write_csv", true);
    declare_parameter<std::string>("csv_file_path", "");
    declare_parameter<bool>("convert_opencv_to_ros", true);

    // RViz Marker Detected 패널용 처리 이미지 토픽
    declare_parameter<std::string>("output_image_topic", "/offboard_control/image_proc");
  }

  void loadParameters()
  {
    marker_size_ = get_parameter("marker_size").as_double();
    active_mission_state_ = get_parameter("active_mission_state").as_int();
    detect_only_active_state_ = get_parameter("detect_only_active_state").as_bool();
    target_marker_id_ = get_parameter("target_marker_id").as_int();
    dictionary_name_ = get_parameter("dictionary").as_string();
    default_camera_frame_id_ = get_parameter("default_camera_frame_id").as_string();
    target_frame_ = get_parameter("target_frame").as_string();
    publish_map_pose_ = get_parameter("publish_map_pose").as_bool();
    write_csv_ = get_parameter("write_csv").as_bool();
    csv_file_path_ = get_parameter("csv_file_path").as_string();
    convert_opencv_to_ros_ = get_parameter("convert_opencv_to_ros").as_bool();
    output_image_topic_ = get_parameter("output_image_topic").as_string();

    camera_frame_id_ = default_camera_frame_id_;
  }

  void initializeArucoDictionary()
  {
    int dictionary_id = cv::aruco::DICT_4X4_50;

    if (dictionary_name_ == "DICT_4X4_100") {
      dictionary_id = cv::aruco::DICT_4X4_100;
    } else if (dictionary_name_ == "DICT_5X5_50") {
      dictionary_id = cv::aruco::DICT_5X5_50;
    } else if (dictionary_name_ == "DICT_5X5_100") {
      dictionary_id = cv::aruco::DICT_5X5_100;
    } else if (dictionary_name_ == "DICT_6X6_50") {
      dictionary_id = cv::aruco::DICT_6X6_50;
    } else if (dictionary_name_ == "DICT_6X6_100") {
      dictionary_id = cv::aruco::DICT_6X6_100;
    }

    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id);
    detector_params_ = cv::aruco::DetectorParameters::create();
  }

  void initializeTf()
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void initializeCsvLog()
  {
    if (!write_csv_) {
      return;
    }

    try {
      if (csv_file_path_.empty()) {
        std::filesystem::path log_dir = std::filesystem::current_path() / "aruco_log";
        std::filesystem::create_directories(log_dir);
        csv_file_path_ = (log_dir / "marker_detections.csv").string();
      } else {
        std::filesystem::path p(csv_file_path_);
        if (p.has_parent_path()) {
          std::filesystem::create_directories(p.parent_path());
        }
      }

      const bool need_header =
        !std::filesystem::exists(csv_file_path_) ||
        std::filesystem::file_size(csv_file_path_) == 0;

      csv_file_.open(csv_file_path_, std::ios::app);
      if (!csv_file_.is_open()) {
        RCLCPP_WARN(get_logger(), "Failed to open CSV file: %s", csv_file_path_.c_str());
        return;
      }

      if (need_header) {
        csv_file_ << "time_sec,marker_id,camera_frame,camera_x,camera_y,camera_z,"
                  << "map_frame,map_x,map_y,map_z\n";
      }

      RCLCPP_INFO(get_logger(), "CSV log file: %s", csv_file_path_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "CSV initialization failed: %s", e.what());
    }
  }

  void initializeRosInterfaces()
  {
    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/uav/camera/image_raw",
      image_qos,
      std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1)
    );

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/uav/camera/camera_info",
      image_qos,
      std::bind(&ArucoDetectorNode::cameraInfoCallback, this, std::placeholders::_1)
    );

    mission_state_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/mission_state",
      rclcpp::QoS(10),
      std::bind(&ArucoDetectorNode::missionStateCallback, this, std::placeholders::_1)
    );

    marker_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/aruco/marker_pose",
      rclcpp::QoS(10)
    );

    marker_pose_map_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/aruco/marker_pose_map",
      rclcpp::QoS(10)
    );

    marker_id_pub_ = create_publisher<std_msgs::msg::Int32>(
      "/aruco/marker_id",
      rclcpp::QoS(10)
    );

    output_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      output_image_topic_,
      image_qos
    );
  }

  void missionStateCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    mission_state_ = msg->data;
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "mission_state = %d",
      mission_state_
    );
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (camera_info_received_) {
      return;
    }

    camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data())).clone();
    dist_coeffs_ = cv::Mat(
      static_cast<int>(msg->d.size()),
      1,
      CV_64F,
      const_cast<double *>(msg->d.data())
    ).clone();

    /*
     * 중요:
     * camera_info의 frame_id는 x500_gimbal_0/camera_link/camera로 들어올 수 있지만,
     * 현재 TF tree에서 map과 연결된 frame은 x500_gimbal_0/camera_link이다.
     * 따라서 camera_info의 header.frame_id로 덮어쓰지 않고,
     * YAML/default_camera_frame_id 값을 그대로 사용한다.
     */

    camera_info_received_ = true;

    RCLCPP_INFO(
      get_logger(),
      "CameraInfo received. camera_frame_id = %s",
      camera_frame_id_.c_str()
    );
  }

  void publishProcessedImage(
    cv_bridge::CvImagePtr cv_ptr,
    const rclcpp::Time & stamp)
  {
    if (!output_image_pub_) {
      return;
    }

    auto out_msg = cv_ptr->toImageMsg();
    out_msg->header.stamp = stamp;
    out_msg->header.frame_id = camera_frame_id_;
    output_image_pub_->publish(*out_msg);
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (detect_only_active_state_ && mission_state_ != active_mission_state_) {
      return;
    }

    if (!camera_info_received_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting for /uav/camera/camera_info ..."
      );
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    cv::aruco::detectMarkers(
      cv_ptr->image,
      dictionary_,
      marker_corners,
      marker_ids,
      detector_params_
    );

    // 마커가 없어도 Marker Detected 패널에는 원본 영상이 계속 나오게 publish
    if (marker_ids.empty()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "No ArUco marker detected."
      );

      publishProcessedImage(cv_ptr, this->now());
      return;
    }

    // 검출된 모든 마커 박스 그리기
    cv::aruco::drawDetectedMarkers(cv_ptr->image, marker_corners, marker_ids);

    int selected_index = selectMarker(marker_ids, marker_corners);
    if (selected_index < 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Markers detected, but target_marker_id=%d was not found.",
        target_marker_id_
      );

      publishProcessedImage(cv_ptr, this->now());
      return;
    }

    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;

    cv::aruco::estimatePoseSingleMarkers(
      marker_corners,
      marker_size_,
      camera_matrix_,
      dist_coeffs_,
      rvecs,
      tvecs
    );

    // 모든 검출 마커에 ID 텍스트와 축 표시
    for (size_t i = 0; i < marker_ids.size(); ++i) {
      cv::drawFrameAxes(
        cv_ptr->image,
        camera_matrix_,
        dist_coeffs_,
        rvecs[i],
        tvecs[i],
        marker_size_ * 0.5
      );

      cv::putText(
        cv_ptr->image,
        "ID: " + std::to_string(marker_ids[i]),
        marker_corners[i][0],
        cv::FONT_HERSHEY_SIMPLEX,
        0.8,
        cv::Scalar(0, 255, 0),
        2
      );
    }

    publishMarkerResult(
      this->now(),
      marker_ids[selected_index],
      rvecs[selected_index],
      tvecs[selected_index]
    );

    // RViz Marker Detected 패널용 처리 이미지 publish
    publishProcessedImage(cv_ptr, this->now());
  }

  int selectMarker(
    const std::vector<int> & marker_ids,
    const std::vector<std::vector<cv::Point2f>> & marker_corners) const
  {
    if (marker_ids.empty() || marker_corners.empty()) {
      return -1;
    }

    if (target_marker_id_ >= 0) {
      for (size_t i = 0; i < marker_ids.size(); ++i) {
        if (marker_ids[i] == target_marker_id_) {
          return static_cast<int>(i);
        }
      }
      return -1;
    }

    double best_area = -1.0;
    int best_index = -1;

    for (size_t i = 0; i < marker_corners.size(); ++i) {
      const double area = std::abs(cv::contourArea(marker_corners[i]));
      if (area > best_area) {
        best_area = area;
        best_index = static_cast<int>(i);
      }
    }

    return best_index;
  }

  geometry_msgs::msg::PoseStamped makeCameraPoseMsg(
    const rclcpp::Time & stamp,
    const cv::Vec3d & rvec,
    const cv::Vec3d & tvec
  )
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = camera_frame_id_;

    if (convert_opencv_to_ros_) {
      // OpenCV: x right, y down, z forward
      // ROS 변환: x = -z, y = x, z = -y
      pose_msg.pose.position.x = -tvec[2];
      pose_msg.pose.position.y =  tvec[0];
      pose_msg.pose.position.z = -tvec[1];
    } else {
      pose_msg.pose.position.x = tvec[0];
      pose_msg.pose.position.y = tvec[1];
      pose_msg.pose.position.z = tvec[2];
    }

    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);

    tf2::Matrix3x3 marker_rot_cv(
      rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
      rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
      rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2)
    );

    tf2::Quaternion q_marker_cv;
    marker_rot_cv.getRotation(q_marker_cv);

    if (convert_opencv_to_ros_) {
      tf2::Matrix3x3 R_ros_from_cv;
      R_ros_from_cv.setValue(
        0, 1, 0,
        0, 0, -1,
        -1, 0, 0
      );

      tf2::Quaternion q_ros_from_cv;
      R_ros_from_cv.getRotation(q_ros_from_cv);

      tf2::Quaternion q = q_ros_from_cv * q_marker_cv;
      q.normalize();
      pose_msg.pose.orientation = tf2::toMsg(q);
    } else {
      q_marker_cv.normalize();
      pose_msg.pose.orientation = tf2::toMsg(q_marker_cv);
    }

    return pose_msg;
  }

  void publishMarkerResult(
    const rclcpp::Time & stamp,
    int marker_id,
    const cv::Vec3d & rvec,
    const cv::Vec3d & tvec
  )
  {
    auto camera_pose = makeCameraPoseMsg(stamp, rvec, tvec);

    std_msgs::msg::Int32 id_msg;
    id_msg.data = marker_id;

    marker_id_pub_->publish(id_msg);
    marker_pose_pub_->publish(camera_pose);

    bool map_pose_valid = false;
    geometry_msgs::msg::PoseStamped map_pose;

    if (publish_map_pose_) {
      try {
        map_pose = tf_buffer_->transform(
          camera_pose,
          target_frame_,
          tf2::durationFromSec(0.2)
        );

        marker_pose_map_pub_->publish(map_pose);
        map_pose_valid = true;

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "Published /aruco/marker_pose_map | id=%d | x=%.3f y=%.3f z=%.3f frame=%s",
          marker_id,
          map_pose.pose.position.x,
          map_pose.pose.position.y,
          map_pose.pose.position.z,
          map_pose.header.frame_id.c_str()
        );
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Failed to transform marker pose from '%s' to '%s': %s",
          camera_pose.header.frame_id.c_str(),
          target_frame_.c_str(),
          ex.what()
        );
      }
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "Published /aruco/marker_pose | id=%d | x=%.3f y=%.3f z=%.3f frame=%s",
      marker_id,
      camera_pose.pose.position.x,
      camera_pose.pose.position.y,
      camera_pose.pose.position.z,
      camera_pose.header.frame_id.c_str()
    );

    writeCsvRow(marker_id, camera_pose, map_pose, map_pose_valid);
  }

  void writeCsvRow(
    int marker_id,
    const geometry_msgs::msg::PoseStamped & camera_pose,
    const geometry_msgs::msg::PoseStamped & map_pose,
    bool map_pose_valid
  )
  {
    if (!write_csv_ || !csv_file_.is_open()) {
      return;
    }

    const double time_sec = now().seconds();

    csv_file_ << time_sec << ","
              << marker_id << ","
              << camera_pose.header.frame_id << ","
              << camera_pose.pose.position.x << ","
              << camera_pose.pose.position.y << ","
              << camera_pose.pose.position.z << ",";

    if (map_pose_valid) {
      csv_file_ << map_pose.header.frame_id << ","
                << map_pose.pose.position.x << ","
                << map_pose.pose.position.y << ","
                << map_pose.pose.position.z << "\n";
    } else {
      csv_file_ << target_frame_ << ",nan,nan,nan\n";
    }

    csv_file_.flush();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
