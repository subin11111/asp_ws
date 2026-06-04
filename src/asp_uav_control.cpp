#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <limits>

// CSV에서 읽어올 웨이포인트 구조체
struct Waypoint {
    double x;
    double y;
    double z;
    double yaw_deg;
    double gimbal_pitch_deg;
    double hold_sec;
    std::string tag;
};

class AspUavControlNode : public rclcpp::Node
{
public:
    AspUavControlNode()
    : Node("asp_uav_control_node"), current_wp_index_(0), is_exploring_(false)
    {
        // 1. 파라미터 선언 및 읽기 (CSV 파일 경로)
        this->declare_parameter<std::string>("csv_path", "/home/jhp1220/workspace/asp_ws/src/asp_uav_control/path/uav_path_safe.csv");
        std::string csv_path = this->get_parameter("csv_path").as_string();

        // 2. Publisher 생성 (드론 이동 명령, 카메라 각도 제어)
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/command/pose", 10);
        gimbal_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gimbal_pitch_degree", 10);

        // 3. Subscriber 생성 (탐색 시작 신호)
        start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/uav/exploration_start", 10,
            std::bind(&AspUavControlNode::startCallback, this, std::placeholders::_1));

        // 4. 타이머 생성 (1초마다 드론 상태 체크 및 목표 퍼블리시)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&AspUavControlNode::timerCallback, this));

        // 5. CSV 파일 읽기
        loadCSV(csv_path);

        RCLCPP_INFO(this->get_logger(), "ASP UAV Control Node Started. Waiting for start signal...");
    }

private:
    // --- [콜백 함수] ---
    void startCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !is_exploring_) {
            RCLCPP_INFO(this->get_logger(), "Exploration Start Signal Received!");
            
            // TODO: 실제로는 현재 드론의 위치를 받아와야 하지만, 임시로 첫 웨이포인트를 시작점으로 잡음
            double start_x = raw_waypoints_.empty() ? 0.0 : raw_waypoints_[0].x;
            double start_y = raw_waypoints_.empty() ? 0.0 : raw_waypoints_[0].y;

            // TSP 최적화 알고리즘 실행
            optimizeWaypoints(start_x, start_y);

            is_exploring_ = true;
            current_wp_index_ = 0;
        }
    }

    void timerCallback()
    {
        if (!is_exploring_ || optimized_waypoints_.empty() || current_wp_index_ >= optimized_waypoints_.size()) {
            return; // 탐색 중이 아니거나 모든 경로를 다 돌았으면 대기
        }

        // 현재 가야 할 목표 웨이포인트
        Waypoint target = optimized_waypoints_[current_wp_index_];

        // 1. 드론 목표 좌표 (PoseStamped) 퍼블리시
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = target.x;
        pose_msg.pose.position.y = target.y;
        pose_msg.pose.position.z = target.z;
        
        // Yaw 각도를 쿼터니언으로 변환
        double yaw_rad = target.yaw_deg * M_PI / 180.0;
        pose_msg.pose.orientation.z = std::sin(yaw_rad / 2.0);
        pose_msg.pose.orientation.w = std::cos(yaw_rad / 2.0);

        pose_pub_->publish(pose_msg);

        // 2. 짐벌(카메라) 각도 퍼블리시
        std_msgs::msg::Float32 gimbal_msg;
        gimbal_msg.data = target.gimbal_pitch_deg;
        gimbal_pub_->publish(gimbal_msg);

        RCLCPP_INFO(this->get_logger(), "Heading to Waypoint [%zu/%zu] Tag: %s (x:%.2f, y:%.2f)", 
                    current_wp_index_ + 1, optimized_waypoints_.size(), target.tag.c_str(), target.x, target.y);

        // TODO: 원래는 현재 위치를 수신해서 오차범위 내에 도달했는지 확인해야 합니다.
        // 현재는 코드를 간결하게 유지하기 위해 강제로 인덱스를 넘기는 코드는 주석 처리해둡니다.
        // if (도착 확인 로직) {
        //     current_wp_index_++;
        // }
    }

    // --- [CSV 로더] ---
    void loadCSV(const std::string& path)
    {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", path.c_str());
            return;
        }

        std::string line;
        std::getline(file, line); // 첫 줄(헤더) 건너뛰기

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            Waypoint wp;

            try {
                std::getline(ss, token, ','); wp.x = std::stod(token);
                std::getline(ss, token, ','); wp.y = std::stod(token);
                std::getline(ss, token, ','); wp.z = std::stod(token);
                std::getline(ss, token, ','); wp.yaw_deg = std::stod(token);
                std::getline(ss, token, ','); wp.gimbal_pitch_deg = std::stod(token);
                std::getline(ss, token, ','); wp.hold_sec = std::stod(token);
                std::getline(ss, token, ','); wp.tag = token;

                raw_waypoints_.push_back(wp);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Skipping invalid CSV line.");
            }
        }
        RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu waypoints.", raw_waypoints_.size());
    }

    // --- [TSP 최근접 이웃(Nearest Neighbor) 알고리즘] ---
    void optimizeWaypoints(double start_x, double start_y)
    {
        std::vector<Waypoint> unvisited = raw_waypoints_;
        optimized_waypoints_.clear();

        double curr_x = start_x;
        double curr_y = start_y;

        while (!unvisited.empty()) {
            int closest_idx = -1;
            double min_dist = std::numeric_limits<double>::max();

            // 현재 위치에서 가장 가까운 웨이포인트 찾기
            for (size_t i = 0; i < unvisited.size(); ++i) {
                double dist = std::hypot(unvisited[i].x - curr_x, unvisited[i].y - curr_y);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_idx = i;
                }
            }

            // 찾은 웨이포인트를 확정 경로에 넣고, 현재 위치 업데이트
            optimized_waypoints_.push_back(unvisited[closest_idx]);
            curr_x = unvisited[closest_idx].x;
            curr_y = unvisited[closest_idx].y;

            // 방문한 곳은 목록에서 제거
            unvisited.erase(unvisited.begin() + closest_idx);
        }

        RCLCPP_INFO(this->get_logger(), "TSP Optimization Complete! Sorted %zu waypoints.", optimized_waypoints_.size());
    }

    // --- [멤버 변수] ---
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gimbal_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Waypoint> raw_waypoints_;
    std::vector<Waypoint> optimized_waypoints_;
    
    size_t current_wp_index_;
    bool is_exploring_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AspUavControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}