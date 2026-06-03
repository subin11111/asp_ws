#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <std_msgs/msg/int32.hpp>

class PrecisionLandingNode : public rclcpp::Node
{
public:
    PrecisionLandingNode()
    : Node("precision_landing_node")
    {
        mission_state_ = 0;

        marker_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco/marker_pose",
                10,
                std::bind(
                    &PrecisionLandingNode::markerCallback,
                    this,
                    std::placeholders::_1));

        mission_sub_ =
            this->create_subscription<std_msgs::msg::Int32>(
                "/mission_state",
                10,
                std::bind(
                    &PrecisionLandingNode::missionCallback,
                    this,
                    std::placeholders::_1));

        cmd_pub_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                "/command/twist",
                10);

        RCLCPP_INFO(
            this->get_logger(),
            "Precision Landing Node Started");
    }

private:

    void missionCallback(
        const std_msgs::msg::Int32::SharedPtr msg)
    {
        mission_state_ = msg->data;
    }

    void markerCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (mission_state_ != 1)
        {
            return;
        }

        double x = msg->pose.position.x;
        double y = msg->pose.position.y;

        RCLCPP_INFO(
            this->get_logger(),
            "Marker Position x=%.2f y=%.2f",
            x,
            y);

        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = 0.05 * x;
        cmd.linear.y = 0.05 * y;

        if (std::abs(x) < 0.2 &&
            std::abs(y) < 0.2)
        {
            cmd.linear.z = -0.2;

            RCLCPP_INFO(
                this->get_logger(),
                "Descending..."
            );
        }

        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mission_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    int mission_state_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PrecisionLandingNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}