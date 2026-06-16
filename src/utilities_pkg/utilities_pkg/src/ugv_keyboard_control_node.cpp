#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#include <algorithm>
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

class UGVKeyboardControl : public rclcpp::Node
{
public:
  UGVKeyboardControl()
  : Node("ugv_keyboard_control_node")
  {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/command/ugv_cmd_vel", 10);

    setup_terminal();

    timer_ = this->create_wall_timer(
      50ms,
      std::bind(&UGVKeyboardControl::timer_callback, this));

    print_help();
  }

  ~UGVKeyboardControl() override
  {
    publish_stop();
    restore_terminal();
    std::cout << "\nUGV keyboard control stopped.\n";
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  struct termios old_termios_;
  bool terminal_configured_{false};

  double linear_x_{0.0};
  double angular_z_{0.0};

  static constexpr double LINEAR_STEP = 0.2;
  static constexpr double ANGULAR_STEP = 0.2;
  static constexpr double MAX_LINEAR = 2.0;
  static constexpr double MAX_ANGULAR = 1.5;

  void setup_terminal()
  {
    if (tcgetattr(STDIN_FILENO, &old_termios_) != 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to get terminal attributes.");
      return;
    }

    struct termios new_termios = old_termios_;
    new_termios.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    new_termios.c_cc[VMIN] = 0;
    new_termios.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_termios) != 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to set terminal raw mode.");
      return;
    }

    terminal_configured_ = true;
  }

  void restore_terminal()
  {
    if (terminal_configured_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
      terminal_configured_ = false;
    }
  }

  bool read_key(char & key)
  {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int result = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
    if (result > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
      if (read(STDIN_FILENO, &key, 1) == 1) {
        return true;
      }
    }

    return false;
  }

  void print_help()
  {
    std::cout << "\nUGV keyboard control started.\n"
              << "Topic: /command/ugv_cmd_vel\n"
              << "w/x: forward/backward\n"
              << "a/d: turn left/right\n"
              << "s or SPACE: stop\n"
              << "h: help\n"
              << "Ctrl+C: quit\n\n";
  }

  void clamp_command()
  {
    linear_x_ = std::clamp(linear_x_, -MAX_LINEAR, MAX_LINEAR);
    angular_z_ = std::clamp(angular_z_, -MAX_ANGULAR, MAX_ANGULAR);
  }

  void publish_current_command()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x_;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = angular_z_;

    cmd_pub_->publish(cmd);
  }

  void publish_stop()
  {
    linear_x_ = 0.0;
    angular_z_ = 0.0;
    publish_current_command();
  }

  void timer_callback()
  {
    char key = 0;
    bool updated = false;

    while (read_key(key)) {
      switch (key) {
        case 'w':
          linear_x_ += LINEAR_STEP;
          updated = true;
          break;

        case 'x':
          linear_x_ -= LINEAR_STEP;
          updated = true;
          break;

        case 'a':
          angular_z_ += ANGULAR_STEP;
          updated = true;
          break;

        case 'd':
          angular_z_ -= ANGULAR_STEP;
          updated = true;
          break;

        case 's':
        case ' ':
          linear_x_ = 0.0;
          angular_z_ = 0.0;
          updated = true;
          break;

        case 'h':
          print_help();
          break;

        default:
          break;
      }
    }

    clamp_command();
    publish_current_command();

    if (updated) {
      std::cout << "\rlinear.x: " << linear_x_
                << " m/s, angular.z: " << angular_z_
                << " rad/s       " << std::flush;
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UGVKeyboardControl>());
  rclcpp::shutdown();
  return 0;
}
