#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_custom_msgs/msg/robot_status.hpp" // Auto-generated message header

using namespace std::chrono_literals;

class StatusPublisher : public rclcpp::Node
{
public:
  StatusPublisher()
  : Node("status_publisher"), battery_level_(100.0), mission_count_(0)
  {
    // Create the publisher
    publisher_ = this->create_publisher<ros2_custom_msgs::msg::RobotStatus>("/robot_status", 10);
    
    // Create a timer that fires every 1000ms
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&StatusPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Initialize the custom message
    auto message = ros2_custom_msgs::msg::RobotStatus();
    message.robot_name = "Explorer1";
    message.battery_level = battery_level_;
    message.is_active = true;
    message.mission_count = mission_count_;

    // Log the output to match the expected format
    RCLCPP_INFO(this->get_logger(), "Publishing: robot=%s, battery=%.1f, active=%s, missions=%d",
                message.robot_name.c_str(),
                message.battery_level,
                message.is_active ? "true" : "false",
                message.mission_count);

    // Publish the message
    publisher_->publish(message);

    // Update state variables for the next tick
    battery_level_ -= 0.5;
    mission_count_ += 1;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ros2_custom_msgs::msg::RobotStatus>::SharedPtr publisher_;
  
  // State variables
  double battery_level_;
  int mission_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatusPublisher>());
  rclcpp::shutdown();
  return 0;
}
