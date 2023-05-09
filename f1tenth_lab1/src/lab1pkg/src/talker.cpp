#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/AckermannDriveStamped.hpp>
#include <ackermann_msgs/msg/AckermannDrive.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher")
    {
      this->declare_parameter("v", rclcpp::PARAMETER_INTEGER);
      this->declare_parameter("d", rclcpp::PARAMETER_INTEGER);
      rclcpp::Parameter v_param = this->get_parameter("v");
      rclcpp::Parameter d_param = this->get_parameter("d");
      int v = v_param.as_int();
      int d = d_param.as_int();
      publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = ackermann_msgs::msg::AckermannDriveStamped();
      auto value = ackermann_msgs::msg::AckermannDrive();
      value.speed = v;
      value.steering_angle = d;
      message.header.stamp = pp.get_clock().now().to_msg()
      message.drive = value;      
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}