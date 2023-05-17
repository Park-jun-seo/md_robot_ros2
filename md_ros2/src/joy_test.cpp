#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

double v = 1.8;
double rad = 0.3;

class JoyToTwist : public rclcpp::Node
{
public:
  JoyToTwist()
      : Node("joy_to_twist")
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&JoyToTwist::joy_callback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // initialize linear x to zero and angular z to zero
    twist_.linear.x = 0.0;
    twist_.angular.z = 0.0;
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // update linear x based on trigger button
    if (msg->axes[2] == 1)
    {
      float trigger_value = map(msg->axes[5], 1.0, -1.0, 0.0, v);
      twist_.linear.x = trigger_value;
    }
    else if (msg->axes[5] == 1)
    {
      float trigger_value = map(msg->axes[2], 1.0, -1.0, 0.0, -1 * v);
      twist_.linear.x = trigger_value;
    }

    // update angular z based on joystick
    float joystick_value = map(msg->axes[3], -1.0, 1.0, -1.0 * rad, 1.0 * rad);
    twist_.angular.z = (joystick_value > 0.0) ? map(joystick_value, 0.0, 1.0, 0.0, 1.0) : map(joystick_value, -1.0, 0.0, -1.0, 0.0);

    twist_pub_->publish(twist_);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  geometry_msgs::msg::Twist twist_;

  float map(float x, float in_min, float in_max, float out_min, float out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToTwist>());
  rclcpp::shutdown();
  return 0;
}
