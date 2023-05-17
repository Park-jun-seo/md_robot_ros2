#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class VelocityConverter : public rclcpp::Node
{
public:
    VelocityConverter()
        : Node("velocity_converter")
    {
        fleft_pub_ = this->create_publisher<std_msgs::msg::Float64>("/heroehs/labor/whl/fl/velocity/command", 10);
        fright_pub_ = this->create_publisher<std_msgs::msg::Float64>("/heroehs/labor/whl/fr/velocity/command", 10);

        bleft_pub_ = this->create_publisher<std_msgs::msg::Float64>("/heroehs/labor/whl/bl/velocity/command", 10);
        bright_pub_ = this->create_publisher<std_msgs::msg::Float64>("/heroehs/labor/whl/br/velocity/command", 10);

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&VelocityConverter::callback, this, std::placeholders::_1));
    }

private:
    void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std_msgs::msg::Float64 left_wheel_vel;
        std_msgs::msg::Float64 right_wheel_vel;
        
        double wheel_radius = 0.0535; // Wheel radius, you may need to adjust this
        double wheel_separation = 0.47; // Distance between two wheels, you may need to adjust this

        double v = msg->linear.x;
        double omega = msg->angular.z;

        left_wheel_vel.data = (v - omega * wheel_separation / 2) / wheel_radius;
        right_wheel_vel.data = (v + omega * wheel_separation / 2) / wheel_radius;

        fleft_pub_->publish(left_wheel_vel);
        bleft_pub_->publish(left_wheel_vel);
        fright_pub_->publish(right_wheel_vel);
        bright_pub_->publish(right_wheel_vel);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fleft_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fright_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bleft_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bright_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityConverter>());
    rclcpp::shutdown();
    return 0;
}
