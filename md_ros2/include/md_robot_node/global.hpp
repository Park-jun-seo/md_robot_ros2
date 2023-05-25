#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <stdlib.h>

#include <sstream>
#include <iostream>
#include <sys/ioctl.h>

#include <boost/asio.hpp>
#include <float.h>
#include <math.h>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"

#include "md_msgs/msg/brake.hpp"
#include "md_msgs/msg/pose.hpp"
#include "md_msgs/msg/rpm.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"

// #define LIMIT(value, minValue, maxValue) ((value) < (minValue) ? (minValue) : ((value) > (maxValue) ? (maxValue) : (value)))
#define constrain(amt, low, high) ((amt) <= (low) ? (low) : ((amt) >= (high) ? (high) : (amt)))


void cmdVelCallBack(const geometry_msgs::msg::Twist::SharedPtr keyVel); // from turtlebot3_teleop_key node
void resetOdomCallBack(const std_msgs::msg::Bool::SharedPtr reset_odom_msg, rclcpp::Node::SharedPtr node);
void resetAlarmCallBack(const std_msgs::msg::Bool::SharedPtr reset_alarm_msg, rclcpp::Node::SharedPtr node);

void BrakeCallback(const md_msgs::msg::Brake::SharedPtr msg, rclcpp::Node::SharedPtr node);

using namespace std;
