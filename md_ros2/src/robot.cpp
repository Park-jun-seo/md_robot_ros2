#include <stdint.h>
#include "md_robot_node/global.hpp"
#include "md_robot_node/main.hpp"
#include "md_robot_node/com.hpp"
#include "md_msgs/msg/pose.hpp"

#define VELOCITY_CONSTANT_VALUE 9.5492743 // 이동속도(m/min), v = 바퀴 둘레의 길이 x RPM
                                          // 이동속도(m/sec), v = (2 x 바퀴 반지름 x (pi / 60) x RPM)
                                          // 0.10472 = (2 x pi / 60)
                                          // V = r * w = r * (RPM * 0.10472)
                                          //           = r * RPM * 0.10472
                                          // RPM = V / r * 9.5492743


#define LEFT 0 // Swing direction
#define RIGHT 1

static double robot_old_x;
static double robot_old_y;
static double robot_old_theta;

md_msgs::msg::Pose robot_pose;
md_msgs::msg::RPM robot_rpm;
// extern md_msgs::msg::Pose robot_pose;
// extern md_msgs::msg::RPM robot_rpm;

// RPM --> m/sec
double *RPMSpeedToRobotSpeed(int16_t rpm_left, int16_t rpm_right, rclcpp::Node::SharedPtr node)
{
    double v_left;
    double v_right;
    double temp;
    static double robot_speed[2];

    temp = (2.0 * M_PI * robotParamData.wheel_radius) / 60;

    v_left = temp * (double)rpm_left;
    v_right = temp * (double)rpm_right;

    robot_speed[0] = (v_right + v_left) / 2;
    robot_speed[1] = (v_right - v_left) / robotParamData.nWheelLength;

    // RCLCPP_INFO(node->get_logger(),"l:%f, a:%f", (double)robot_speed[0], (double)robot_speed[1]);

    return robot_speed;
}

// m/sec --> RPM
int16_t *RobotSpeedToRPMSpeed(double linear, double angular, rclcpp::Node::SharedPtr node)
{
    double wheel_radius;
    double wheel_separation;
    double reduction;
    double wheel_velocity_cmd[2];
    static int16_t goal_rpm_spped[2];

    wheel_radius = robotParamData.wheel_radius;
    wheel_separation = robotParamData.nWheelLength;
    reduction = (double)robotParamData.nGearRatio;

    // RCLCPP_INFO(node->get_logger(),"l:%f, a:%f", (double)linear, (double)angular);

    wheel_velocity_cmd[LEFT] = linear - (angular * wheel_separation / 2);
    wheel_velocity_cmd[RIGHT] = linear + (angular * wheel_separation / 2);

    // ROS_INFO("left:%f, right:%f", (double)wheel_velocity_cmd[LEFT], (double)wheel_velocity_cmd[RIGHT]);

    //***************************************************************************************
    // Linearvelocity --> RPM 으로 환산
    //***************************************************************************************
    wheel_velocity_cmd[LEFT] = constrain(wheel_velocity_cmd[LEFT] * VELOCITY_CONSTANT_VALUE / wheel_radius * reduction, -robotParamData.nMaxRPM, robotParamData.nMaxRPM);
    wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius * reduction, -robotParamData.nMaxRPM, robotParamData.nMaxRPM);

    // RCLCPP_INFO(node->get_logger(),"RPM1 L:%f, R:%f\r\n", (double)wheel_velocity_cmd[LEFT], (double)wheel_velocity_cmd[RIGHT]);

    goal_rpm_spped[0] = (int16_t)(wheel_velocity_cmd[LEFT]);
    goal_rpm_spped[1] = (int16_t)(wheel_velocity_cmd[RIGHT]);

    return goal_rpm_spped;
}

/*******************************************************************

Robot pose

*******************************************************************/

void ResetOdom(void)
{
    robot_old_x = 0.0;
    robot_old_y = 0.0;
    robot_old_theta = 0.0;

    robot_pose.x = 0.0;
    robot_pose.y = 0.0;
    robot_pose.theta = 0.0;
    robot_pose.linear_velocity = 0.0;
    robot_pose.angular_velocity = 0.0;
}

void CalRobotPoseFromRPM(PID_PNT_MAIN_DATA_t *pData, rclcpp::Node::SharedPtr node)
{
    static rclcpp::Time previous_time;
    // static int32_t old_mtr_pos_id1;
    // static int32_t old_mtr_pos_id2;
    static long previous_rpm;
    
    double robot_curr_theta;
    double interval_time;
    double *pVelocity;
    int16_t rpm_left;
    int16_t rpm_right;
    double lAvrRPM;
    
    double robot_curr_x;
    double robot_curr_y;
    double delta_theta;
    double delta_s;

    rclcpp::Time curr_time = rclcpp::Clock().now();
    //--------------------------------------------------------------------------
    rpm_left = pData->rpm_id1;
    if (rpm_left > 0)
    {
        rpm_left = rpm_left + (robotParamData.nGearRatio / 2);
    }
    else
    {
        rpm_left = rpm_left - (robotParamData.nGearRatio / 2);
    }
    rpm_left /= robotParamData.nGearRatio;

    
    rpm_right = pData->rpm_id2;
    if (rpm_right > 0)
    {
        rpm_right = rpm_right + (robotParamData.nGearRatio / 2);
    }
    else
    {
        rpm_right = rpm_right - (robotParamData.nGearRatio / 2);
    }
    rpm_right /= robotParamData.nGearRatio;
    //----------------------------------------------------------------------
    // RCLCPP_INFO(node->get_logger(),"\r\n");
    // RCLCPP_INFO(node->get_logger(),"Robot RPM-1: %d : %d", pData->rpm_id1, pData->rpm_id2);
    // RCLCPP_INFO(node->get_logger(),"Robot RPM-2: %d : %d", rpm_left, rpm_right);

    //---------------------------------------------------------------------
    // RPM noise filter
    //---------------------------------------------------------------------
    lAvrRPM = (rpm_left + rpm_right) / 2;
    if (abs(lAvrRPM - previous_rpm) > (robotParamData.nMaxRPM / 2))
    {
        lAvrRPM = previous_rpm;
    }
    previous_rpm = lAvrRPM;
    //---------------------------------------------------------------------

    pVelocity = RPMSpeedToRobotSpeed(rpm_left, rpm_right, node);

    // RCLCPP_INFO(node->get_logger(),"1-Robot v:a: %f : %f", pVelocity[0], pVelocity[1]);

    interval_time = curr_time.seconds() - previous_time.seconds();
    previous_time = curr_time;

    delta_s = pVelocity[0] * interval_time;
    delta_theta = pVelocity[1] * interval_time;

    robot_curr_x = robot_old_x + delta_s * cos((long double)(robot_old_theta + (delta_theta / 2.0)));
    robot_curr_y = robot_old_y + delta_s * sin((long double)(robot_old_theta + (delta_theta / 2.0)));
    robot_curr_theta = robot_old_theta + delta_theta;

    robot_old_x = robot_curr_x;
    robot_old_y = robot_curr_y;
    robot_old_theta = robot_curr_theta;

    robot_pose.x = robot_curr_x;
    robot_pose.y = robot_curr_y;
    robot_pose.theta = robot_curr_theta;
    robot_pose.linear_velocity = pVelocity[0];
    robot_pose.angular_velocity = pVelocity[1];

    // robot_pose.left_motor_state = pData->mtr_state_id1.val;
    // robot_pose.right_motor_state = pData->mtr_state_id2.val;

    robot_rpm.l_rpm = rpm_left;
    robot_rpm.r_rpm = rpm_right;
    PubRobotRPM(robot_rpm,node);
    // PubRobotPose(robot_pose,node);
#if 1
    // RCLCPP_INFO(node->get_logger(),"1-pos x:y:th  %f:%f:%f\r\n", robot_curr_x, robot_curr_y, robot_curr_theta);
#endif
}
