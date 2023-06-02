#include "md_robot_node/global.hpp"
#include "md_robot_node/main.hpp"
#include "md_robot_node/com.hpp"

// md_msgs::msg::Pose robot_pose;
// md_msgs::msg::RPM robot_rpm;

ROBOT_PARAMETER_t robotParamData;

SETTINNG_PARAM_STEP_t byCntInitStep;
volatile uint16_t appTick;
volatile uint16_t req_pid_pnd_main_data_count;
uint16_t byCntComStep;
uint32_t velCmdUpdateCount;
uint32_t velCmdRcvCount;

static uint8_t byCntCmdVel;
static uint8_t fgSendCmdVel;

static uint8_t byCntCase[10];
static uint8_t byFglIO;
INIT_SETTING_STATE_t fgInitsetting;

double goal_cmd_speed;     // m/sec
double goal_cmd_ang_speed; // radian/sec
bool reset_odom_flag;
bool reset_alarm_flag;

extern PID_PNT_MAIN_DATA_t curr_pid_pnt_main_data;
extern PID_ROBOT_MONITOR2_t curr_pid_robot_monitor2;
extern PID_PNT_IO_MONITOR_t curr_pid_pnt_io_monitor;
extern PID_ROBOT_MONITOR_t curr_pid_robot_monitor;

int L_brake_data = -1;
int R_brake_data = -1;

// double L_RPM = 0;
// double R_RPM = 0;

void PubRobotPose(const md_msgs::msg::Pose &msg, rclcpp::Node::SharedPtr node)
{
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    std::string topic_name = "/robot_pose/"+robotParamData.fb_state;

    auto robot_pose_pub = node->create_publisher<md_msgs::msg::Pose>(topic_name, qos_profile);
    robot_pose_pub->publish(msg);
    return;
}

void PubRobotRPM(const md_msgs::msg::RPM &msg, rclcpp::Node::SharedPtr node)
{
    std_msgs::msg::Float32MultiArray ms_msg;

    ms_msg.data.resize(2);

    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    std::string topic_name = "/heroehs/labor/whl/"+robotParamData.fb_state+"/rpm";
    auto robot_rpm_pub = node->create_publisher<md_msgs::msg::RPM>(topic_name, qos_profile);
    robot_rpm_pub->publish(msg);

    auto robot_ms_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/heroehs/labor/whl/" + robotParamData.fb_state + "/ms", qos_profile);

    static double temp = (2.0 * M_PI * robotParamData.wheel_radius) / 60;

    // temp = (2.0 * M_PI * robotParamData.wheel_radius) / 60;

    ms_msg.data[0] = temp * (double)msg.l_rpm;
    ms_msg.data[1] = temp * (double)msg.r_rpm;

    robot_ms_pub->publish(ms_msg);

    return;
}

int main(int argc, char *argv[])
{
    /**********************************************************************************************************/
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mdrobot_inwheel");
    /**********************************************************************************************************/

#if 1
    reset_odom_flag = false;
    reset_alarm_flag = false;

    fgSendCmdVel = 1;
    fgInitsetting = INIT_SETTING_STATE_NONE;

    parsingParameter(node);

    robotParamData.nIDPC = MID_PC;     // Platform mini-PC ID
    robotParamData.nIDMDUI = MID_MDUI; // MDUI ID
    robotParamData.nIDMDT = MID_MDT;   // MD750T, MD400T, MD200T ID
    if (robotParamData.use_MDUI == 1)
    { // If using MDUI
        robotParamData.nRMID = robotParamData.nIDMDUI;
        RCLCPP_INFO(node->get_logger(), "Using MDUI");
    }
    else
    {
        robotParamData.nRMID = robotParamData.nIDMDT;
        RCLCPP_INFO(node->get_logger(), "Not using MDUI");
    }

    robotParamData.nBaudrate = 57600; // fixed

    RCLCPP_INFO(node->get_logger(), "PC ID          : %d", robotParamData.nIDPC);
    RCLCPP_INFO(node->get_logger(), "MDUI ID        : %d", robotParamData.nIDMDUI);
    RCLCPP_INFO(node->get_logger(), "MDT ID         : %d", robotParamData.nIDMDT);
    RCLCPP_INFO(node->get_logger(), "Receving ID    : %d", robotParamData.nRMID);
    RCLCPP_INFO(node->get_logger(), "baudrate       : %d", robotParamData.nBaudrate);
    RCLCPP_INFO(node->get_logger(), "Wheel Radius(m): %f", robotParamData.wheel_radius);
    RCLCPP_INFO(node->get_logger(), "WheelLength(m) : %f", robotParamData.nWheelLength);
    RCLCPP_INFO(node->get_logger(), "Reduction rate : %d", robotParamData.nGearRatio);
    RCLCPP_INFO(node->get_logger(), "Motor pole     : %d", robotParamData.motor_pole);
    if (robotParamData.motor_position_type == 0)
    {
        RCLCPP_INFO(node->get_logger(), "motor position detection: hall sensor");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "motor position detection: encoder");
    }

    // If use hall sensor: 3 x pole no x reduction rate
    // If use encoder: 4 x encder x reduction rate
    RCLCPP_INFO(node->get_logger(), "PPR: %d", robotParamData.encoder_PPR);

    if (robotParamData.motor_position_type == 0)
    {
        RCLCPP_INFO(node->get_logger(), "Robot direction: Forward");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Robot direction: Reverse");
    }
    RCLCPP_INFO(node->get_logger(), "Max RPM        : %d", robotParamData.nMaxRPM);
    RCLCPP_INFO(node->get_logger(), "Position proportion gain: %d", robotParamData.position_proportion_gain);
    RCLCPP_INFO(node->get_logger(), "Speed proportion gain   : %d", robotParamData.speed_proportion_gain);
    RCLCPP_INFO(node->get_logger(), "Integral gain  : %d", robotParamData.integral_gain);
    RCLCPP_INFO(node->get_logger(), "Slow start     : %d", robotParamData.nSlowstart);
    RCLCPP_INFO(node->get_logger(), "Slow down      : %d\r\n", robotParamData.nSlowdown);

    if (robotParamData.motor_position_type == 0)
    {
        robotParamData.motor_count = robotParamData.motor_pole * 3 * robotParamData.nGearRatio;
    }
    else
    {
        robotParamData.motor_count = robotParamData.encoder_PPR * 4 * robotParamData.nGearRatio;
    }

    robotParamData.motor_count_per_degree = (double)(360.0 / (double)robotParamData.motor_count);
    RCLCPP_INFO(node->get_logger(), "count per degree: %f", robotParamData.motor_count_per_degree);

    robotParamData.nDiameter = (int)(robotParamData.wheel_radius * 2.0 * 1000.0); // nDiameter is (mm) unit

    InitSerialComm(node); // communication initialization in com.cpp

    /**********************************************************************************************************/
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    auto keyboard_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "/heroehs/labor/whl/cmd_vel", 10, cmdVelCallBack);

    auto reset_odom_sub = node->create_subscription<std_msgs::msg::Bool>(
        "reset_odom", 10,
        [node](std_msgs::msg::Bool::SharedPtr reset_odom_msg)
        {
            resetOdomCallBack(reset_odom_msg, node);
        });

    auto reset_alarm_sub = node->create_subscription<std_msgs::msg::Bool>(
        "reset_alarm", 10,
        [node](std_msgs::msg::Bool::SharedPtr reset_alarm_msg)
        {
            resetAlarmCallBack(reset_alarm_msg, node);
        });

    std::string topic_name = "/heroehs/labor/whl/"+robotParamData.fb_state+"/brake";
    auto brake_sub = node->create_subscription<md_msgs::msg::Brake>(
        topic_name, 10,
        [node](md_msgs::msg::Brake::SharedPtr msg)
        {
            BrakeCallback(msg, node);
        });

    std::string rpm_topic_name = "/heroehs/labor/whl/"+robotParamData.fb_state+"/rpm";
    auto robot_rpm_pub = node->create_publisher<md_msgs::msg::RPM>(rpm_topic_name, qos_profile);

    auto robot_ms_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("/heroehs/labor/whl/" + robotParamData.fb_state + "/ms", qos_profile);
    /**********************************************************************************************************/

    rclcpp::WallRate r(std::chrono::milliseconds(10));

    rclcpp::Time start_time = rclcpp::Clock().now();
    rclcpp::Duration start_delay = rclcpp::Duration::from_seconds(1.5);
    rclcpp::Time now_time = rclcpp::Clock().now();
    rclcpp::Time start_delay_time = now_time + start_delay;

    double start_delay_sec = start_delay_time.seconds();

    //---------------------------------------------------------------------------------------------------------
    // Start delay: 1.5sec
    //---------------------------------------------------------------------------------------------------------
    while (rclcpp::ok())
    {
        if (rclcpp::Clock().now().seconds() >= start_delay_sec)
        {
            break;
        }

        ReceiveDataFromController(node);

        rclcpp::spin_some(node);
        r.sleep();
    }

    appTick = 0;
    byCntInitStep = SETTING_PARAM_STEP_PID_PNT_VEL_CMD;
    auto app_tick_timer = node->create_wall_timer(std::chrono::milliseconds(500), AppTickTimerCallback);
    while (rclcpp::ok())
    {
        ReceiveDataFromController(node);

        if (appTick > 0)
        {
            appTick = 0;

            InitMotorParameter(node);

            if (fgInitsetting != INIT_SETTING_STATE_NONE)
            {
                break;
            }
        }

        rclcpp::spin_some(node);
        r.sleep();
    }

    app_tick_timer->cancel();

    if (fgInitsetting != INIT_SETTING_STATE_OK)
    {
        RCLCPP_INFO(node->get_logger(), "error.init ROBOT");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Init done");
    }

    byCntComStep = 0;
    appTick = 0;

    app_tick_timer = node->create_wall_timer(std::chrono::milliseconds(20), AppTickTimerCallback);

    auto vel_cmd_rcv_timeout = node->create_wall_timer(std::chrono::milliseconds(1000), VelCmdRcvTimeoutCallback);

    auto req_PID_PNT_MAIN_DATA_timer = node->create_wall_timer(std::chrono::milliseconds(100), Req_PID_PNT_MAIN_DATA_Callback);
#endif
    /****************************************************************************************************************************/
    while (rclcpp::ok())
    {
        ReceiveDataFromController(node);
        if (appTick > 0)
        {
            appTick = 0;
            RequestRobotStatusTask();
        }
        if (velCmdUpdateCount > 0 && L_brake_data == -1 && R_brake_data == -1)
        {
            SetRPM(node);
        }
        rclcpp::spin_some(node);
        r.sleep();
    }
    /****************************************************************************************************************************/
}

/**************************************************************************************

Set RPM

***************************************************************************************/

void SetRPM(rclcpp::Node::SharedPtr node)
{
    velCmdUpdateCount = 0;

    PID_PNT_VEL_CMD_t pid_pnt_vel_cmd, *p;
    int16_t *pGoalRPMSpeed;

    pGoalRPMSpeed = RobotSpeedToRPMSpeed(goal_cmd_speed, goal_cmd_ang_speed, node);
    // RCLCPP_INFO(node->get_logger(), "Goal RPM L:%d, R:%d", pGoalRPMSpeed[0], pGoalRPMSpeed[1]);

    p = &pid_pnt_vel_cmd;
    p->enable_id1 = 1;
    p->rpm_id1 = pGoalRPMSpeed[0];
    p->enable_id2 = 1;
    p->rpm_id2 = pGoalRPMSpeed[1];
    // p->req_monitor_id = REQUEST_PNT_MAIN_DATA;
    p->req_monitor_id = 0;

    PutMdData(PID_PNT_VEL_CMD, robotParamData.nRMID, (const uint8_t *)&pid_pnt_vel_cmd, sizeof(pid_pnt_vel_cmd));
}

/**************************************************************************************

Call Back

***************************************************************************************/

void BrakeCallback(const md_msgs::msg::Brake::SharedPtr msg, rclcpp::Node::SharedPtr node)
{
    PID_PROP_BRAKE_DATA_t pid_prop_brake_data, *p;

    p = &pid_prop_brake_data;

    if (msg->l_brake <= -1)
    {
        p->l_brake_disable = 0;
        L_brake_data = -1;
    }
    else if (msg->l_brake >= 0 && msg->l_brake <= 1000)
    {
        p->r_brake_disable = 1;
        L_brake_data = msg->l_brake;
        p->l_brake_data = L_brake_data;
    }
    else if (msg->l_brake > 1000)
    {
        p->r_brake_disable = 1;
        L_brake_data = 1000;
        p->l_brake_data = L_brake_data;
    }

    if (msg->r_brake <= -1)
    {
        p->r_brake_disable = 0;
        R_brake_data = -1;
    }
    else if (msg->r_brake >= 0 && msg->r_brake <= 1000)
    {
        p->r_brake_disable = 1;
        R_brake_data = msg->r_brake;
        p->r_brake_data = R_brake_data;
    }
    else if (msg->r_brake > 1000)
    {
        p->r_brake_disable = 1;
        R_brake_data = 1000;
        p->r_brake_data = R_brake_data;
    }

    p->return_type = 0;
    if (R_brake_data >= 0 && L_brake_data >= 0)
    {
        PutMdData(PID_PNT_PROP_BRAKE, robotParamData.nRMID, (const uint8_t *)&pid_prop_brake_data, sizeof(pid_prop_brake_data));
    }
}

void AppTickTimerCallback(void)
{
    appTick++;
}

void VelCmdRcvTimeoutCallback(void)
{
    static uint32_t old_velCmdRcvCount;

    if (velCmdRcvCount == velCmdRcvCount)
    {
        goal_cmd_speed = 0;
        goal_cmd_ang_speed = 0;
    }

    old_velCmdRcvCount = velCmdRcvCount;
}

void Req_PID_PNT_MAIN_DATA_Callback(void)
{
    req_pid_pnd_main_data_count++;
}

void resetOdomCallBack(const std_msgs::msg::Bool::SharedPtr reset_odom_msg, rclcpp::Node::SharedPtr node)
{
    if (reset_odom_msg->data == 1)
    {
        RCLCPP_INFO(node->get_logger(), "Reset Odom");
        reset_odom_flag = true;
    }
}

void resetAlarmCallBack(const std_msgs::msg::Bool::SharedPtr reset_alarm_msg, rclcpp::Node::SharedPtr node)
{
    if (reset_alarm_msg->data == 1)
    {
        RCLCPP_INFO(node->get_logger(), "Reset Alarm");
        reset_alarm_flag = true;
    }
}

void cmdVelCallBack(const geometry_msgs::msg::Twist::SharedPtr keyVel)
{
    static geometry_msgs::msg::Twist old_cmd;

    if (fgInitsetting == INIT_SETTING_STATE_OK)
    {
        velCmdUpdateCount++;

        goal_cmd_speed = keyVel->linear.x;
        goal_cmd_ang_speed = keyVel->angular.z;
    }

    return;
}

/********************************************************************************

RequestRobotStatusTask

*********************************************************************************/

void RequestRobotStatusTask(void)
{
    int nArray[5];
    uint8_t req_pid;

    switch (byCntComStep)
    {
    case 0:
    {
        req_pid = PID_PNT_MAIN_DATA; // PID 210
        PutMdData(PID_REQ_PID_DATA, robotParamData.nRMID, (const uint8_t *)&req_pid, 1);

        if (robotParamData.use_MDUI == 1)
        {
            byCntComStep = 1; // If using MDUI
        }
        else
        {
            byCntComStep = 3;
        }
        break;
    }
    case 1:
    {
        if (robotParamData.use_MDUI == 1)
        {                                 // If using MDUI
            req_pid = PID_ROBOT_MONITOR2; // PID 224, Only MDUI
            PutMdData(PID_REQ_PID_DATA, MID_MDUI, (const uint8_t *)&req_pid, 1);
        }

        byCntComStep = 3;
        break;
    }
#if 0
    case 2:
        {
            if(robotParamData.use_MDUI == 1) {  // If using MDUI
                req_pid = PID_PNT_IO_MONITOR;               // PID 241, Only MDUI, but not using
                PutMdData(PID_REQ_PID_DATA, MID_MDUI, (const uint8_t *)&req_pid, 1);
            }

            byCntComStep = 3;
            break;
        }
#endif
    case 3:
    {
        if (curr_pid_robot_monitor2.byPlatStatus.bits.bEmerSW == 1)
        {
            PID_PNT_TQ_OFF_t pid_pnt_tq_off, *p;
            fgSendCmdVel = 0;
            pid_pnt_tq_off.enable_id1 = 1;
            pid_pnt_tq_off.enable_id2 = 1;
            pid_pnt_tq_off.req_monitor_id = REQUEST_PNT_MAIN_DATA;
            PutMdData(PID_PNT_TQ_OFF, robotParamData.nRMID, (const uint8_t *)&pid_pnt_tq_off, sizeof(pid_pnt_tq_off));
        }
        else if (reset_odom_flag == true)
        {
            reset_odom_flag = false;

            PutMdData(PID_POSI_RESET, robotParamData.nRMID, NULL, 0);
            ResetOdom();
        }
        else if (reset_alarm_flag == true)
        {
            uint8_t cmd_pid;

            reset_alarm_flag = false;

            cmd_pid = CMD_ALARM_RESET;
            PutMdData(PID_COMMAND, robotParamData.nRMID, (const uint8_t *)&cmd_pid, 1);
        }

        byCntComStep = 0;
        break;
    }
    case 4:
        req_pid = PID_GAIN; // PID: 203
        PutMdData(PID_REQ_PID_DATA, robotParamData.nIDMDT, (const uint8_t *)&req_pid, 1);

        byCntComStep = 0;
        break;

    default:
        byCntComStep = 0;
        break;
    }
}

/********************************************************************************

load parameter

*********************************************************************************/

void parsingParameter(std::shared_ptr<rclcpp::Node> node)
{
    // Do something with the node
    node->declare_parameter("use_MDUI");
    node->declare_parameter("wheel_radius");
    node->declare_parameter("wheel_length");
    node->declare_parameter("reduction");
    node->declare_parameter("motor_pole");
    node->declare_parameter("reverse_direction");
    node->declare_parameter("maxrpm");
    node->declare_parameter("motor_posi");
    node->declare_parameter("encoder_PPR");
    node->declare_parameter("position_proportion_gain");
    node->declare_parameter("speed_proportion_gain");
    node->declare_parameter("integral_gain");
    node->declare_parameter("slow_start");
    node->declare_parameter("slow_down");
    node->declare_parameter("usb");
    node->declare_parameter("fb_state");

    if (node->has_parameter("usb"))
    {
        robotParamData.usb = node->get_parameter("usb").as_string();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of usb");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }
    if (node->has_parameter("use_MDUI"))
    {
        robotParamData.use_MDUI = node->get_parameter("use_MDUI").as_int();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of robotParamData.use_MDUI");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }
    if (node->has_parameter("wheel_radius"))
    {
        robotParamData.wheel_radius = node->get_parameter("wheel_radius").as_double();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of robotParamData.wheel_radius");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }
    if (node->has_parameter("wheel_length"))
    {
        robotParamData.nWheelLength = node->get_parameter("wheel_length").as_double();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of robotParamData.nWheelLength");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }
    if (node->has_parameter("reduction"))
    {
        robotParamData.nGearRatio = node->get_parameter("reduction").as_int();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of robotParamData.nGearRatio");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }
    if (node->has_parameter("motor_pole"))
    {
        robotParamData.motor_pole = node->get_parameter("motor_pole").as_int();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of robotParamData.motor_pole");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }
    if (node->has_parameter("reverse_direction"))
    {
        robotParamData.reverse_direction = node->get_parameter("reverse_direction").as_int();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of robotParamData.reverse_direction");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }
    if (node->has_parameter("maxrpm"))
    {
        robotParamData.nMaxRPM = node->get_parameter("maxrpm").as_int();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of robotParamData.nMaxRPM");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }
    if (node->has_parameter("motor_posi"))
    {
        robotParamData.motor_position_type = node->get_parameter("motor_posi").as_int();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of robotParamData.motor_position_type");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }
    if (node->has_parameter("encoder_PPR"))
    {
        robotParamData.encoder_PPR = node->get_parameter("encoder_PPR").as_int();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of robotParamData.encoder_PPR");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }
    if (node->has_parameter("position_proportion_gain"))
    {
        robotParamData.position_proportion_gain = node->get_parameter("position_proportion_gain").as_int();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of robotParamData.position_proportion_gain");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }
    if (node->has_parameter("speed_proportion_gain"))
    {
        robotParamData.speed_proportion_gain = node->get_parameter("speed_proportion_gain").as_int();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of robotParamData.speed_proportion_gain");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }
    if (node->has_parameter("integral_gain"))
    {
        robotParamData.integral_gain = node->get_parameter("integral_gain").as_int();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of robotParamData.integral_gain");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }
    if (node->has_parameter("slow_start"))
    {
        robotParamData.nSlowstart = node->get_parameter("slow_start").as_int();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of robotParamData.nSlowstart");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }
    if (node->has_parameter("slow_down"))
    {
        robotParamData.nSlowdown = node->get_parameter("slow_down").as_int();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of robotParamData.nSlowdown");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }

    if (node->has_parameter("fb_state"))
    {
        robotParamData.fb_state = node->get_parameter("fb_state").as_string();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get the value of robotParamData.fb_state");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }
}