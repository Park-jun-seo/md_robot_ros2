#define ID_ALL 0xfe
#define MAX_PACKET_SIZE 26
#define MAX_DATA_SIZE 21
#define REQUEST_PNT_MAIN_DATA 2

#define CMD_TQ_OFF 2 // CMD_TQ_OFF, 모터 자연정지
#define CMD_ALARM_RESET 8
#define CMD_BRAKE 4       // CMD_BRAKE, 전기적 브레이크, 급작정지
#define CMD_FAN_ON 15     // 팬(FAN) ON
#define CMD_FAN_OFF 16    // 팬(FAN) OFF
#define CMD_CLUTCH_ON 17  // 클러치 ON
#define CMD_CLUTCH_OFF 18 // 클러치 OFF
#define CMD_PNT_MAIN_DATA_BC_ON 61 //PID_PNT_MAIN_DATA 의 BC ON
#define CMD_BRAKE1 69     // 모터 1 에 대한 전자브레이크 ON
#define CMD_BRAKE2 70     // 모터 2 에 대한 전자브레이크 ON

#define MID_MDUI 184
#define MID_MDT 183
#define MID_PC 172

typedef enum
{
    PID_REQ_PID_DATA = 4,   // 데이터 요청
    PID_COMMAND = 10,       //
    PID_POSI_RESET = 13,    //
    PID_INV_SIGN_CMD = 16,  //
    PID_INV_SIGN_CMD2 = 18, //
    PID_USE_POSI = 46,
    PID_PPR = 126,              //
    PID_SLOW_START = 153,       //
    PID_SLOW_DOWN = 154,        //
    PID_PNT_TQ_OFF = 174,       //
    PID_PNT_BRAKE = 175,        //
    PID_GAIN = 203,             //
    PID_PNT_VEL_CMD = 207,      // 모터 2 개 구동용 제어기의 속도제어명령
    PID_PNT_MAIN_DATA = 210,    // req
    PID_ROBOT_MONITOR2 = 224,   // req, onlt MDUI
    PID_PNT_IO_MONITOR = 241,   // req, only MDUI
    PID_ROBOT_PARAM = 247,      // set, only MDUI
    PID_STOP_STATUS = 24,       // 0 : STOP_TQ_OFF 2 : STOP_BRAKE 3 : STOP_FREE
    PID_REGENERATION = 38,      // 회생제동 기능 선택 1 : 회생제동기능 동작, 0 : 회생제동기능 OFF
    PID_OVERLOAD_ALARM_ON = 51, // 1 : 최대 전류의 90%이상에서 4 초 이상구동하는 경우에 알람을 띄우고 모터 정지 0 : 알람을 띄우지 않고 최대전류제한 기능만 수행 최대전류의 120%이상에서는 알람을 발생함
    PID_USE_CLUTCH = 63,        // 클러지 사용유무
    PID_USE_CLUTCH2 = 64,       // 모터 2, 클러지 사용유무
    PID_USE_CLUTCH1 = 67,       // 모터 1, 클러지 사용유무
    PID_PROP_BRAKE = 107,       // 전기적 브레이크 출력을 비례적으로 제어 DATA(0~1023) 183, TMID, ID, 107, 2, D1, D2, CHK
    PID_VOLT_IN = 143,          // 제어기 입력전압
    PID_PNT_VEL_DATA = 177,     // MOT1,MOT2 의 속도
    PID_COM_WATCH_DELAY =  185, //(0.1s 단위) 동안 통신신호 없으면 모터 정지
    PID_PNT_PROP_BRAKE = 192,
} PID_CMD_t;

typedef enum
{
    SETTING_PARAM_STEP_PID_PNT_VEL_CMD = 1,
    SETTING_PARAM_STEP_PID_ROBOT_PARAM,
    SETTING_PARAM_STEP_PID_POSI_RESET,
    SETTING_PARAM_STEP_PID_SLOW_START,
    SETTING_PARAM_STEP_PID_SLOW_DOWN,
    SETTING_PARAM_STEP_PID_GAIN,
    SETTING_PARAM_STEP_PID_INV_SIGH_CMD,
    SETTING_PARAM_STEP_PID_INV_SIGH_CMD2,
    SETTING_PARAM_STEP_PID_REGENERATION,
    SETTING_PARAM_STEP_PID_USE_EPOSI,
    SETTING_PARAM_STEP_PID_PPR,
    SETTING_PARAM_STEP_PID_COM_WATCH_DELAY,
    SETTING_PARAM_STEP_PID_STOP_STATUS,
    SETTING_PARAM_STEP_DONE,
} SETTINNG_PARAM_STEP_t;

typedef enum
{
    INIT_SETTING_STATE_NONE = 0,
    INIT_SETTING_STATE_OK = 1,
    INIT_SETTING_STATE_ERROR = 2,
} INIT_SETTING_STATE_t;

typedef enum
{
    MOTOR_INSTALL_DIR_CW = 0,
    MOTOR_INSTALL_DIR_CCW = 1,
} MOTOR_INSTALL_DIR_t;

typedef struct
{
    int16_t PPR;
} __attribute__((aligned(1), packed)) PID_PPR_t;

typedef struct
{
    int16_t position_proportion_gain;
    int16_t speed_proportion_gain;
    int16_t integral_gain;
} __attribute__((aligned(1), packed)) PID_GAIN_t;

typedef struct
{
    uint8_t enable_id1;
    int16_t rpm_id1;
    uint8_t enable_id2;
    int16_t rpm_id2;
    uint8_t req_monitor_id;
} __attribute__((aligned(1), packed)) PID_PNT_VEL_CMD_t;

typedef struct
{
    uint16_t nDiameter;
    uint16_t nWheelLength;
    uint16_t nGearRatio;
} __attribute__((aligned(1), packed)) PID_ROBOT_PARAM_t;

typedef struct
{
    uint8_t enable_id1;
    uint8_t enable_id2;
    uint8_t req_monitor_id;
} __attribute__((aligned(1), packed)) PID_PNT_TQ_OFF_t;

typedef union _MOTOR_STATE_t
{
    uint8_t val;
    struct
    {
        uint8_t Alarm : 1;
        uint8_t CtrlFail : 1;
        uint8_t OverVolt : 1;
        uint8_t OverTemp : 1;
        uint8_t OverLoad : 1;
        uint8_t HallFail : 1;
        uint8_t InvVel : 1;
        uint8_t Stall : 1;
    } bits;
} MOTOR_STATE_t;

typedef struct
{
    int16_t rpm_id1;
    int16_t current_id1;
    MOTOR_STATE_t mtr_state_id1;
    int32_t mtr_pos_id1;

    int16_t rpm_id2;
    int16_t current_id2;
    MOTOR_STATE_t mtr_state_id2;
    int32_t mtr_pos_id2;
} __attribute__((aligned(1), packed)) PID_PNT_MAIN_DATA_t;

typedef union _PLATFORM_STATE_t
{
    uint8_t val;
    struct
    {
        uint8_t bEmerSW : 1;
        uint8_t bBusy : 1;
        uint8_t bBumper1 : 1;
        uint8_t bBumper2 : 1;
        uint8_t bBumper3 : 1;
        uint8_t bBumper4 : 1;
        uint8_t bReserved1 : 1;
        uint8_t bReserved2 : 1;
    } bits;
} PLATFORM_STATE_t;

typedef union _PLATFORM_STATE_2_t
{
    uint8_t val;
    struct
    {
        uint8_t bEmerSW : 1;
        uint8_t bBusy : 1;
        uint8_t bBumper1 : 1;
        uint8_t bBumper2 : 1;
        uint8_t bBumper3 : 1;
        uint8_t bBumper4 : 1;
        uint8_t bReserved1 : 1;
        uint8_t bReserved2 : 1;
    } bits;
} PLATFORM_STATE_2_t;

typedef union _DOCKING_STATE_t
{
    uint8_t val;
    struct
    {
        uint8_t bDocComple : 1;
        uint8_t bChargeState : 1;
        uint8_t bCharComple : 1;
        uint8_t reserved1 : 1;
        uint8_t bIr1 : 1;
        uint8_t bIr2 : 1;
        uint8_t bIr3 : 1;
        uint8_t bRccState : 1;
    } bits;
} DOCKING_STATE_t;

typedef struct
{
    uint32_t lTempPosi_x;
    uint32_t lTempPosi_y;
    uint16_t sTempTheta;
    uint8_t battery_percent;
    uint8_t byUS1;
    uint8_t byUS2;
    uint8_t byUS3;
    uint8_t byUS4;
    PLATFORM_STATE_t byPlatStatus;
    uint8_t charger_connection_state;
} __attribute__((aligned(1), packed)) PID_ROBOT_MONITOR_t;

typedef struct
{
    uint16_t sVoltIn;
    uint8_t byUS1;
    uint8_t byUS2;
    uint8_t byUS3;
    uint8_t byUS4;
    PLATFORM_STATE_2_t byPlatStatus;
    DOCKING_STATE_t byDockingStatus;
    int16_t head_motor_speed_ctrl_signal;
} __attribute__((aligned(1), packed)) PID_ROBOT_MONITOR2_t;

typedef union _MOTOR_CTRL_SIGNAL_t
{
    uint8_t val;
    struct
    {
        uint8_t fgIntSpeed : 1;
        uint8_t fgAlarmReset : 1;
        uint8_t fgDirection : 1;
        uint8_t fgRunBrake : 1;
        uint8_t fgStartStop : 1;
        uint8_t fgEncoderA : 1;
        uint8_t fgEncoderB : 1;
    } bits;
} MOTOR_CTRL_SIGNAL_t;

typedef struct
{
    MOTOR_CTRL_SIGNAL_t motor_1;
    MOTOR_CTRL_SIGNAL_t motor_2;
    uint8_t motor_1_hall_sensor_input;
    uint8_t motor_2_hall_sensor_input;
    uint16_t motor_1_speed_var_volume;
    uint16_t motor_2_speed_var_volume;
    uint16_t sVoltIn;
    uint8_t slow_start_var_volume;
    uint8_t load_var_volume;
    uint8_t dip_sw;
} __attribute__((aligned(1), packed)) PID_PNT_IO_MONITOR_t;

typedef struct
{
    uint16_t value;
} PID_SLOW_START_t;

typedef struct
{
    uint16_t value;
} PID_SLOW_DOWN_t;

typedef struct
{
    uint16_t data1;
    uint16_t data2;

} __attribute__((aligned(1), packed)) PID_WATCH_DELAY_DATA_t;

typedef struct
{
    uint16_t l_brake_disable;
    int8_t l_brake_data;
    uint16_t r_brake_disable;
    int8_t r_brake_data;
    uint8_t return_type;

} __attribute__((aligned(1), packed)) PID_PROP_BRAKE_DATA_t;

typedef struct
{
    uint8_t l_brake_on;
    uint8_t r_brake_on;
    uint8_t return_type;

} __attribute__((aligned(1), packed)) PID_PNT_BRAKE_DATA_t;

typedef struct
{
    int use_MDUI;
    int nIDPC;
    int nIDMDUI;
    int nIDMDT;
    int nBaudrate;
    int nDiameter;
    double wheel_radius;
    double nWheelLength;
    int nRMID;
    int nGearRatio;
    int reverse_direction;
    int motor_position_type;
    int encoder_PPR;
    int nMaxRPM;
    int position_proportion_gain;
    int speed_proportion_gain;
    int integral_gain;
    int nSlowstart;
    int nSlowdown;
    int motor_pole;
    int motor_count;
    double motor_count_per_degree;
    std::string usb;

    uint16_t sSetDia;
    uint16_t sSetWheelLen;
    uint16_t sSetGear;

    std::string fb_state;

} ROBOT_PARAMETER_t;

extern ROBOT_PARAMETER_t robotParamData;
extern SETTINNG_PARAM_STEP_t byCntInitStep;
extern INIT_SETTING_STATE_t fgInitsetting;

// extern int InitSerial(void);
extern int PutMdData(PID_CMD_t pid, uint16_t rmid, const uint8_t *pData, uint16_t length);
extern void ReceiveDataFromController(rclcpp::Node::SharedPtr node);
void ResetOdom(void);
int InitSerialComm(rclcpp::Node::SharedPtr node);

int16_t *RobotSpeedToRPMSpeed(double linear, double angular,rclcpp::Node::SharedPtr node);
double *RPMSpeedToRobotSpeed(int16_t rpm_left, int16_t rpm_right,rclcpp::Node::SharedPtr node);

void CalRobotPoseFromRPM(PID_PNT_MAIN_DATA_t *pData,rclcpp::Node::SharedPtr node);
void CalRobotPoseFromPos(PID_PNT_MAIN_DATA_t *pData,rclcpp::Node::SharedPtr node);

void SetRPM(rclcpp::Node::SharedPtr node);

void PubRobotPose(const md_msgs::msg::Pose& msg_content, rclcpp::Node::SharedPtr node);
void PubRobotRPM(const md_msgs::msg::RPM& msg_content, rclcpp::Node::SharedPtr node);
void Status(PID_PNT_MAIN_DATA_t *pData, rclcpp::Node::SharedPtr node);

void InitMotorParameter(rclcpp::Node::SharedPtr node);
void parsingParameter(std::shared_ptr<rclcpp::Node> node);
