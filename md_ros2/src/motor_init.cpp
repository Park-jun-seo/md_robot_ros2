#include "md_robot_node/global.hpp"
#include "md_robot_node/main.hpp"
#include "md_robot_node/com.hpp"

void InitMotorParameter(rclcpp::Node::SharedPtr node)
{
    switch (byCntInitStep)
    {
    case SETTING_PARAM_STEP_PID_PNT_VEL_CMD:
    {
        PID_PNT_VEL_CMD_t cmd_data, *p;

#if 0
            RCLCPP_INFO(node->get_logger(),"SET PID_PNT_VEL_CMD(%d)", PID_PNT_VEL_CMD);
            RCLCPP_INFO(node->get_logger(),"size of PID_PNT_VEL_CMD_t: %ld", sizeof(PID_PNT_VEL_CMD_t));
#endif

        p = &cmd_data;
        p->enable_id1 = 1;
        p->rpm_id1 = 0;
        p->enable_id2 = 1;
        p->rpm_id2 = 0;
        p->req_monitor_id = REQUEST_PNT_MAIN_DATA;
        PutMdData(PID_PNT_VEL_CMD, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data)); // 207

        byCntInitStep = SETTING_PARAM_STEP_PID_ROBOT_PARAM;
        break;
    }

    case SETTING_PARAM_STEP_PID_ROBOT_PARAM:
    {
        if (robotParamData.use_MDUI == 1)
        { // If using MDUI
            if (robotParamData.nRMID == robotParamData.nIDMDUI)
            {
                PID_ROBOT_PARAM_t cmd_data, *p;

#if 0
                    RCLCPP_INFO(node->get_logger(),"SETTING_PARAM_STEP_PID_ROBOT_PARAMv(%d)", SETTING_PARAM_STEP_PID_ROBOT_PARAM);
                    RCLCPP_INFO(node->get_logger(),"size of PID_ROBOT_PARAM_t: %ld", sizeof(c));
#endif
                p = &cmd_data;
                p->nDiameter = (uint16_t)robotParamData.nDiameter;
                p->nWheelLength = (uint16_t)robotParamData.nWheelLength * 1000; // m unit --> mm unit
                p->nGearRatio = (uint16_t)robotParamData.nGearRatio;
                PutMdData(PID_ROBOT_PARAM, MID_MDUI, (const uint8_t *)p, sizeof(cmd_data)); // 247

                byCntInitStep = SETTING_PARAM_STEP_PID_POSI_RESET;
            }
            else
            {
                RCLCPP_INFO(node->get_logger(), "err.mismatch ID(RMID(%d), MDUI(%d))", robotParamData.nRMID, robotParamData.nIDMDUI);
                fgInitsetting = INIT_SETTING_STATE_ERROR;
            }
        }
        else
        {
            byCntInitStep = SETTING_PARAM_STEP_PID_POSI_RESET;
        }
        break;
    }

    case SETTING_PARAM_STEP_PID_POSI_RESET:
    {
#if 0
            RCLCPP_INFO(node->get_logger(),"PID_POSI_RESET(%d)", PID_POSI_RESET);
#endif

        PutMdData(PID_POSI_RESET, robotParamData.nRMID, NULL, 0);

        ResetOdom();

        byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_START;
        break;
    }

    case SETTING_PARAM_STEP_PID_SLOW_START:
    {
        PID_SLOW_START_t cmd_data, *p;

#if 1
        RCLCPP_INFO(node->get_logger(), "PID_SLOW_START(%d)", PID_SLOW_START);
        RCLCPP_INFO(node->get_logger(), "size of PID_SLOW_START_t: %ld", sizeof(PID_SLOW_START_t));
#endif

        p = &cmd_data;
        p->value = robotParamData.nSlowstart;

        PutMdData(PID_SLOW_START, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data));

        byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_DOWN;
        break;
    }

    case SETTING_PARAM_STEP_PID_SLOW_DOWN:
    {
        PID_SLOW_DOWN_t cmd_data, *p;

#if 1
        RCLCPP_INFO(node->get_logger(), "PID_SLOW_DOWN(%d)", PID_SLOW_DOWN);
        RCLCPP_INFO(node->get_logger(), "size of PID_SLOW_DOWN_t: %ld", sizeof(PID_SLOW_DOWN_t));
#endif

        p = &cmd_data;
        p->value = robotParamData.nSlowdown;

        PutMdData(PID_SLOW_DOWN, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data));

        byCntInitStep = SETTING_PARAM_STEP_PID_GAIN;
        break;
    }

    case SETTING_PARAM_STEP_PID_GAIN:
    {
        PID_GAIN_t cmd_data, *p;

#if 1
        RCLCPP_INFO(node->get_logger(), "PID_GAIN(%d)", PID_GAIN);
        RCLCPP_INFO(node->get_logger(), "size of PID_GAIN: %ld", sizeof(PID_GAIN_t));
#endif

        p = &cmd_data;
        p->position_proportion_gain = robotParamData.position_proportion_gain;
        p->speed_proportion_gain = robotParamData.speed_proportion_gain;
        p->integral_gain = robotParamData.integral_gain;

        PutMdData(PID_GAIN, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data));

        byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD;
        break;
    }
    //////////////////////////////////////////////////////
    case SETTING_PARAM_STEP_PID_INV_SIGH_CMD: // Left motor
    {
        uint8_t cmd_data;

#if 1
        RCLCPP_INFO(node->get_logger(), "PID_INV_SIGN_CMD(%d)", PID_INV_SIGN_CMD);
#endif

        if (robotParamData.reverse_direction == 0)
        {
            cmd_data = 1;
        }
        else
        {
            cmd_data = 0;
        }

        PutMdData(PID_INV_SIGN_CMD, robotParamData.nRMID, (const uint8_t *)&cmd_data, 1);

        byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD2;
        break;
    }
    //////////////////////////////////////////////////////
    case SETTING_PARAM_STEP_PID_INV_SIGH_CMD2: // Right motor
    {
        uint8_t cmd_data;

#if 1
        RCLCPP_INFO(node->get_logger(), "PID_INV_SIGN_CMD2(%d)", PID_INV_SIGN_CMD2);
#endif

        if (robotParamData.reverse_direction == 0)
        {
            cmd_data = 0;
        }
        else
        {
            cmd_data = 1;
        }

        PutMdData(PID_INV_SIGN_CMD2, robotParamData.nRMID, (const uint8_t *)&cmd_data, 1);

        byCntInitStep = SETTING_PARAM_STEP_PID_REGENERATION;
        break;
    }

    case SETTING_PARAM_STEP_PID_REGENERATION:
    {
        uint8_t cmd_data;

#if 1
        RCLCPP_INFO(node->get_logger(), "PID_REGENERATION(%d)", PID_REGENERATION);
#endif

        cmd_data = 0; // PID_REGENERATION OFF

        PutMdData(PID_REGENERATION, robotParamData.nRMID, (const uint8_t *)&cmd_data, 1);

        byCntInitStep = SETTING_PARAM_STEP_PID_USE_EPOSI;
        break;
    }

    case SETTING_PARAM_STEP_PID_USE_EPOSI:
    {
        uint8_t cmd_data;

#if 1
        RCLCPP_INFO(node->get_logger(), "PID_USE_POSI(%d)", PID_USE_POSI);
#endif

        if (robotParamData.motor_position_type == 0)
        {
            cmd_data = 0; // hall sensor
        }
        else
        {
            cmd_data = 1; // encoder
        }

        PutMdData(PID_USE_POSI, robotParamData.nRMID, (const uint8_t *)&cmd_data, 1);

        byCntInitStep = SETTING_PARAM_STEP_PID_COM_WATCH_DELAY;
        break;
    }

    case SETTING_PARAM_STEP_PID_COM_WATCH_DELAY:
    {
        // uint8_t cmd_data;

#if 1
        RCLCPP_INFO(node->get_logger(), "PID_COM_WATCH_DELAY(%d)", PID_COM_WATCH_DELAY);
#endif

        // cmd_data = 10; // watch_delay (*0.1)
        PID_WATCH_DELAY_DATA_t cmd_data, *p;
        p = &cmd_data;
        p->data1 = 10;
        p->data2 = 10;
        PutMdData(PID_COM_WATCH_DELAY, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data));

        byCntInitStep = SETTING_PARAM_STEP_PID_STOP_STATUS;
        break;
    }

    case SETTING_PARAM_STEP_PID_STOP_STATUS:
    {
        uint8_t cmd_data;

#if 1
        RCLCPP_INFO(node->get_logger(), "PID_STOP_STATUS(%d)", PID_STOP_STATUS);
#endif

        cmd_data = 3; // STOP_SERVO_LOCK
                      // 0 : 기준속도입력 0 인 경우 모터를 강제정지하고
                      // 속도가 0 에 도달하면 제어상태 해제(FREE)
                      // 1 : 모터가 정지되면 정지한 위치를 계속하여
                      // 유지
                      // 2 : 모터가 정지되면 젂기적브레이크 적용
                      // 3 : 기준입력이 0 이 되면 제어상태 해제(FREE)
                      // 4 : 리미트스위치에 의한 동작에서 속도 0 으로
                      // 제어후에 모터정지상태에서 브레이크 동작
                      // 5 : START/STOP 의 입력으로 속도제어하는
                      // 경우에, 이 싞호가 OFF 이 되면
                      // RUN/BRAKE 싞호와 동일한 동작을
                      // 수행(RUN/BRAKE 가 없는 모델에서 주로적용)
                      // MD500S, MD200T, etc.
                      // 183, TMID, ID, 24, 1, DATA, CHK

        PutMdData(PID_STOP_STATUS, robotParamData.nRMID, (const uint8_t *)&cmd_data, 1);

        byCntInitStep = SETTING_PARAM_STEP_PID_PPR;
        break;
    }

    case SETTING_PARAM_STEP_PID_PPR:
    {
        PID_PPR_t cmd_data, *p;

#if 1
        RCLCPP_INFO(node->get_logger(), "PID_PPR(%d)", PID_PPR);
#endif
        p = &cmd_data;

        p->PPR = robotParamData.encoder_PPR;

        PutMdData(PID_PPR, robotParamData.nRMID, (const uint8_t *)&cmd_data, sizeof(PID_PPR_t));

        byCntInitStep = SETTING_PARAM_STEP_DONE;

        fgInitsetting = INIT_SETTING_STATE_OK;
        break;
    }

    default:
        break;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////