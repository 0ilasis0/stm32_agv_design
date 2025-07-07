#include "vehicle/basic.h"

VehicleState vehicle_state;

/**
  * @brief 等待車輛完全停止
  */
void vehicle_ensure_stop(void)
{
    motor_set_state(&motor_right, MOTOR_STATE_SLOW);
    motor_set_state(&motor_left, MOTOR_STATE_SLOW);
    uint32_t error_start = HAL_GetTick();
    for(;;)
    {
        if (HAL_GetTick() - error_start > 10000)
        {
        }
        if (
               (motor_right.rps_present == 0)
            && (motor_left.rps_present == 0)
        ) break;
        osDelay(50);
    }
    motor_set_state(&motor_right, MOTOR_STATE_FREE);
    motor_set_state(&motor_left, MOTOR_STATE_FREE);
}

/**
  * @brief 根據運動模式控制馬達旋轉方向與設定速度
  */
void vehicle_set_motion(MotionCommand mode)
{
    vehicle_ensure_stop();
    switch(mode)
    {
        case motion_forward:
        {
            motor_set_direction(&motor_left,  MOTOR_ROTATE_CCLW);
            motor_set_direction(&motor_right, MOTOR_ROTATE_CLW);
            vehicle_state.motion_present = motion_forward;
            break;
        }
        case motion_backward:
        {
            motor_set_direction(&motor_left,  MOTOR_ROTATE_CLW);
            motor_set_direction(&motor_right, MOTOR_ROTATE_CCLW);
            vehicle_state.motion_present = motion_backward;
            break;
        }
        case motion_clockwise:
        {
            motor_set_direction(&motor_left,  MOTOR_ROTATE_CCLW);
            motor_set_direction(&motor_right, MOTOR_ROTATE_CCLW);
            vehicle_state.motion_present = motion_clockwise;
            break;
        }
        case motion_c_clockwise:
        {
            motor_set_direction(&motor_left,  MOTOR_ROTATE_CLW);
            motor_set_direction(&motor_right, MOTOR_ROTATE_CLW);
            vehicle_state.motion_present = motion_c_clockwise;
            break;
        }
        default: break;
    }
}

void vehicle_set_speed(uint8_t value)
{
    motor_set_speed(&motor_right, value);
    motor_set_speed(&motor_left , value);
}
