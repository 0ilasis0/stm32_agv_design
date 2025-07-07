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

void vehicle_set_mode(VehicleMode mode)
{
    vehicle_state.mode = mode;
}

void vehicle_set_motion(VehicleMotion motion)
{
    vehicle_state.motion = motion;
}

void vehicle_set_speed(Percentage value)
{
    if (value > 100) value = 100;
    vehicle_state.speed = value;
    motor_set_rps_pcn(&motor_right, value);
    motor_set_rps_pcn(&motor_left , value);
}
