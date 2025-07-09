#include "vehicle/basic.h"
#include "main/adc.h"


VehicleState vehicle_state;

void vehicle_ensure_stop_inner(void)
{
    uint32_t loop_start = HAL_GetTick();
    for(;;)
    {
        if (HAL_GetTick() - loop_start > 10000)
        {
        }
        if (
               (motor_left.rps_present < MOTOR_STOP_GATE)
            && (motor_right.rps_present < MOTOR_STOP_GATE)
        ) break;
        osDelay(50);
    }
}

/**
  * @brief AGV 強迫前進不進行循跡直到離開強力磁鐵
  */
void agv_forward_leave_strong_magnet (void)
{
    vehicle_set_motion(motion_forward);
    vehicle_set_speed(VEHICLE_setpoint_straight);

    uint32_t error_start = HAL_GetTick();
    // 確保轉彎後能夠脫離強力磁鐵進入循跡
    while(adc_hall.sensor_node <= adc_hall.strong_magnet_value )
    {
        timeout_error(error_start, &error_state.agv_forward_leave_strong_magnet);
    }
}

/**
  * @brief 等待車輛完全停止
  */
void vehicle_ensure_stop(void)
{
    motor_set_state(&motor_left, MOTOR_STATE_SLOW);
    motor_set_state(&motor_right, MOTOR_STATE_SLOW);
    vehicle_ensure_stop_inner();
    motor_set_state(&motor_left, MOTOR_STATE_CONTROL);
    motor_set_state(&motor_right, MOTOR_STATE_CONTROL);
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
