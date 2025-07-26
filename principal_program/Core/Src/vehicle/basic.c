#include "vehicle/basic.h"
#include "adc/main.h"

VehicleParameter vehicle_h;

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
  * @brief 等待車輛完全停止
  */
void vehicle_ensure_stop(void)
{
    motor_set_state(&motor_left, MOTOR_STATE_SLOW_0);
    motor_set_state(&motor_right, MOTOR_STATE_SLOW_0);
    vehicle_ensure_stop_inner();
    motor_set_state(&motor_left, MOTOR_STATE_CONTROL);
    motor_set_state(&motor_right, MOTOR_STATE_CONTROL);
}

/**
  * @brief AGV 強迫前進不進行循跡直到離開強力磁鐵
  */
// void agv_forward_leave_strong_magnet (void)
// {
//     vehicle_set_motion(VEHICLE_MOTION_FORWARD);
//     vehicle_set_speed(MAP_SETPOINT_TRACK);

//     uint32_t error_start = HAL_GetTick();
//     // 確保轉彎後能夠脫離強力磁鐵進入循跡
//     while(adchall_node.state != ADC_HALL_STATE_NONE )
//     {
//         timeout_error(error_start, &error_state.agv_forward_leave_strong_magnet);
//     }
// }

void vehicle_set_mode(VehicleMode mode)
{
    switch (mode)
    {
        case VEHICLE_MODE_TRACK:
        case VEHICLE_MODE_ROTATE:
        {
            vehicle_h.last_tick_on_mag = HAL_GetTick();
            break;
        }
        default: break;
    }
    vehicle_h.mode = mode;
}

void vehicle_set_motion(VehicleMotion motion)
{
    vehicle_h.motion = motion;
}

void vehicle_set_speed(Percentage value)
{
    if (value > 100) value = 100;
    vehicle_h.speed = value;
    motor_set_rps_pcn(&motor_right, value);
    motor_set_rps_pcn(&motor_left , value);
}

void vehicle_set_need_rotate(uint8_t value)
{
    vehicle_h.need_rotate_count = value;
}

void vehicle_stop(void)
{
    vehicle_set_motion(VEHICLE_MOTION_STOP);
    vehicle_ensure_stop();
}
