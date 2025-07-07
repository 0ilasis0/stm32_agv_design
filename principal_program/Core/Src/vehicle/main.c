#include "vehicle/main.h"

/**
  * @brief 測試空載情況下的馬達最大速度
  */
void vehicle_test_no_load_rps(uint32_t ms)
{
    if (!sys_run_switch.enable_debug_test_no_load_speed) return;

    sys_run_switch.enable_rps_control = 0;

    vehicle_set_motion(motion_forward);
    uint32_t past_time = HAL_GetTick(), time_diff = past_time;
    motor_set_duty(&motor_left,  100);
    motor_set_duty(&motor_right, 100);
    for(;;)
    {
        if (HAL_GetTick() - past_time > ms) break;
        if (motor_left.rps_present > motor_left.rps_max)
        {
            motor_set_max_rps(&motor_left, motor_left.rps_present);
            past_time = HAL_GetTick();
        }
        if (motor_right.rps_present > motor_right.rps_max)
        {
            motor_set_max_rps(&motor_right, motor_right.rps_present);
            past_time = HAL_GetTick();
        }
        osDelay(10);
        // timeout_error(previous_time_diff, &error_state.vehicle_test_no_load_rps);
    }
    motor_set_duty(&motor_left,  0);
    motor_set_duty(&motor_right, 0);
    if (motor_left.rps_max > motor_right.rps_max) motor_set_max_rps(&motor_left, motor_right.rps_max);
    else motor_set_max_rps(&motor_right, motor_left.rps_max);
    time_diff = HAL_GetTick() - time_diff;
    vehicle_ensure_stop();
    osDelay(1000);

    vehicle_set_motion(motion_backward);
    past_time = HAL_GetTick();
    motor_set_duty(&motor_left,  100);
    motor_set_duty(&motor_right, 100);
    for(;;)
    {
        if (HAL_GetTick() - past_time >= time_diff) break;
        osDelay(10);
        // timeout_error(past_time, &error_state.vehicle_test_no_load_rps);
    }
    motor_set_duty(&motor_left,  0);
    motor_set_duty(&motor_right, 0);
    vehicle_ensure_stop();

    sys_run_switch.enable_rps_control = 1;
}
