#include "vehicle/main.h"

/**
  * @brief 測試空載情況下的馬達最大速度
  * 僅使用右邊測試空載轉速
  */
void vehicle_test_no_load_speed(uint32_t ms)
{
    if (!sys_run_switch.enable_debug_test_no_load_speed) return;
    sys_run_switch.enable_rps_control = 0;

    vehicle_set_motion(motion_forward);
    uint32_t past_time = HAL_GetTick()
            ,previous_time_dif = past_time;
    motor_set_duty(&motor_left,  100);
    motor_set_duty(&motor_right, 100);
    while (
        HAL_GetTick() - past_time < ms || motors_max_rps <= 10
    ) {
        if (motors_max_rps < motor_right.rps_present) {
                motors_max_rps = motor_right.rps_present;
                past_time = HAL_GetTick();
        }

        timeout_error(previous_time_dif, &error_state.vehicle_test_no_load_speed);
    }
    motor_set_duty(&motor_left,  0);
    motor_set_duty(&motor_right, 0);
    previous_time_dif = HAL_GetTick() - previous_time_dif;

    vehicle_ensure_stop();
    osDelay(1000);

    vehicle_set_motion(motion_backward);
    past_time = HAL_GetTick();
    motor_set_duty(&motor_left,  100);
    motor_set_duty(&motor_right, 100);
    while(HAL_GetTick() - past_time <= previous_time_dif) {
        timeout_error(past_time, &error_state.vehicle_test_no_load_speed);
    }
    motor_set_duty(&motor_left,  0);
    motor_set_duty(&motor_right, 0);
    vehicle_ensure_stop();
    
    sys_run_switch.enable_rps_control = 1;
}
