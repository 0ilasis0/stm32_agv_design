#include "tim.h"
#include "vehicle/basic.h"
#include "vehicle/navigation.h"
#include "vehicle/search.h"
#include "main/fn_state.h"
#include "adc/main.h"
#include "main/config.h"



/**
 * @brief 循跡模式
 */
void vehicle_track_mode()
{
    if (
           adchall_track_left.value  >  adchall_track_left.const_h.magnetic_value
        && adchall_track_right.value <= adchall_track_right.const_h.magnetic_value
    ) {
        vehicle_parameter.last_tick_on_mag = HAL_GetTick();
        motor_set_state(&motor_left, MOTOR_STATE_CONTROL);
        motor_set_state(&motor_right, MOTOR_STATE_SLOW);
    }
    else if
    (
           adchall_track_left.value  <= adchall_track_left.const_h.magnetic_value
        && adchall_track_right.value >  adchall_track_right.const_h.magnetic_value
    ) {
        vehicle_parameter.last_tick_on_mag = HAL_GetTick();
        motor_set_state(&motor_left, MOTOR_STATE_SLOW);
        motor_set_state(&motor_right, MOTOR_STATE_CONTROL);
    }
    else
    {
        if (adchall_direction.value <= adchall_direction.const_h.magnetic_value)
        {
            vehicle_parameter.last_tick_on_mag = HAL_GetTick();
        }
        motor_set_state(&motor_left, MOTOR_STATE_CONTROL);
        motor_set_state(&motor_right, MOTOR_STATE_CONTROL);
    }
    if (HAL_GetTick() - vehicle_parameter.last_tick_on_mag >= UNFIND_MAG_TIME)
    {
        // vehicle_set_mode(VEHICLE_MODE_SEARCH);
    }
}

/**
  * @brief AGV 原地旋轉直到對準方向，根據強磁計數更新 AGV 方向資料
  */
void vehicle_rotate_in_place(void)
{
    //邊緣觸發判斷+時間預防
    bool triggered = false;
    uint32_t time_out = HAL_GetTick();
    uint32_t triggered_time;

    while (vehicle_parameter.need_rotate_count != 0){
        if (adchall_direction.value <= adchall_direction.const_h.magnetic_value  && !triggered)
        {
            vehicle_parameter.need_rotate_count --;
            triggered_time = HAL_GetTick();
            triggered = true;
        }
        if (
            adchall_direction.value > adchall_direction.const_h.magnetic_value
            && triggered_time - HAL_GetTick() > MAGNATIC_STRIPE_TIME_DIF
            )
        {
            triggered = false;
        }

        timeout_error(time_out, &error_state.vehicle_rotate_in_place);
    }

    vehicle_ensure_stop();
    vehicle_set_mode(VEHICLE_MODE_TRACK);
    vehicle_set_motion(VEHICLE_DIRECT_FORWARD);
    vehicle_set_speed(VEHICLE_SETPOINT_TRACK);
}
