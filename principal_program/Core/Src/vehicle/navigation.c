#include "vehicle/navigation.h"
#include "tim.h"
#include "vehicle/basic.h"
#include "vehicle/search.h"
#include "adc/main.h"
#include "main/config.h"

/**
 * @brief 循跡模式
 */
FnState vehicle_track_mode(uint32_t unfind_ms)
{
    MotorParameter *motor_l, *motor_r;
    switch (vehicle_h.motion)
    {
        case VEHICLE_MOTION_FORWARD:
        {
            motor_l = &motor_left;
            motor_r = &motor_right;
            break;
        }
        case VEHICLE_MOTION_BACKWARD:
        {
            motor_l = &motor_right;
            motor_r = &motor_left;
            break;
        }
        default:
        {
            vehicle_set_motion(VEHICLE_MOTION_STOP);
            vehicle_ensure_stop();
            vehicle_set_mode(VEHICLE_MODE_FREE);
            return FNS_INVALID;
        }
    }
    if (
           adchall_track_left.state == ADC_HALL_STATE_NONE
        && adchall_track_right.state != ADC_HALL_STATE_NONE
        && adchall_direction.state == ADC_HALL_STATE_NONE
    ) {
        vehicle_h.last_tick_on_mag = HAL_GetTick();
        motor_set_state(motor_l, MOTOR_STATE_CONTROL);
        motor_set_state(motor_r, MOTOR_STATE_SLOW);
    }
    else if
    (
           adchall_track_left.state != ADC_HALL_STATE_NONE
        && adchall_track_right.state == ADC_HALL_STATE_NONE
        && adchall_direction.state == ADC_HALL_STATE_NONE
    ) {
        vehicle_h.last_tick_on_mag = HAL_GetTick();
        motor_set_state(motor_l, MOTOR_STATE_SLOW);
        motor_set_state(motor_r, MOTOR_STATE_CONTROL);
    }
    else
    {
        if (adchall_direction.state != ADC_HALL_STATE_NONE)
        {
            vehicle_h.last_tick_on_mag = HAL_GetTick();
        }
        motor_set_state(motor_l, MOTOR_STATE_CONTROL);
        motor_set_state(motor_r, MOTOR_STATE_CONTROL);
    }
    if (HAL_GetTick() - vehicle_h.last_tick_on_mag >= unfind_ms)
    {
        vehicle_set_motion(VEHICLE_MOTION_STOP);
        vehicle_ensure_stop();
        vehicle_set_mode(VEHICLE_MODE_SEARCH);
        return FNS_FAIL;
    }
    return FNS_OK;
}

/**
  * @brief AGV 原地旋轉直到對準方向，根據強磁計數更新 AGV 方向資料
  */
FnState vehicle_rotate_in_place(uint32_t unfind_ms)
{
    //邊緣觸發判斷+時間預防
    bool mag_trigger = 1;
    // uint32_t time_out = HAL_GetTick();
    // uint32_t triggered_time;

    while (vehicle_h.need_rotate_count != 0){
        if (mag_trigger)
        {
            if (adchall_direction.state == ADC_HALL_STATE_NONE)
            {
                mag_trigger = 0;
            }
        }
        else
        {
            if (adchall_direction.state != ADC_HALL_STATE_NONE)
            {
                mag_trigger = 1;
                vehicle_h.need_rotate_count--;
                vehicle_h.last_tick_on_mag = HAL_GetTick();
            }
        }
        if (HAL_GetTick() - vehicle_h.last_tick_on_mag >= unfind_ms)
        {
            vehicle_set_motion(VEHICLE_MOTION_STOP);
            vehicle_ensure_stop();
            vehicle_set_mode(VEHICLE_MODE_FREE);
            return FNS_FAIL;
        }
        osDelay(50);
    }
    
    return FNS_OK;
}
