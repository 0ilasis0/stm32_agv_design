#include <stdint.h>
#include "vehicle/rotate.h"
#include "main/fn_state.h"
#include "main/map.h"
#include "adc/main.h"




/**
  * @brief AGV 原地旋轉直到對準方向，根據強磁計數更新 AGV 方向資料
  */
static uint8_t look_rotate_count = 0;
void vehicle_rotate_in_place(MapDirF count, VehicleDirect currnet_direction, Percentage setpoint_speed)
{
    vehicle_set_direct(currnet_direction);
    vehicle_set_speed(setpoint_speed);

    //邊緣觸發判斷+時間預防
    bool triggered = false;
    uint32_t time_out = HAL_GetTick();
    uint32_t triggered_time;

    while (count != 0){
        if (adchall_direction.value <= adchall_direction.const_h.magnetic_value  && !triggered)
        {
            count --;
            triggered_time = HAL_GetTick();
            triggered = true;
        }
        if (
            adchall_direction.value > adchall_direction.const_h.magnetic_value
            && triggered_time - HAL_GetTick() > NAVUGATION_TIME_DIFF
            )
        {
            triggered = false;
        }

        look_rotate_count = count;

        timeout_error(time_out, &error_state.vehicle_rotate_in_place);
    }
    vehicle_ensure_stop();
    vehicle_set_mode(VEHICLE_MODE_TRACK);

    // agv_forward_leave_strong_magnet();
}
