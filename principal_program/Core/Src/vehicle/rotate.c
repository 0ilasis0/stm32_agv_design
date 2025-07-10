#include <stdint.h>
#include "vehicle/rotate.h"
#include "main/fn_state.h"
#include "main/map.h"
#include "adc/main.h"



/**
  * @brief 根據強磁計數更新 AGV 方向資料
  */
static uint8_t look_rotate_count = 0;
static void renew_vehicle_rotation_status (MapDirF count_until_zero)
{
    //邊緣觸發判斷+時間預防
    bool triggered = false;
    uint32_t time_out = HAL_GetTick();
    uint32_t triggered_time;

    while (count_until_zero != 0){
        if (adchall_direction.value <= adchall_direction.const_h.magnetic_value  && !triggered)
        {
            count_until_zero --;
            triggered_time = HAL_GetTick();
            triggered = true;
        }
        if (
            adchall_direction.value > adchall_direction.const_h.magnetic_value
            && triggered_time - HAL_GetTick() > 200
            )
        {
            triggered = false;
        }

        look_rotate_count = count_until_zero;

        timeout_error(time_out, &error_state.renew_vehicle_rotation_status);
    }
    vehicle_ensure_stop();
}

/**
  * @brief AGV 原地旋轉直到對準方向
  */
void vehicle_rotate_in_place(MapDirF count, VehicleDirect currnet_direction, Percentage setpoint_speed)
{
    vehicle_set_direct(currnet_direction);
    vehicle_set_speed(setpoint_speed);

    renew_vehicle_rotation_status(count);

    agv_forward_leave_strong_magnet();
}
