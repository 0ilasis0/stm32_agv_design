#include <stdint.h>
#include "vehicle/basic.h"
#include "vehicle/rotate.h"
#include "vehicle/navigation.h"
#include "main/fn_state.h"
#include "adc/main.h"




/**
  * @brief AGV 原地旋轉直到對準方向，根據強磁計數更新 AGV 方向資料
  */
static uint8_t look_rotate_count = 0;
void vehicle_rotate_in_place()
{
    //邊緣觸發判斷+時間預防
    bool triggered = false;
    uint32_t time_out = HAL_GetTick();
    uint32_t triggered_time;

    while (agv_state.real_rotate_count != 0){
        if (adchall_direction.value <= adchall_direction.const_h.magnetic_value  && !triggered)
        {
            agv_state.real_rotate_count --;
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

        look_rotate_count = agv_state.real_rotate_count;

        timeout_error(time_out, &error_state.vehicle_rotate_in_place);
    }

    vehicle_ensure_stop();
    vehicle_set_mode(VEHICLE_MODE_TRACK);
}
