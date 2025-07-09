#include <stdint.h>
#include "vehicle/rotate.h"
#include "main/fn_state.h"
#include "main/map.h"
#include "adc/main.h"



/**
  * @brief 判斷旋轉方向（順時針／逆時針）
  */
static VehicleMotion get_rotate_direction(int8_t start_dir, int8_t end_dir)
{
    int8_t diff = (end_dir - start_dir + 8) % 8;

    if (diff <= 3) {
        return motion_clockwise;

    } else {
        return motion_c_clockwise;

    }
}

/**
  * @brief 根據旋轉方向，計算在旋轉過程中會通過幾條磁條
  */
static uint8_t pass_magnetic_stripe_calculate(
    VehicleMotion rotate_direction_mode,
    uint16_t current_id_input,
    uint8_t from_dir,
    uint8_t to_dir
)
{
    uint8_t count = 0;

    // 取得目前節點（node）在 locations_t 中的索引值
    int current_id = get_index_by_id(current_id_input);

    if (rotate_direction_mode == motion_clockwise) {
        for (int i = (from_dir + 1) % 8; i != (to_dir + 1) % 8; i = (i + 1) % 8)
        {
            if (locations_t[current_id].connect[i].distance != 0)
            {
                count++;
            }
        }
    } else {
        for (int i = (from_dir - 1 + 8) % 8; i != (to_dir - 1 + 8) % 8; i = (i - 1 + 8) % 8)
        {
            if (locations_t[current_id].connect[i].distance != 0)
            {
                count++;
            }
        }
    }

    //若原方向上也有磁條，表示會需加一次
    if (locations_t[current_id].connect[from_dir].distance != 0)
    {
        count++;
    }

    return count;
}

/**
  * @brief 根據強磁計數更新 AGV 方向資料
  */
static uint8_t look_rotate_count = 0;
static void renew_vehicle_rotation_status (uint8_t count_until_zero)
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
  * @brief 偵測是否有初始方向數據，如果存在，則執行原地旋轉修正以對準起始航向
  */
void vehicle_adjust_startup_heading (void)
{
    if (map_data.start_address_id == no_data) return;

    VehicleMotion rotate_direction_mode = get_rotate_direction(
        map_data.start_direction,
        map_data.direction[0]
        );

    vehicle_set_motion(rotate_direction_mode);
    vehicle_set_speed(VEHICLE_setpoint_rotate);

    uint8_t renew_count = pass_magnetic_stripe_calculate(
        rotate_direction_mode,
        map_data.address_id[0],
        map_data.start_direction,
        map_data.direction[0]
        );

    renew_vehicle_rotation_status(renew_count);

}

/**
  * @brief AGV 原地旋轉直到對準方向
  */
void vehicle_rotate_in_place(void)
{
    if (map_data.current_count == 0) error_state.rotate_in_place__map_data_current_count = FNS_NOT_FOUND;

    VehicleMotion rotate_direction_mode = get_rotate_direction(
        map_data.direction[map_data.current_count - 1],
        map_data.direction[map_data.current_count]
        );

    uint8_t renew_count = pass_magnetic_stripe_calculate(
            rotate_direction_mode,
            map_data.address_id[map_data.current_count],
            map_data.direction[map_data.current_count - 1],
            map_data.direction[map_data.current_count]
            );

    vehicle_set_motion(rotate_direction_mode);
    vehicle_set_speed(VEHICLE_setpoint_rotate);

    renew_vehicle_rotation_status(renew_count);

    agv_forward_leave_strong_magnet();
}
