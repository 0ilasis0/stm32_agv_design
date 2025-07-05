#include "main/vehicle2.h"
#include "main/fn_state.h"
#include "main/map.h"

/**
  * @brief 根據運動模式控制馬達旋轉方向與設定速度
  */
void vehicle2_motion_and_speed_control(MotionCommand mode, uint8_t sepoint_value)
{
    if (mode != agv_state.vehicle_currnet_mode) vehicle2_ensure_motor_stop();
    switch(mode) {
        case motion_forward:
            motor_set_direction(&motor_right, rotate_clockwise);
            motor_set_direction(&motor_left,  rotate_c_clockwise);
            agv_state.vehicle_currnet_mode = motion_forward;
            break;
        case motion_backward:
            motor_set_direction(&motor_right, rotate_c_clockwise);
            motor_set_direction(&motor_left,  rotate_clockwise);
            agv_state.vehicle_currnet_mode = motion_backward;
            break;
        case motion_clockwise:
            motor_set_direction(&motor_right, rotate_c_clockwise);
            motor_set_direction(&motor_left,  rotate_c_clockwise);
            agv_state.vehicle_currnet_mode = motion_clockwise;
            break;
        case motion_c_clockwise:
            motor_set_direction(&motor_right, rotate_clockwise);
            motor_set_direction(&motor_left,  rotate_clockwise);
            agv_state.vehicle_currnet_mode = motion_c_clockwise;
            break;
    }

    motor_set_speed(&motor_right, sepoint_value);
    motor_set_speed(&motor_left , sepoint_value);
}

/**
  * @brief 判斷旋轉方向（順時針／逆時針）
  */
MotionCommand vehicle2_get_rotate_direction(int8_t start_dir, int8_t end_dir)
{
    int8_t diff = (end_dir - start_dir + 8) % 8;

    if (diff <= 3) {
        return motion_clockwise;

    } else {
        return motion_c_clockwise;

    }
}

/**
  * @brief 等待左右馬達完全停止
  */
void vehicle2_ensure_motor_stop(void) {

    agv_state.vehicle_currnet_mode = motion_stop;

    // Todo ?
    // motor_right.rps_sepoint = 0;
    // motor_left.rps_sepoint  = 0;

    uint32_t error_start = HAL_GetTick();
    while(motor_right.rps_present != 0 || motor_left.rps_present != 0)
    {
        timeout_error(error_start, &error_state.vehicle2_ensure_motor_stop);
    }
}

/**
  * @brief 根據旋轉方向，計算在旋轉過程中會通過幾條磁條
  */
uint8_t vehicle2_pass_magnetic_stripe_calculate(
    MotionCommand rotate_direction_mode,
    uint16_t current_id_input,
    uint8_t from_dir,
    uint8_t to_dir
) {
    uint8_t count = 0;

    // 取得目前節點（node）在 locations_t 中的索引值
    int current_id = get_index_by_id(current_id_input);

    if (rotate_direction_mode == motion_clockwise) {
        for (int i = (from_dir + 1) % 8; i != (to_dir + 1) % 8; i = (i + 1) % 8) {
            if (locations_t[current_id].connect[i].distance != 0) {
                count++;
            }
        }
    } else {
        for (int i = (from_dir - 1 + 8) % 8; i != (to_dir - 1 + 8) % 8; i = (i - 1 + 8) % 8) {
            if (locations_t[current_id].connect[i].distance != 0) {
                count++;
            }
        }
    }

    //若原方向上也有磁條，表示會需加一次
    if (locations_t[current_id].connect[from_dir].distance != 0) {
        count++;
    }

    return count;
}

/**
  * @brief 根據強磁計數更新 AGV 方向資料
  */
static uint8_t look_rotate_count = 0;
void vehicle2_renew_vehicle_rotation_status (uint8_t count_until_zero)
{
    //邊緣觸發判斷+時間預防
    bool triggered = false;
    uint32_t time_out = HAL_GetTick();

    while (count_until_zero != 0){
        if (adc_hall.sensor_direction <= adc_hall.magnetic_stripe_value  && !triggered) {
            count_until_zero --;
            triggered = true;
        }
        if (adc_hall.sensor_direction > adc_hall.magnetic_stripe_value && time_out - HAL_GetTick() > 500) {
            triggered = false;
        }

        look_rotate_count = count_until_zero;

        timeout_error(time_out, &error_state.vehicle2_renew_vehicle_rotation_status);
    }
    vehicle2_ensure_motor_stop();
}
