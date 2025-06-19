#include "main/vehicle2.h"
#include "motor/PI_control.h"
#include "main/const_and_error.h"


// 判斷磁條強度大小
uint32_t hall_magnetic_stripe_value = HALL_MAGNITUTE_EDGE;
// 判斷強力磁鐵強度大小
uint32_t hall_strong_magnet_value = HALL_MAGNITUTE_EDGE;

/*測試用--------------------------------------*/
uint32_t hall_sensor_node = HALL_MAGNITUTE_EDGE + 1;
uint32_t hall_sensor_direction = 0;
/*測試用--------------------------------------*/

/**
  * @brief 根據運動模式控制馬達旋轉方向與設定速度
  */
void vehicle2_motion_and_speed_control(MOTIONCOMMAND mode, uint8_t sepoint_value){
    vehicle2_ensure_motor_stop();

    switch(mode) {
        case motion_forward:
            motor_set_direction(&motor_right, clockwise);
            motor_set_direction(&motor_left,  counter_clockwise);
            break;

        case motion_backward:
            motor_set_direction(&motor_right, counter_clockwise);
            motor_set_direction(&motor_left,  clockwise);
            break;

        case motion_clockwise:
            motor_set_direction(&motor_right, counter_clockwise);
            motor_set_direction(&motor_left,  counter_clockwise);
            break;

        case motion_counter_clockwise:
            motor_set_direction(&motor_right, clockwise);
            motor_set_direction(&motor_left,  clockwise);
            break;
    }

    motor_set_speed_setpoint(&motor_right, sepoint_value);
    motor_set_speed_setpoint(&motor_left , sepoint_value);
}

/**
  * @brief 判斷旋轉方向（順時針／逆時針）
  */
ROTATE_STATUS vehicle2_get_rotate_direction(int8_t start_dir, int8_t end_dir) {
    int8_t diff = (end_dir - start_dir + 8) % 8;

    if (diff == 0) {
        return either;                               // 旋轉完成

    } else if (diff <= 3) {
        return clockwise;

    } else {
        return counter_clockwise;

    }
}

/**
  * @brief 等待左右馬達完全停止
  */
void vehicle2_ensure_motor_stop(void) {
    //
    motor_right.speed_sepoint_pcn = 0;
    motor_left.speed_sepoint_pcn  = 0;

    uint32_t error_start = HAL_GetTick();
    while(motor_right.speed_present != 0 || motor_left.speed_present != 0) {
        if (!timeout_error(error_start, &error_state.vehicle2_ensure_motor_stop)) break;
    }
}

/**
  * @brief 根據旋轉方向，計算在旋轉過程中會通過幾條磁條
  */
uint8_t vehicle2_pass_magnetic_stripe_calculate(
    ROTATE_STATUS rotate_direction_mode,
    uint16_t current_id_input,
    uint8_t from_dir,
    uint8_t to_dir
) {
    uint8_t count = 0;

    // 取得目前節點（node）在 locations_t 中的索引值
    int current_id = get_index_by_id(current_id_input);

    if (rotate_direction_mode == clockwise) {
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
uint8_t text_count_until_zero;
void vehicle2_renew_vehicle_rotation_status (uint8_t count_until_zero) {
    //邊緣觸發判斷
    bool triggered = false;

    uint32_t time_out = HAL_GetTick();
    while (count_until_zero != 0){
        if (hall_sensor_direction >= hall_magnetic_stripe_value  && !triggered) {
            count_until_zero --;
            triggered = true;
        }
        if (hall_sensor_direction < hall_magnetic_stripe_value) {
            triggered = false;
        }
        text_count_until_zero = count_until_zero;

        if (!timeout_error(time_out, &error_state.vehicle2_renew_vehicle_rotation_status)) break;
    }
}

/**
  * @brief ROTATE_STATUS 轉 MOTIONCOMMAND
  */
MOTIONCOMMAND vehicle2_rotate_status_to_motioncommand (ROTATE_STATUS mode) {
    if (mode == clockwise) {
        return motion_clockwise;
    } else {
        return motion_counter_clockwise;
    }

}
