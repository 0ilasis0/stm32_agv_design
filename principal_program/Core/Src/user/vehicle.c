#include "user/vehicle.h"
#include "user/PI_control.h"
#include "user/user_adc.h"
#include "user/user_it.h"
#include "user/const_and_error.h"
#include "tim.h"
#include <math.h>
#include "stm32g4xx_hal.h"

// 判斷磁條強度大小
uint32_t hall_magnetic_stripe_value = HALL_MAGNITUTE_EDGE;
// 判斷強力磁鐵強度大小
uint32_t hall_strong_magnet_value = HALL_MAGNITUTE_EDGE;

/*測試用--------------------------------------*/
uint32_t hall_sensor_direction = 0;
uint32_t text_return_start_time = 0;
uint32_t text_time = 0;
/*測試用--------------------------------------*/

AGV_STATUS agv_current_status = agv_straight;
MOTIONCOMMAND direction_mode;


/**
  * @brief 根據運動模式控制馬達旋轉方向與設定速度
  */
void vehicle_motion_and_speed_control(MOTIONCOMMAND mode, uint8_t sepoint_value){
    vehicle_ensure_motor_stop();

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
  * @brief 一般循跡模式控制
  */
void vehicle_track_mode(void) {
    adc_renew();

    vehicle_breakdown_all_hall_lost ();

    if (motor_right.adc_value >= hall_magnetic_stripe_value) {
        motor_set_speed_setpoint(&motor_left, setpoint_straight);
        motor_set_speed_setpoint(&motor_right, 0);

    } else if (motor_left.adc_value >= hall_magnetic_stripe_value) {
        motor_set_speed_setpoint(&motor_left, 0);
        motor_set_speed_setpoint(&motor_right, setpoint_straight);

    } else {
        motor_set_speed_setpoint(&motor_left, setpoint_straight);
        motor_set_speed_setpoint(&motor_right, setpoint_straight);
    }
}

/**
  * @brief 當所有相關的霍爾感測器都失去磁條訊號時，嘗試重新搜尋並回到磁條路徑上
  */
void vehicle_breakdown_all_hall_lost (void) {
    if (!debug_breakdown_all_hall_lost_enable) return;

    if (
        hall_sensor_direction < hall_magnetic_stripe_value &&
        hall_sensor_node      < hall_magnetic_stripe_value &&
        motor_right.adc_value < hall_magnetic_stripe_value &&
        motor_left.adc_value  < hall_magnetic_stripe_value
        ) {
            vehicle_ensure_motor_stop();
            vehicle_search_magnetic_path (motion_clockwise, 3000);
            vehicle_search_magnetic_path (motion_counter_clockwise, 6000);
            if (search_magnetic_path_enable == 1) {
                while (true) error_data.breakdown_all_hall_lost__path_not_found = 1;
            }

            search_magnetic_path_enable = 1;
    }
}

/**
  * @brief AGV 原地旋轉直到對準方向
  */
void vehicle_rotate_in_place(void) {
    if (map_data.current_count == 0) error_data.rotate_in_place__map_data_current_count = 1;

    ROTATE_STATUS rotate_direction_mode = vehicle_get_rotate_direction(map_data.direction[map_data.current_count - 1], map_data.direction[map_data.current_count]);

    if (rotate_direction_mode != either) {
        uint8_t renew_count = vehicle_pass_magnetic_stripe_calculate(
                rotate_direction_mode,
                map_data.address_id[map_data.current_count],
                map_data.direction[map_data.current_count - 1],
                map_data.direction[map_data.current_count]
                );

        vehicle_motion_and_speed_control(vehicle_rotate_status_to_motioncommand(rotate_direction_mode), setpoint_rotate);

        vehicle_renew_vehicle_rotation_status(renew_count);
    }

    vehicle_motion_and_speed_control(motion_forward, setpoint_straight);
    uint32_t error_start = HAL_GetTick();
    // 確保轉彎後能夠脫離強力磁鐵進入循跡
    while(hall_sensor_node >= hall_strong_magnet_value ) {
        if (!timeout_error(error_start, &error_timeout.rotate_in_place_hall)) break;
    }
}

/**
  * @brief 根據旋轉方向，計算在旋轉過程中會通過幾條磁條
  */
uint8_t vehicle_pass_magnetic_stripe_calculate(
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

    return count;
}

/**
  * @brief AGV 倒退直到離開強力磁鐵感應
  */
void vehicle_over_hall_fall_back(void) {
    vehicle_motion_and_speed_control(motion_backward, setpoint_fall_back);

    uint32_t error_start = HAL_GetTick();
    while(hall_sensor_node <= hall_strong_magnet_value) {
        if (!timeout_error(error_start, &error_timeout.vehicle_over_hall_fall_back)) break;
    }

    vehicle_motion_and_speed_control(motion_forward, 0);
}

/**
  * @brief 測試空載情況下的馬達最大速度
  * 僅使用右邊測試空載轉速
  */
void vehicle_test_no_load_speed(uint16_t mile_sec) {
    if (!debug_test_no_load_speed_enable) return;

    PI_enable = 0;
    // 確定正轉
    vehicle_motion_and_speed_control(motion_forward, 0);

    uint32_t past_time = HAL_GetTick()
            ,previous_time_dif = past_time;
    motor_set_duty(&motor_left,  70);
    motor_set_duty(&motor_right, 70);

    while (
        HAL_GetTick() - past_time < mile_sec || max_speed_pcn <= 10
    ) {
        if (max_speed_pcn < motor_right.speed_present) {
                max_speed_pcn = motor_right.speed_present;
                past_time = HAL_GetTick();
                text_time = past_time;
        }

        if (!timeout_error(previous_time_dif, &error_timeout.vehicle_test_no_load_speed)) break;
    }

    motor_set_duty(&motor_left,  0);
    motor_set_duty(&motor_right, 0);
    previous_time_dif = HAL_GetTick() - previous_time_dif;

    vehicle_motion_and_speed_control(motion_backward, 0);
    vehicle_ensure_motor_stop();
    HAL_Delay(1000);
    motor_set_duty(&motor_left,  70);
    motor_set_duty(&motor_right, 70);
    past_time = HAL_GetTick();
    while(HAL_GetTick() - past_time <= previous_time_dif) {
        if (!timeout_error(past_time, &error_timeout.over_hall_fall_back_time_based)) break;
    }
    motor_set_duty(&motor_left,  0);
    motor_set_duty(&motor_right, 0);
    vehicle_ensure_motor_stop();
    vehicle_motion_and_speed_control(motion_forward, 0);
    PI_enable = 1;
}

/**
  * @brief 等待左右馬達完全停止
  */
void vehicle_ensure_motor_stop(void) {
    motor_right.speed_sepoint_pcn = 0;
    motor_left.speed_sepoint_pcn  = 0;

    uint32_t error_start = HAL_GetTick();
    while(motor_right.speed_present != 0 || motor_left.speed_present != 0) {
        if (!timeout_error(error_start, &error_timeout.vehicle_ensure_motor_stop)) break;
    }
}

/**
  * @brief 判斷旋轉方向（順時針／逆時針）
  */
ROTATE_STATUS vehicle_get_rotate_direction(int8_t start_dir, int8_t end_dir) {
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
  * @brief 根據強磁計數更新 AGV 方向資料
  */
void vehicle_renew_vehicle_rotation_status (uint8_t count_until_zero) {
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

        if (!timeout_error(time_out, &error_timeout.vehicle_renew_vehicle_rotation_status)) break;
    }
}

/**
  * @brief 在指定時間內，讓裝置順或逆旋轉，直到偵測到磁條，並停止
  */
void vehicle_search_magnetic_path (MOTIONCOMMAND search_direction, uint16_t time){
    if (!search_magnetic_path_enable) return;
    vehicle_motion_and_speed_control(search_direction, setpoint_rotate);

    uint32_t past_time = HAL_GetTick();
    while (HAL_GetTick() - past_time <= time) {
        // if hall sensor sensing magnetic force
        if (
            hall_sensor_direction >= hall_magnetic_stripe_value ||
            motor_left.adc_value  >= hall_magnetic_stripe_value ||
            motor_right.adc_value >= hall_magnetic_stripe_value
        ) {
            vehicle_motion_and_speed_control(motion_forward, 0);

            search_magnetic_path_enable = 0;
            break;
        }

        adc_renew();

        if (!timeout_error(past_time, &error_timeout.vehicle_search_magnetic_path)) break;
    }

    vehicle_ensure_motor_stop();
}

/**
  * @brief ROTATE_STATUS 轉 MOTIONCOMMAND
  */
MOTIONCOMMAND vehicle_rotate_status_to_motioncommand (ROTATE_STATUS mode) {
    if (mode == clockwise) {
        return motion_clockwise;
    } else {
        return motion_counter_clockwise;
    }

}

/**
  * @brief 偵測是否有初始方向數據，如果存在，則執行原地旋轉修正以對準起始航向
  */
void vehicle_adjust_startup_heading (void) {
    if (map_data.start_direction == no_data) return;

    ROTATE_STATUS rotate_direction_mode = vehicle_get_rotate_direction(map_data.start_direction, map_data.direction[0]);
    vehicle_motion_and_speed_control(vehicle_rotate_status_to_motioncommand(rotate_direction_mode), setpoint_rotate);

    uint8_t renew_count = vehicle_pass_magnetic_stripe_calculate(
        rotate_direction_mode,
        map_data.address_id[0],
        map_data.start_direction,
        map_data.direction[0]
        );

    vehicle_renew_vehicle_rotation_status(renew_count);
}
