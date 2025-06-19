#include "main/vehicle.h"
#include <math.h>
#include "tim.h"
#include "stm32g4xx_hal.h"
#include "main/adc.h"
#include "main/it.h"
#include "main/const_and_error.h"
#include "motor/PI_control.h"

/*測試用--------------------------------------*/
uint32_t text_return_start_time = 0;
uint32_t text_time = 0;
/*測試用--------------------------------------*/

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
  * @brief AGV 原地旋轉直到對準方向
  */
ROTATE_STATUS rotate_direction_mode;
void vehicle_rotate_in_place(void) {
    if (map_data.current_count == 0) error_state.rotate_in_place__map_data_current_count = state_data_no_match;

    rotate_direction_mode = vehicle2_get_rotate_direction(map_data.direction[map_data.current_count - 1], map_data.direction[map_data.current_count]);

    if (rotate_direction_mode != either) {
        uint8_t renew_count = vehicle2_pass_magnetic_stripe_calculate(
                rotate_direction_mode,
                map_data.address_id[map_data.current_count],
                map_data.direction[map_data.current_count - 1],
                map_data.direction[map_data.current_count]
                );

        vehicle2_motion_and_speed_control(vehicle2_rotate_status_to_motioncommand(rotate_direction_mode), setpoint_rotate);

        vehicle2_renew_vehicle_rotation_status(renew_count);
    }

    vehicle2_motion_and_speed_control(motion_forward, setpoint_straight);
    uint32_t error_start = HAL_GetTick();
    // 確保轉彎後能夠脫離強力磁鐵進入循跡
    while(hall_sensor_node >= hall_strong_magnet_value ) {
        if (!timeout_error(error_start, &error_state.vehicle_rotate_in_place_hall)) break;
    }
}

/**
  * @brief AGV 倒退直到離開強力磁鐵感應
  */
void vehicle_over_hall_fall_back(void) {
    vehicle2_motion_and_speed_control(motion_backward, setpoint_fall_back);

    uint32_t error_start = HAL_GetTick();
    while(hall_sensor_node <= hall_strong_magnet_value) {
        if (!timeout_error(error_start, &error_state.vehicle_over_hall_fall_back)) break;
    }

    vehicle2_motion_and_speed_control(motion_forward, 0);
}

/**
  * @brief 當所有相關的霍爾感測器都失去磁條訊號時，嘗試重新搜尋並回到磁條路徑上
  */
void vehicle_breakdown_all_hall_lost (void) {
    if (!sys_run_switch.enable_debug_breakdown_all_hall_lost) return;

    if (
        hall_sensor_direction < hall_magnetic_stripe_value &&
        hall_sensor_node      < hall_magnetic_stripe_value &&
        motor_right.adc_value < hall_magnetic_stripe_value &&
        motor_left.adc_value  < hall_magnetic_stripe_value
        ) {
            vehicle2_ensure_motor_stop();
            vehicle_search_magnetic_path (motion_clockwise, 3000);
            vehicle_search_magnetic_path (motion_counter_clockwise, 6000);
            if (sys_run_switch.enable_search_magnetic_path == 1) {
                while (true) error_state.breakdown_all_hall_lost__path_not_found = state_stop_move;
            }

            sys_run_switch.enable_search_magnetic_path = 1;
    }
}

/**
  * @brief 在指定時間內，讓裝置順或逆旋轉，直到偵測到磁條，並停止
  */
void vehicle_search_magnetic_path (MOTIONCOMMAND search_direction, uint16_t time){
    if (!sys_run_switch.enable_search_magnetic_path) return;
    vehicle2_motion_and_speed_control(search_direction, setpoint_rotate);

    uint32_t past_time = HAL_GetTick();
    while (HAL_GetTick() - past_time <= time) {
        // if hall sensor sensing magnetic force
        if (
            hall_sensor_direction >= hall_magnetic_stripe_value ||
            motor_left.adc_value  >= hall_magnetic_stripe_value ||
            motor_right.adc_value >= hall_magnetic_stripe_value
        ) {
            vehicle2_motion_and_speed_control(motion_forward, 0);

            sys_run_switch.enable_search_magnetic_path = 0;
            break;
        }

        adc_renew();

        if (!timeout_error(past_time, &error_state.vehicle_search_magnetic_path)) break;
    }

    vehicle2_ensure_motor_stop();
}

/**
  * @brief 偵測是否有初始方向數據，如果存在，則執行原地旋轉修正以對準起始航向
  */
void vehicle_adjust_startup_heading (void) {
    if (map_data.start_address_id == no_data) return;

    ROTATE_STATUS rotate_direction_mode = vehicle2_get_rotate_direction(map_data.start_direction, map_data.direction[0]);
    vehicle2_motion_and_speed_control(vehicle2_rotate_status_to_motioncommand(rotate_direction_mode), setpoint_rotate);

    uint8_t renew_count = vehicle2_pass_magnetic_stripe_calculate(
        rotate_direction_mode,
        map_data.address_id[0],
        map_data.start_direction,
        map_data.direction[0]
        );

    vehicle2_renew_vehicle_rotation_status(renew_count);
}

/**
  * @brief 測試空載情況下的馬達最大速度
  * 僅使用右邊測試空載轉速
  */
void vehicle_test_no_load_speed(uint16_t mile_sec) {
    if (!sys_run_switch.enable_debug_test_no_load_speed) return;

    sys_run_switch.enable_PI = 0;
    // 確定正轉
    vehicle2_motion_and_speed_control(motion_forward, 0);

    uint32_t past_time = HAL_GetTick()
            ,previous_time_dif = past_time;
    motor_set_duty(&motor_left,  70);
    motor_set_duty(&motor_right, 70);

    while (
        HAL_GetTick() - past_time < mile_sec || max_speed <= 10
    ) {
        if (max_speed < motor_right.speed_present) {
                max_speed = motor_right.speed_present;
                past_time = HAL_GetTick();
                text_time = past_time;
        }

        if (!timeout_error(previous_time_dif, &error_state.vehicle_test_no_load_speed)) break;
    }

    motor_set_duty(&motor_left,  0);
    motor_set_duty(&motor_right, 0);
    previous_time_dif = HAL_GetTick() - previous_time_dif;

    vehicle2_motion_and_speed_control(motion_backward, 0);
    vehicle2_ensure_motor_stop();
    HAL_Delay(1000);
    motor_set_duty(&motor_left,  70);
    motor_set_duty(&motor_right, 70);
    past_time = HAL_GetTick();
    while(HAL_GetTick() - past_time <= previous_time_dif) {
        if (!timeout_error(past_time, &error_state.vehicle_test_no_load_speed)) break;
    }
    motor_set_duty(&motor_left,  0);
    motor_set_duty(&motor_right, 0);
    vehicle2_ensure_motor_stop();
    vehicle2_motion_and_speed_control(motion_forward, 0);
    sys_run_switch.enable_PI = 1;
}
