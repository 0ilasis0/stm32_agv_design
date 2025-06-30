#include "main/vehicle.h"
#include <math.h>
#include "tim.h"
#include "stm32g4xx_hal.h"
#include "main/adc.h"
#include "main/it.h"
#include "main/fn_state.h"
#include "main/config.h"

int text_end = 0;
void vehicle_main (void)
{
    if (hall_sensor_node > hall_strong_magnet_value) {
        decide_move_mode();

    } else {
        if (map_data.status[map_data.current_count] == agv_next) {
            map_data.current_count++ ;

        } else {
            vehicle_track_mode();

        }
    }
}

/**
  * @brief 決定移動MODE
  */
void decide_move_mode(void)
{
    switch(map_data.status[map_data.current_count])
    {
        case agv_straight:
            motor_set_speed_setpoint(&motor_right, VEHICLE_setpoint_straight);
            motor_set_speed_setpoint (&motor_left, VEHICLE_setpoint_straight);

            // 改為agv_next，直到離開HALL，使else之後能renew status
            map_data.status[map_data.current_count] = agv_next;
            break;

        case agv_rotate:
            // protect_over_hall();
            vehicle_rotate_in_place();

            // 改為agv_next，直到離開HALL，使else之後能renew status
            map_data.status[map_data.current_count] = agv_next;
            break;

        case agv_end:
            // protect_over_hall();
            init_map_data_direction_and_address(&map_data, map_data.address_id[map_data.current_count - 1], map_data.direction[map_data.current_count - 1]);
            // 終止目前沒有要做甚麼所以先停止動作
            while (1) {
                motor_set_speed_setpoint(&motor_right, 0);
                motor_set_speed_setpoint (&motor_left, 0);
                text_end = 1;
            }
            break;
        default:
            break;
    }
}

/**
  * @brief 一般循跡模式控制
  */
void vehicle_track_mode(void) {
    adc_renew();

    vehicle_breakdown_all_hall_lost ();

    if (motor_right.adc_value >= hall_magnetic_stripe_value) {
        motor_set_speed_setpoint(&motor_left, VEHICLE_setpoint_straight);
        motor_set_speed_setpoint(&motor_right, 0);

    } else if (motor_left.adc_value >= hall_magnetic_stripe_value) {
        motor_set_speed_setpoint(&motor_left, 0);
        motor_set_speed_setpoint(&motor_right, VEHICLE_setpoint_straight);

    } else {
        motor_set_speed_setpoint(&motor_left, VEHICLE_setpoint_straight);
        motor_set_speed_setpoint(&motor_right, VEHICLE_setpoint_straight);
    }
}

/**
  * @brief AGV 原地旋轉直到對準方向
  */
void vehicle_rotate_in_place(void)
{
    if (map_data.current_count == 0) error_state.rotate_in_place__map_data_current_count = FNS_NO_MATCH;

    MotionCommand rotate_direction_mode = vehicle2_get_rotate_direction(map_data.direction[map_data.current_count - 1], map_data.direction[map_data.current_count]);

    uint8_t renew_count = vehicle2_pass_magnetic_stripe_calculate(
            rotate_direction_mode,
            map_data.address_id[map_data.current_count],
            map_data.direction[map_data.current_count - 1],
            map_data.direction[map_data.current_count]
            );

    vehicle2_motion_and_speed_control(rotate_direction_mode, VEHICLE_setpoint_rotate);

    vehicle2_renew_vehicle_rotation_status(renew_count);


    vehicle2_motion_and_speed_control(motion_forward, VEHICLE_setpoint_straight);
    uint32_t error_start = HAL_GetTick();
    // 確保轉彎後能夠脫離強力磁鐵進入循跡
    while(hall_sensor_node >= hall_strong_magnet_value ) {
        timeout_error(error_start, &error_state.vehicle_rotate_in_place_hall);
    }
}

/**
  * @brief AGV 倒退直到離開強力磁鐵感應
  */
void vehicle_over_hall_fall_back(void) {
    vehicle2_motion_and_speed_control(motion_backward, VEHICLE_setpoint_fall_back);

    uint32_t error_start = HAL_GetTick();
    while(hall_sensor_node <= hall_strong_magnet_value) {
        timeout_error(error_start, &error_state.vehicle_over_hall_fall_back);
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
        vehicle_search_magnetic_path (motion_c_clockwise, 6000);
        if (sys_run_switch.enable_search_magnetic_path == 1)
        {
            while (true) error_state.breakdown_all_hall_lost__path_not_found = FNS_NOT_MOVE;
        }

        sys_run_switch.enable_search_magnetic_path = 1;
    }
}

/**
  * @brief 在指定時間內，讓裝置順或逆旋轉，直到偵測到磁條，並停止
  */
void vehicle_search_magnetic_path (MotionCommand search_direction, uint16_t time){
    if (!sys_run_switch.enable_search_magnetic_path) return;
    vehicle2_motion_and_speed_control(search_direction, VEHICLE_setpoint_rotate);

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

        timeout_error(past_time, &error_state.vehicle_search_magnetic_path);
    }

    vehicle2_ensure_motor_stop();
}

/**
  * @brief 偵測是否有初始方向數據，如果存在，則執行原地旋轉修正以對準起始航向
  */
void vehicle_adjust_startup_heading (void) {
    if (map_data.start_address_id == no_data) return;

    MotionCommand rotate_direction_mode = vehicle2_get_rotate_direction(map_data.start_direction, map_data.direction[0]);
    vehicle2_motion_and_speed_control(rotate_direction_mode, VEHICLE_setpoint_rotate);

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
        }

        timeout_error(previous_time_dif, &error_state.vehicle_test_no_load_speed);
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
        timeout_error(past_time, &error_state.vehicle_test_no_load_speed);
    }
    motor_set_duty(&motor_left,  0);
    motor_set_duty(&motor_right, 0);
    vehicle2_ensure_motor_stop();
    vehicle2_motion_and_speed_control(motion_forward, 0);
    sys_run_switch.enable_PI = 1;
}

/* 保護未完成動作卻已超出hall範圍 -------------------------------------*/
void protect_over_hall(void)
{
    vehicle2_ensure_motor_stop();

    if (hall_sensor_node > hall_strong_magnet_value) return;

    //防止 原地旋轉前 衝過hall_sensor速度仍未停止，後退並強制進入原地旋轉
    if (map_data.status[map_data.current_count] == agv_rotate)
    {
        vehicle_over_hall_fall_back();
    }

    //防止 結束後 衝過hall_sensor 速度仍未停止，進行後退
    if (map_data.status[map_data.current_count] == agv_end)
    {
        vehicle_over_hall_fall_back();
    }
}

