#include "user/vehicle.h"
#include "user/PI_control.h"
#include "user/user_adc.h"
#include "user/user_it.h"
#include "user/const_and_error.h"
#include "tim.h"
#include <math.h>
#include "stm32g4xx_hal.h"

// 判斷是否轉灣大小
const uint32_t track_hall_critical_value = 2*16*16*16 + 16*16 + 16 + 1;
// 判斷強力磁鐵位置
const uint32_t node_hall_critical_value = 16*16*16 + 16*16 + 16 + 1;

/*測試用--------------------------------------*/
uint32_t hall_count_direction = 0;
uint32_t text_previous_time_fall_back_dif = 0;
uint32_t text_return_start_time = 0;

/*測試用--------------------------------------*/

AGV_STATUS agv_current_status = agv_straight;
VEHICLE_DATA vehicle_current_data;
MOTIONCOMMAND direction_mode;

VEHICLE_DATA vehicle_data_new(const MAP_DATA *map_data, int index) {
    VEHICLE_DATA data;
    data.direction = map_data->direction[index];
    data.address_id = map_data->address_id[index];
    return data;
};



/**
  *@brief setup
  */
void vehicle_setup(void) {
    vehicle_current_data = vehicle_data_new(&map_current_data, 0);
}

/**
  * @brief 設定馬達速度目標值，限制範圍 0~100
  * @retval true：成功，false：超出範圍並已修正
  */
bool motor_speed_setpoint_set(MOTOR_PARAMETER* motor, uint8_t value) {
    if (value > 100) {
        motor->speed_sepoint = 100;
        return false;
    }
    motor->speed_sepoint = value;
    return true;
}

/**
  * @brief 一般循跡模式控制
  */
void track_mode(void) {

    adc_renew();

    if(motor_right.adc_value >= track_hall_critical_value) {
        motor_speed_setpoint_set(&motor_left, setpoint_straight);
        motor_speed_setpoint_set(&motor_right, 0);

    } else if(motor_left.adc_value >= track_hall_critical_value) {
        motor_speed_setpoint_set(&motor_left, 0);
        motor_speed_setpoint_set(&motor_right, setpoint_straight);

    } else {
        motor_speed_setpoint_set(&motor_left, setpoint_straight);
        motor_speed_setpoint_set(&motor_right, setpoint_straight);
    }
}

/**
  * @brief AGV 原地旋轉直到對準方向
  */
void rotate_in_place(void) {
    uint32_t previous_time = HAL_GetTick();
    uint32_t error_start = HAL_GetTick();

    while (get_rotate_direction() != either){
        switch (get_rotate_direction()) {
            case clockwise:                         // 順時針旋轉的動作
                motor_motion_control(motion_clockwise);
                renew_vehicle_current_direction(1, &previous_time);
                break;

            case counter_clockwise:                 // 逆時針旋轉的動作
                motor_motion_control(motion_counter_clockwise);
                renew_vehicle_current_direction(-1, &previous_time);
                break;

            case either:                            //結束旋轉
                motor_motion_control(motion_forward);
                break;
        }

        motor_left.speed_sepoint = setpoint_rotate;
        motor_right.speed_sepoint = setpoint_rotate;
        timeout_error(error_start, &error_timeout.rotate_in_place);
    }

    error_start = HAL_GetTick();
    // 確保轉彎後能夠脫離強力磁鐵進入循跡
    while(hall_count_direction >= node_hall_critical_value ) {
        motor_left.speed_sepoint = setpoint_straight;
        motor_right.speed_sepoint = setpoint_straight;
        timeout_error(error_start, &error_timeout.rotate_in_place_hall);
    }
}



/**
  * @brief AGV 倒退直到離開強力磁鐵感應
  */
void over_hall_fall_back(void) {
    // 更改為倒退方向
    motor_motion_control(motion_backward);

    uint32_t error_start = HAL_GetTick();
    while(hall_count_direction >= node_hall_critical_value) {
        motor_left.speed_sepoint = setpoint_fall_back;
        motor_right.speed_sepoint = setpoint_fall_back;
        timeout_error(error_start, &error_timeout.over_hall_fall_back);
    }

    // 更改為前進方向
    motor_motion_control(motion_forward);
}



/**
  * @brief 判斷旋轉方向（順時針／逆時針）
  */
ROTATE_STATUS get_rotate_direction(void) {
    int diff = (map_current_data.direction[map_current_data.current_count] - vehicle_current_data.direction + 8) % 8;

    if (diff == 0) {
        return either;                               // 旋轉完成

    } else if (diff <= 3) {
        return clockwise;

    } else {
        return counter_clockwise;

    }
}

/**
  * @brief 根據運動模式控制馬達旋轉方向
  */
void motor_motion_control(MOTIONCOMMAND mode){
    switch(mode) {
        case motion_forward:
            veh_direction(&motor_right, clockwise);
            veh_direction(&motor_left,  counter_clockwise);
            break;

        case motion_backward:
            veh_direction(&motor_right, counter_clockwise);
            veh_direction(&motor_left,  clockwise);
            break;

        case motion_clockwise:
            veh_direction(&motor_right, counter_clockwise);
            veh_direction(&motor_left,  counter_clockwise);
            break;

        case motion_counter_clockwise:
            veh_direction(&motor_right, clockwise);
            veh_direction(&motor_left,  clockwise);
            break;

    }
}

void veh_direction(MOTOR_PARAMETER *motor, ROTATE_STATUS direction){
    motor->rotate_direction = direction;
}

/**
  * @brief 根據強磁計數更新 AGV 方向資料
  */
void renew_vehicle_current_direction (int renew_direction, uint32_t *previous_time) {
    if (hall_count_direction >= node_hall_critical_value && HAL_GetTick() - *previous_time >= 500) {
        *previous_time = HAL_GetTick();          //renew time 為了不立刻重讀
        vehicle_current_data.direction += renew_direction;
    }
}

/**
  * @brief 等待左右馬達完全停止
  */
void ensure_motor_stop(void) {
    motor_right.speed_sepoint = 0;
    motor_left.speed_sepoint  = 0;

    uint32_t error_start = HAL_GetTick();
    while(motor_right.speed_present != 0 || motor_left.speed_present != 0) {
        timeout_error(error_start, &error_timeout.ensure_motor_stop);
    }
}

/**
  * @brief 測試空載情況下的馬達最大速度
  * 僅使用右邊測試空載轉速
  */
 uint32_t text_previous_time_fall_back_dif;
void test_no_load_speed(uint16_t mile_sec) {
    // 確定正轉
    motor_motion_control(motion_forward);

    uint32_t previous_time = HAL_GetTick()
            ,previous_time_dif = previous_time;
    set_motor_duty(&motor_left,  100);
    set_motor_duty(&motor_right, 100);

    while (
        HAL_GetTick() - previous_time < mile_sec ||
        max_speed <= 10
    ) {
        if(max_speed < motor_right.speed_present) {
                max_speed = motor_right.speed_present;
                previous_time = HAL_GetTick();
        }

        timeout_error(previous_time_dif, &error_timeout.test_no_load_speed);
    }

    set_motor_duty(&motor_left,  0);
    set_motor_duty(&motor_right, 0);
    previous_time_dif = HAL_GetTick() - previous_time_dif;

    motor_motion_control(motion_backward);
    ensure_motor_stop();
    previous_time = HAL_GetTick();
    set_motor_duty(&motor_left,  100);
    set_motor_duty(&motor_right, 100);
    while(HAL_GetTick() - previous_time <= previous_time_dif) {
        timeout_error(previous_time, &error_timeout.over_hall_fall_back_time_based);
    }
    set_motor_duty(&motor_left,  0);
    set_motor_duty(&motor_right, 0);
    ensure_motor_stop();
    motor_motion_control(motion_forward);
}
