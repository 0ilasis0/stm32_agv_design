#include "user/vehicle.h"
#include "user/PI_control.h"
#include "user/user_adc.h"
#include "user/user_it.h"
#include "user/const_and_error.h"
#include "tim.h"
#include <math.h>
#include "stm32g4xx_hal.h"

// 判斷是否轉灣大小
const uint32_t track_hall_critical_value = 16*16*16 + 16*16 + 16 + 1;
// 判斷強力磁鐵位置
const uint32_t node_hall_critical_value = 16*16*16 + 16*16 + 16 + 1;

/*測試用--------------------------------------*/
uint32_t hall_count_direction = 0;
uint32_t text_previous_time_fall_back_dif = 0;
uint32_t text_return_start_time = 0;

/*測試用--------------------------------------*/

AGV_STATUS agv_current_status = agv_straight;
VEHICLE_DATA vehicle_current_data;

VEHICLE_DATA vehicle_data_new(const MAP_DATA *map_data, int index) {
    VEHICLE_DATA data;
    data.direction = map_data->direction[index];
    data.address_id = map_data->address_id[index];
    return data;
};



/* setup -----------------------------------------------------------*/
void vehicle_setup(void) {
    vehicle_current_data = vehicle_data_new(&map_current_data, 0);
}

/* AGV一般循跡功能 --------------------------------------------------*/
void track_mode(void) {

    adc_renew();

    if(motor_right.adc_value >= track_hall_critical_value) {
        motor_left.speed_sepoint = setpoint_straight;
        motor_right.speed_sepoint = 0;

    } else if(motor_left.adc_value >= track_hall_critical_value) {
        motor_left.speed_sepoint = 0;
        motor_right.speed_sepoint = setpoint_straight;

    } else {
        motor_left.speed_sepoint = setpoint_straight;
        motor_right.speed_sepoint = setpoint_straight;

    }
}



/* AGV原地旋轉功能 --------------------------------------------------*/
void rotate_in_place(void) {
    uint32_t previous_time = HAL_GetTick();
    uint32_t error_start = HAL_GetTick();

    while (get_rotate_direction() != either){
        switch (get_rotate_direction()) {
            case clockwise:                         // 順時針旋轉的動作
                rotate_control_direction(counter_clockwise, counter_clockwise);
                renew_vehicle_current_direction(1, &previous_time);
                break;

            case counter_clockwise:                 // 逆時針旋轉的動作
                rotate_control_direction(clockwise, clockwise);
                renew_vehicle_current_direction(-1, &previous_time);
                break;

            case either:                            //結束旋轉
                rotate_control_direction(counter_clockwise, clockwise);
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



/* AGV倒退 ----------------------------------------------------------*/
void over_hall_fall_back(void) {
    // 更改為倒退方向
    rotate_control_direction(clockwise, counter_clockwise);

    uint32_t error_start = HAL_GetTick();
    while(hall_count_direction >= node_hall_critical_value) {
        motor_left.speed_sepoint = setpoint_fall_back;
        motor_right.speed_sepoint = setpoint_fall_back;
        timeout_error(error_start, &error_timeout.over_hall_fall_back);
    }

    // 更改為前進方向
    rotate_control_direction(counter_clockwise, clockwise);
}



/* 判斷順逆旋轉 ------------------------------------------------------*/
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



/* 更新轉速與決定馬達順逆時針轉 ----------------------------------------*/
void rotate_control_direction (ROTATE_STATUS left_motor_rotate, ROTATE_STATUS right_motor_rotate) {
    motor_right.rotate_direction = right_motor_rotate;
    motor_left.rotate_direction = left_motor_rotate;
}



/* 更新車子方向資料 ---------------------------------------------------*/
void renew_vehicle_current_direction (int renew_direction, uint32_t *previous_time) {
    if (hall_count_direction >= node_hall_critical_value && HAL_GetTick() - *previous_time >= 500) {
        *previous_time = HAL_GetTick();          //renew time 為了不立刻重讀
        vehicle_current_data.direction += renew_direction;
    }
}



/* 直到左右馬達停止才下個動作 -----------------------------------------*/
void ensure_motor_stop(void) {
    motor_right.speed_sepoint = 0;
    motor_left.speed_sepoint  = 0;

    uint32_t error_start = HAL_GetTick();
    while(motor_right.present_speed != 0 || motor_left.present_speed != 0) {
        speed_calculate(&motor_right);
        speed_calculate(&motor_left );
        timeout_error(error_start, &error_timeout.ensure_motor_stop);
    }
}

/**
  * @brief 測試空載速度
  */
 uint32_t text_previous_time_fall_back_dif;
void test_no_load_speed(void) {
    PI_enable = 0;

    bool next = 0;

    uint32_t previous_time_it = HAL_GetTick()
            ,previous_time_fall_back_dif = previous_time_it;

    // 確定正轉
    rotate_control_direction(counter_clockwise, clockwise);

    set_motor_duty(&motor_left,  100);
    set_motor_duty(&motor_right, 100);

    // 因為會用到測速
    uint16_t time = dt * 1000;

    // 僅使用右邊測試空載轉速
    while (fabs(motor_right.present_speed - max_speed) > 1 || HAL_GetTick() - previous_time_fall_back_dif <= time) {
        timeout_error(previous_time_fall_back_dif, &error_timeout.test_no_load_speed);

        if(HAL_GetTick() - previous_time_it > time && next == 0) {
            //更新motor當前速度
            speed_calculate(&motor_right);
            speed_calculate(&motor_left);
            previous_time_it = HAL_GetTick();
            next = 1;

        } else if(HAL_GetTick() - previous_time_it > time && next == 1) {
            if(max_speed < motor_right.present_speed) {
                max_speed = motor_right.present_speed;
            }
            speed_calculate(&motor_right);
            speed_calculate(&motor_left);
            previous_time_it = HAL_GetTick();
            next = 0;

        }
    }

    previous_time_fall_back_dif = HAL_GetTick() - previous_time_fall_back_dif;
    text_previous_time_fall_back_dif = previous_time_fall_back_dif;
    set_motor_duty(&motor_left,  0);
    set_motor_duty(&motor_right, 0);


    ensure_motor_stop();

    HAL_Delay(1000);

    over_hall_fall_back_time_based(previous_time_fall_back_dif);
    ensure_motor_stop();

    PI_enable = 1;
}

/**
  * @brief 倒車回到原位
  */
void over_hall_fall_back_time_based(uint32_t  previous_time_fall_back_dif) {
    rotate_control_direction(clockwise, counter_clockwise);

    uint32_t return_start_time = HAL_GetTick();
    text_previous_time_fall_back_dif = previous_time_fall_back_dif;
    text_return_start_time = return_start_time;
    set_motor_duty(&motor_left,  100);
    set_motor_duty(&motor_right, 100);

    while(HAL_GetTick() - return_start_time <= previous_time_fall_back_dif) {
        timeout_error(return_start_time, &error_timeout.over_hall_fall_back_time_based);
    }

    set_motor_duty(&motor_left,  0);
    set_motor_duty(&motor_right, 0);

    rotate_control_direction(counter_clockwise, clockwise);
}
