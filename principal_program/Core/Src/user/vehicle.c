#include "user/vehicle.h"
#include "user/PI_control.h"
#include "user/user_adc.h"
#include "user/user_it.h"
#include "user/const_and_error.h"
#include "tim.h"
#include <math.h>
#include "stm32g4xx_hal.h"

// 判斷磁條強度大小
const uint32_t hall_magnetic_stripe_value = 2*16*16*16 + 16*16 + 16 + 1;
// 判斷強力磁鐵強度大小
const uint32_t hall_strong_magnet_value = 16*16*16 + 16*16 + 16 + 1;

/*測試用--------------------------------------*/
uint32_t hall_sensor_direction = 0;
uint32_t text_previous_time_fall_back_dif = 0;
uint32_t text_return_start_time = 0;
uint32_t text_time = 0;
/*測試用--------------------------------------*/

AGV_STATUS agv_current_status = agv_straight;
MOTIONCOMMAND direction_mode;



/**
  * @brief 一般循跡模式控制
  */
void track_mode(void) {
    adc_renew();

    breakdown_all_hall_lost ();

    if (motor_right.adc_value >= hall_magnetic_stripe_value) {
        motor_speed_setpoint_set(&motor_left, setpoint_straight);
        motor_speed_setpoint_set(&motor_right, 0);

    } else if(motor_left.adc_value >= hall_magnetic_stripe_value) {
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
    if (map_data.current_count == 0) error_data.rotate_in_place__map_data_current_count = 1;
    ROTATE_STATUS rotate_direction_mode = get_rotate_direction();

    switch (rotate_direction_mode) {
        case clockwise:                         // 順時針旋轉的動作
            motor_motion_control(motion_clockwise);
            renew_vehicle_rotation_status(pass_magnetic_stripe_calculate(clockwise));
            break;

        case counter_clockwise:                 // 逆時針旋轉的動作
            motor_motion_control(motion_counter_clockwise);
            renew_vehicle_rotation_status(pass_magnetic_stripe_calculate(counter_clockwise));
            break;

        case either:                            //結束旋轉
            motor_motion_control(motion_forward);
            break;
    }

    uint32_t error_start = HAL_GetTick();
    // 確保轉彎後能夠脫離強力磁鐵進入循跡
    while(hall_sensor_node >= hall_strong_magnet_value ) {
        motor_left.speed_sepoint = setpoint_straight;
        motor_right.speed_sepoint = setpoint_straight;
        if (!timeout_error(error_start, &error_timeout.rotate_in_place_hall)) break;
    }
}

/**
  * @brief AGV 倒退直到離開強力磁鐵感應
  */
void over_hall_fall_back(void) {
    motor_motion_control(motion_backward);

    motor_left.speed_sepoint = setpoint_fall_back;
    motor_right.speed_sepoint = setpoint_fall_back;

    uint32_t error_start = HAL_GetTick();
    while(hall_sensor_node <= hall_strong_magnet_value) {
        if (!timeout_error(error_start, &error_timeout.over_hall_fall_back)) break;
    }

    motor_motion_control(motion_forward);
}

/**
  * @brief 測試空載情況下的馬達最大速度
  * 僅使用右邊測試空載轉速
  */
uint32_t text_previous_time_fall_back_dif;
void test_no_load_speed(uint16_t mile_sec) {
    PI_enable = 0;
    // 確定正轉
    motor_motion_control(motion_backward);

    uint32_t past_time = HAL_GetTick()
            ,previous_time_dif = past_time;
    set_motor_duty(&motor_left,  100);
    set_motor_duty(&motor_right, 100);

    while (
        HAL_GetTick() - past_time < mile_sec ||
        max_speed <= 10
    ) {
        if(max_speed < motor_right.speed_present) {
                max_speed = motor_right.speed_present;
                past_time = HAL_GetTick();
                text_time = past_time;
        }

        if (!timeout_error(previous_time_dif, &error_timeout.test_no_load_speed)) break;
    }

    set_motor_duty(&motor_left,  0);
    set_motor_duty(&motor_right, 0);
    previous_time_dif = HAL_GetTick() - previous_time_dif;

    motor_motion_control(motion_forward);
    ensure_motor_stop();
    set_motor_duty(&motor_left,  100);
    set_motor_duty(&motor_right, 100);
    past_time = HAL_GetTick();
    while(HAL_GetTick() - past_time <= previous_time_dif) {
        if (!timeout_error(past_time, &error_timeout.over_hall_fall_back_time_based)) break;
    }
    set_motor_duty(&motor_left,  0);
    set_motor_duty(&motor_right, 0);
    ensure_motor_stop();
    motor_motion_control(motion_forward);
    PI_enable = 1;
}

/**
  * @brief 等待左右馬達完全停止
  */
void ensure_motor_stop(void) {
    motor_right.speed_sepoint = 0;
    motor_left.speed_sepoint  = 0;

    uint32_t error_start = HAL_GetTick();
    while(motor_right.speed_present != 0 || motor_left.speed_present != 0) {
        if (!timeout_error(error_start, &error_timeout.ensure_motor_stop)) break;
    }
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
  * @brief 判斷旋轉方向（順時針／逆時針）
  */
ROTATE_STATUS get_rotate_direction(void) {
    int8_t diff = (map_data.direction[map_data.current_count] - map_data.direction[map_data.current_count - 1] + 8) % 8;

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

/**
  * @brief 設定馬達旋轉方向（rotate_direction）
  */
void veh_direction(MOTOR_PARAMETER *motor, ROTATE_STATUS direction){
    motor->rotate_direction = direction;
}

/**
  * @brief 根據旋轉方向，計算在旋轉過程中會通過幾條磁條
  */
uint8_t pass_magnetic_stripe_calculate(ROTATE_STATUS rotate_direction_mode) {
    uint8_t count = 0;

    // 取得目前節點（node）在 locations_t 中的索引值
    int current_id = get_index_by_id(map_data.address_id[map_data.current_count]);
    int from_dir = map_data.direction[map_data.current_count - 1];
    int to_dir   = map_data.direction[map_data.current_count];

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
  * @brief 根據強磁計數更新 AGV 方向資料
  */
void renew_vehicle_rotation_status (uint8_t count_until_zero) {
    motor_left.speed_sepoint = setpoint_rotate;
    motor_right.speed_sepoint = setpoint_rotate;

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

        if (!timeout_error(time_out, &error_timeout.renew_vehicle_rotation_status)) break;
    }
}

void breakdown_all_hall_lost (void) {
    if (!debug_breakdown_all_hall_lost) return;

    if (hall_sensor_direction < hall_magnetic_stripe_value &&
        hall_sensor_node      < hall_magnetic_stripe_value &&
        motor_right.adc_value < hall_magnetic_stripe_value &&
        motor_left.adc_value  < hall_magnetic_stripe_value
        ) {
        ensure_motor_stop();
        search_magnetic_path (motion_clockwise, 2000);
        search_magnetic_path (motion_counter_clockwise, 4000);
        search_magnetic_path_enable = 1;
    }
}

void search_magnetic_path (MOTIONCOMMAND search_direction, uint16_t time){
    if (!search_magnetic_path_enable) return;

    motor_motion_control(search_direction);
    motor_left.speed_sepoint  = setpoint_rotate;
    motor_right.speed_sepoint = setpoint_rotate;

    uint32_t past_time = HAL_GetTick();
    while (HAL_GetTick() - past_time <= time) {
        // if hall sensor sensing magnetic force
        if (hall_sensor_direction >= hall_magnetic_stripe_value) {
            ensure_motor_stop();

            motor_motion_control(motion_forward);
            motor_left.speed_sepoint  = setpoint_straight;
            motor_right.speed_sepoint = setpoint_straight;

            while (motor_left.adc_value  <= hall_magnetic_stripe_value &&
                   motor_right.adc_value <= hall_magnetic_stripe_value
                    ) {
                if (!timeout_error(past_time, &error_timeout.search_magnetic_path_in)) break;
            }
            search_magnetic_path_enable = 0;
            break;
        }

        if (!timeout_error(past_time, &error_timeout.search_magnetic_path)) break;
    }

    ensure_motor_stop();
}
