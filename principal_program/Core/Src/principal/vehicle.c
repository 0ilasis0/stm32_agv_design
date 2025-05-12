#include "principal/vehicle.h"
#include "principal/PI_control.h"
#include "principal/principal_adc.h"

// 判斷是否轉灣大小
const uint32_t track_hall_critical_value = 16*16*16 + 16*16 + 16 + 1;
// 判斷強力磁鐵位置
const uint32_t node_hall_critical_value = 16*16*16 + 16*16 + 16 + 1;

uint32_t previous_time = 0;

/*測試用--------------------------------------*/
uint32_t hall_count_direction = 0;
/*測試用--------------------------------------*/

AGV_STATUS agv_current_status = agv_straight;
VEHICLE_DATA vehicle_current_data;

//MAP_DATA map_data = {0};//
MAP_DATA map_current_data = {     //到時候由最段路徑演算法輸入入徑
    0,                                                         //current_count
    {agv_straight, agv_straight, agv_rotate, agv_straight, agv_end},    //status
    {1, 1, 3, 3},                                              //direction
    {0, 1, 2, 3}                                               //address_id
};

VEHICLE_DATA inti_vehicle_data(const MAP_DATA *map_data, int index) {
    VEHICLE_DATA data;
    data.status = map_data->status[index];
    data.direction = map_data->direction[index];
    data.address_id = map_data->address_id[index];
    return data;
};



/* setup -----------------------------------------------------------*/
void vehicle_setup(void) {
    vehicle_current_data = inti_vehicle_data(&map_current_data, 0);
    vehicle_current_data.rotate_direction = either;
}



/* AGV更新馬達驅動 --------------------------------------------------*/
void renew_motor_drive(int sepoint) {
    setpoint_current = sepoint;

    pwm_limit();                // 限制 PWM 占空比範圍

    motor_right.pwmValue = motor_right.pwmValue_temp;
    motor_left.pwmValue = motor_left.pwmValue_temp;

    commutate_motor(&motor_right);
    commutate_motor(&motor_left);
}



/* 限制PWM最大最小值 -------------------------------------------------*/
void pwm_limit(void) {
    if (motor_right.pwmValue_temp > max_duty) motor_right.pwmValue_temp = max_duty;
    if (motor_right.pwmValue_temp < min_duty) motor_right.pwmValue_temp = min_duty;
    if (motor_left.pwmValue_temp  > max_duty) motor_left.pwmValue_temp  = max_duty;
    if (motor_left.pwmValue_temp  < min_duty) motor_left.pwmValue_temp  = min_duty;
}



/* AGV一般循跡功能 --------------------------------------------------*/
void track_mode(void) {

    adc_renew();

    if(motor_right.adc_value >= track_hall_critical_value) {
        motor_left.pwmValue = motor_left.pwmValue_temp;
        motor_right.pwmValue = min_duty;

    } else if(motor_left.adc_value >= track_hall_critical_value) {
        motor_right.pwmValue = motor_right.pwmValue_temp;
        motor_left.pwmValue = min_duty;

    } else {
        motor_right.pwmValue = motor_right.pwmValue_temp;
        motor_left.pwmValue = motor_left.pwmValue_temp;

    }

    renew_motor_drive(setpoint_straight);
}



/* AGV原地旋轉功能 --------------------------------------------------*/
void rotate_in_place(void) {
    previous_time = HAL_GetTick();

    while (get_rotate_direction() != either){
        switch (get_rotate_direction()) {
            case clockwise:                         // 順時針旋轉的動作
                rotate_control_direction(counter_clockwise, counter_clockwise);
                renew_vehicle_current_direction(1);
                break;

            case counter_clockwise:                 // 逆時針旋轉的動作
                rotate_control_direction(clockwise, clockwise);
                renew_vehicle_current_direction(-1);
                break;

            case either:                            //結束旋轉
                rotate_control_direction(counter_clockwise, clockwise);
                break;
        }

        renew_motor_drive(setpoint_rotate);
    }

    // 確保轉彎後能夠脫離強力磁鐵進入循跡
    while(hall_count_direction >= node_hall_critical_value ) {
        renew_motor_drive(setpoint_straight);
    }
}



/* AGV倒退 ----------------------------------------------------------*/
void over_hall_fall_back(void) {
    // 更改為倒退方向
    rotate_control_direction(clockwise, counter_clockwise);

    while(hall_count_direction >= node_hall_critical_value ) {
        renew_motor_drive(setpoint_fall_back);
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

    motor_right.pwmValue = motor_right.pwmValue_temp;
    motor_left.pwmValue = motor_left.pwmValue_temp;
}



/* 更新車子方向資料 ---------------------------------------------------*/
void renew_vehicle_current_direction (int renew_direction) {
    if (hall_count_direction == node_hall_critical_value && HAL_GetTick() - previous_time >= 500) {
        previous_time = HAL_GetTick();          //renew time 為了不立刻重讀
        vehicle_current_data.direction += renew_direction;
    }
}
