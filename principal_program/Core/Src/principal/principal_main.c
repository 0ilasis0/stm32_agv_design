#include "principal/principal_main.h"
#include "principal/motor.h"
#include "principal/vehicle.h"
#include "principal/PI_control.h"
#include "principal/principal_adc.h"
#include "principal/principal_uart.h"

<<<<<<< HEAD
/*測試用--------------------------------------*/
=======
>>>>>>> c07caf725bb0cf636f62ad409c4eeb6e91d34c0a
uint32_t hall_sensor3 = 16*16*16 + 16*16 + 16 + 1 +1;
/*測試用--------------------------------------*/

/* +Main ------------------------------------------------------------*/
void principal_main(void) {
    motor_tim_setup(&motor_right);
    motor_tim_setup(&motor_left);
    hall_detection_adc_setup();
    PI_tim_setup();
    vehicle_setup();
    uart_setup();

    update_motor_step(&motor_right);
    update_motor_step(&motor_left);
<<<<<<< HEAD

    while (1) {
        if (hall_sensor3 > node_hall_critical_value) {
=======
    while (1) {
        commutate_motor(&motor_right);
        commutate_motor(&motor_left);

        if(hall_sensor3 > node_hall_critical_value) {
>>>>>>> c07caf725bb0cf636f62ad409c4eeb6e91d34c0a
            decide_move_mode();

        } else {
            protect_over_hall();

            if (vehicle_current_data.status == agv_next) {
                map_current_data.current_count++ ;
                vehicle_current_data.status = map_current_data.status[map_current_data.current_count];
                vehicle_current_data.address_id = map_current_data.address_id[map_current_data.current_count];

            } else {
                track_mode();

            }
        }
    }
}

/* 決定移動MODE ------------------------------------------------------*/
void decide_move_mode(void) {
    // renew current data to next node
    vehicle_current_data.status = map_current_data.status[map_current_data.current_count];

<<<<<<< HEAD
    switch(vehicle_current_data.status) {
        case agv_straight:
            renew_motor_drive(setpoint_straight);
            // 改為agv_next，直到離開HALL，使else之後能renew status
            vehicle_current_data.status = agv_next;
            break;

        case agv_rotate:
            // 確定motor stop
            ensure_notor_stop();
            rotate_in_place();

            // 改為agv_next，直到離開HALL，使else之後能renew status
            vehicle_current_data.status = agv_next;
            break;

        case agv_end:
=======
    if(vehicle_current_data.status == agv_straight) {
        straight_mode();
    } else if(vehicle_current_data.status == agv_rotate) {
        // 確定motor stop
        if (motor_right.present_speed == min_duty && motor_left.present_speed == min_duty) {
            rotate_in_place();
        } else {
>>>>>>> c07caf725bb0cf636f62ad409c4eeb6e91d34c0a
            setpoint_current = 0;
            break;
    }
}



/* 保護未完成動作卻已超出hall範圍 -------------------------------------*/
void protect_over_hall(void) {
    ensure_notor_stop();

    //防止 原地旋轉前 衝過hall_sensor速度仍未停止，後退並強制進入原地旋轉
    if (setpoint_current == 0 && vehicle_current_data.status == agv_next) {
        over_hall_fall_back();
        rotate_in_place();

    //防止 結束後 衝過hall_sensor 速度仍未停止，進行後退
    } else if (vehicle_current_data.status == agv_end){
        over_hall_fall_back();

    }
}



/* 直到左右馬達停止才下個動作 -----------------------------------------*/
void ensure_notor_stop(void) {
    while(motor_right.present_speed == 0 && motor_left.present_speed == 0) {
        setpoint_current = 0;
    }
}
