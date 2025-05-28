#include "user/user_main.h"
#include "user/motor.h"
#include "user/PI_control.h"
#include "user/user_adc.h"
#include "user/user_uart.h"
#include "user/packet_proc_mod.h"
#include "user/map.h"

/*測試用--------------------------------------*/
uint32_t hall_sensor3 = 16*16*16 + 16*16 + 16 + 1 +1;
uint32_t text = 0;
/*測試用--------------------------------------*/

/* +Main ------------------------------------------------------------*/
void user_main(void) {
    uart_setup();
    motor_setup();

    test_no_load_speed(1000);

    hall_detection_adc_setup();
    PI_tim_setup();
    // vehicle_setup();
    // map_setup();

/*測試用--------------------------------------*/
    // motor_speed_setpoint_set(&motor_right, setpoint_straight);
    map_current_data.current_count++ ;
/*測試用--------------------------------------*/

    while (1) {
        if (transceive_flags.need_tr_proc) uart_tr_packet_proccess();
        if (transceive_flags.need_re_proc) uart_re_packet_proccess(10);
        // track_mode();
        text=1;
        // rotate_in_place();
        // over_hall_fall_back();
    /*    if (hall_sensor3 > node_hall_critical_value) {
            decide_move_mode();

        } else {
            protect_over_hall();

            if (vehicle_current_data.status == agv_next) {
                map_current_data.current_count++ ;
                vehicle_current_data.status = decide_vehicle_status();
                vehicle_current_data.address_id = map_current_data.address_id[map_current_data.current_count];

            } else {
                track_mode();

            }
        }*/
    }
}

/* 決定移動MODE ------------------------------------------------------*/
void decide_move_mode(void) {
    // renew current data to next node
    vehicle_current_data.status = decide_vehicle_status();

    switch(vehicle_current_data.status) {
        case agv_straight:
            motor_left.speed_sepoint = setpoint_straight;
            motor_right.speed_sepoint = setpoint_straight;
            // 改為agv_next，直到離開HALL，使else之後能renew status
            vehicle_current_data.status = agv_next;
            break;

        case agv_rotate:
            // 確定motor stop
            ensure_motor_stop();
            rotate_in_place();

            // 改為agv_next，直到離開HALL，使else之後能renew status
            vehicle_current_data.status = agv_next;
            break;

        case agv_end:
            motor_right.speed_sepoint = 0;
            motor_left.speed_sepoint  = 0;
            break;
    }
}

/*
 * 決定agv當前狀態
 */
AGV_STATUS decide_vehicle_status(void) {
    if (vehicle_current_data.direction == map_current_data.direction[map_current_data.current_count]) {
        return agv_straight;

    } else if (map_current_data.direction[map_current_data.current_count] == no_data){
        return agv_end;

    } else {
        return agv_rotate;

    }
}

/* 保護未完成動作卻已超出hall範圍 -------------------------------------*/
void protect_over_hall(void) {
    ensure_motor_stop();

    //防止 原地旋轉前 衝過hall_sensor速度仍未停止，後退並強制進入原地旋轉
    if (motor_right.speed_sepoint == 0 &&
        motor_left.speed_sepoint == 0 &&
        vehicle_current_data.status == agv_next
     ) {
        over_hall_fall_back();
        rotate_in_place();

    //防止 結束後 衝過hall_sensor 速度仍未停止，進行後退
    } else if (vehicle_current_data.status == agv_end){
        over_hall_fall_back();

    }
}
