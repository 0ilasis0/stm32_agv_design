#include "user/user_main.h"
#include "user/motor.h"
#include "user/PI_control.h"
#include "user/user_adc.h"
#include "user/uart_mod.h"
#include "user/uart_packet_proc_mod.h"
#include "user/map.h"

/*測試用--------------------------------------*/
uint32_t hall_sensor_node = 16*16*16 + 16*16 + 16 + 1 +1;
uint32_t text = 0;
/*測試用--------------------------------------*/

/* +Main ------------------------------------------------------------*/
void user_main(void) {
    uart_setup();
    motor_setup();

    // test_no_load_speed(1000);

    // hall_detection_adc_setup();
    // PI_tim_setup();
    // map_setup();

/*測試用--------------------------------------*/
    // motor_speed_setpoint_set(&motor_right, setpoint_straight);
/*測試用--------------------------------------*/

    while (1) {
        uart_trcv_proccess();
        // track_mode();
        // rotate_in_place();
        // over_hall_fall_back();
/*
        if (hall_sensor_node > hall_node_value) {
            decide_move_mode();

        } else {
            if (map_data.status[map_data.current_count] == agv_next) {
                map_data.current_count++ ;

            } else {
                track_mode();

            }
        }
*/
        // HAL_Delay(1);
    }
}

/* 決定移動MODE ------------------------------------------------------*/
void decide_move_mode(void) {

    switch(map_data.status[map_data.current_count]) {
        case agv_straight:
            motor_left.speed_sepoint = setpoint_straight;
            motor_right.speed_sepoint = setpoint_straight;

            // 改為agv_next，直到離開HALL，使else之後能renew status
            map_data.status[map_data.current_count] = agv_next;
            break;

        case agv_rotate:
            protect_over_hall();
            rotate_in_place();

            // 改為agv_next，直到離開HALL，使else之後能renew status
            map_data.status[map_data.current_count] = agv_next;
            break;

        case agv_end:
            protect_over_hall();
            break;
    }
}

/* 保護未完成動作卻已超出hall範圍 -------------------------------------*/
void protect_over_hall(void) {
    ensure_motor_stop();

    if (hall_sensor_node > hall_node_value) return;

    //防止 原地旋轉前 衝過hall_sensor速度仍未停止，後退並強制進入原地旋轉
    if (map_data.status[map_data.current_count] == agv_rotate) {
        over_hall_fall_back();
    }

    //防止 結束後 衝過hall_sensor 速度仍未停止，進行後退
    if (map_data.status[map_data.current_count] == agv_end) {
        over_hall_fall_back();
    }
}
