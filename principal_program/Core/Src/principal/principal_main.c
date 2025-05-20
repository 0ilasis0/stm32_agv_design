#include "principal/principal_main.h"
#include "principal/motor.h"
#include "principal/vehicle.h"
#include "principal/PI_control.h"
#include "principal/principal_adc.h"
#include "principal/principal_uart.h"

/*測試用--------------------------------------*/
uint32_t hall_sensor3 = 16*16*16 + 16*16 + 16 + 1 +1;
uint32_t text = 0;
/*測試用--------------------------------------*/

/* +Main ------------------------------------------------------------*/
void principal_main(void) {
    uart_setup();
    motor_setup();

    update_motor_step(&motor_right);
    update_motor_step(&motor_left);
    // test_no_load_speed();

    hall_detection_adc_setup();
    PI_tim_setup();
    vehicle_setup();


/*測試用--------------------------------------*/
    map_current_data.current_count++ ;
/*測試用--------------------------------------*/
    commutate_motor(&motor_right);
    commutate_motor(&motor_left );
    while (1) {
        // track_mode();

        // rotate_in_place();
        // over_hall_fall_back();

    /*    if (hall_sensor3 > node_hall_critical_value) {
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
        }*/
    }
}

/* 決定移動MODE ------------------------------------------------------*/
void decide_move_mode(void) {
    // renew current data to next node
    vehicle_current_data.status = map_current_data.status[map_current_data.current_count];

    switch(vehicle_current_data.status) {
        case agv_straight:
            renew_motor_drive(&motor_left, setpoint_straight);
            renew_motor_drive(&motor_right, setpoint_straight);
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
