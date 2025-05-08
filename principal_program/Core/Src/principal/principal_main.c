#include "principal/principal_main.h"


uint32_t hall_sensor3 = 16*16*16 + 16*16 + 16 + 1 +1;
/* +Main ------------------------------------------------------------*/
void MCmain(void) {
    motor_tim_setup(&motor_right);
    motor_tim_setup(&motor_left);
    hall_detection_adc_setup();
    PI_tim_setup();
    vehicle_setup();
    uart_setup();

    update_motor_step(&motor_right);
    update_motor_step(&motor_left);
    while (1) {
        commutate_motor(&motor_right);
        commutate_motor(&motor_left);

        if(hall_sensor3 > node_hall_critical_value) {
            decide_move_mode();
            // 改為end，直到離開HALL，使else之後能renew status
            vehicle_current_data.status = agv_end;
        } else {
            if(vehicle_current_data.status == agv_end) {
                map_current_data.current_count++ ;
                vehicle_current_data.status = map_current_data.status[map_current_data.current_count];
            }
            track_mode();
            adc_renew();
        }
    }
}

/* 決定移動MODE ------------------------------------------------------*/
void decide_move_mode(void) {
    // renew current data to next node
    vehicle_current_data.status = map_current_data.status[map_current_data.current_count];

    if(vehicle_current_data.status == agv_straight) {
        straight_mode();
    } else if(vehicle_current_data.status == agv_rotate) {
        // 確定motor stop
        if (motor_right.present_speed == min_duty && motor_left.present_speed == min_duty) {
            rotate_in_place();
        } else {
            setpoint_current = 0;
        }
    }
}
