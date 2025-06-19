#include "main/main.h"
#include "cmsis_os.h"
#include "main/adc.h"
#include "main/map.h"
#include "main/it.h"
#include "motor/main.h"
#include "motor/PI_control.h"
#include "uart/main.h"
#include "fdcan/main.h"

GlobalState global_state = {
    .uart_tr_pkt_buf_h = &uart_tr_pkt_buf,
    .uart_rv_pkt_buf_h = &uart_rv_pkt_buf,
};

/*測試用--------------------------------------*/
uint32_t hall_sensor_node = HALL_MAGNITUTE_EDGE +1;
/*測試用--------------------------------------*/

void StartDefaultTask(void *argument)
{
    // fdcan_setup();
    adc_setup();
    map_setup();

    // vehicle_test_no_load_speed(1000);

    vehicle_adjust_startup_heading ();

    /*測試用--------------------------------------*/
    // motor_set_speed_setpoint(&motor_right, 100);
    // motor_set_duty(&motor_right, 75);

    // vehicle_rotate_in_place();
    // vehicle_over_hall_fall_back();

    /*測試用--------------------------------------*/
    for(;;)
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

        osDelay(1); // !DO NOT CANCEL THIS LINE
    }
}

uint32_t user_sys_tick = 0;
void HAL_IncTick(void)
{
    uwTick += uwTickFreq; // !DO NOT CANCEL THIS LINE

    user_sys_tick++;
    // 10ms
    // if (user_sys_tick % 10 == 0) {
    //     motor_step_update(&motor_right);
    //     motor_step_update(&motor_left );
    // }
    // if (user_sys_tick % 50 == 0) {
    // }
    // if (user_sys_tick % 100 == 0) {
    //     motor_speed_calculate(&motor_right);
    //     motor_speed_calculate(&motor_left);
    // }
    // if (user_sys_tick % 500 == 0) {
    //     motor_PI_control(&motor_right);
    //     motor_PI_control(&motor_left);
    // }
    if (user_sys_tick % 1000 == 0) {
        fdcan_transmit();
    }
    if (user_sys_tick % 2000 == 0) {
    }
    // 60s
    if (user_sys_tick >= 60000) {
        user_sys_tick = 0;
    }
}

/* 決定移動MODE ------------------------------------------------------*/
void decide_move_mode(void)
{
    switch(map_data.status[map_data.current_count])
    {
        case agv_straight:

            motor_left.speed_sepoint_pcn = setpoint_straight;
            motor_right.speed_sepoint_pcn = setpoint_straight;

            // 改為agv_next，直到離開HALL，使else之後能renew status
            map_data.status[map_data.current_count] = agv_next;
            break;

        case agv_rotate:
            protect_over_hall();
            vehicle_rotate_in_place();

            // 改為agv_next，直到離開HALL，使else之後能renew status
            map_data.status[map_data.current_count] = agv_next;
            break;

        case agv_end:
            protect_over_hall();
            map_data.start_direction = map_data.direction[map_data.current_count];
            break;
    }
}

/* 保護未完成動作卻已超出hall範圍 -------------------------------------*/
void protect_over_hall(void)
{
    vehicle_ensure_motor_stop();

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
