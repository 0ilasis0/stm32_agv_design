#include "main/main.h"
#include "cmsis_os.h"
#include "main/adc.h"
#include "main/map.h"
#include "main/it.h"
#include "main/vehicle.h"
#include "motor/main.h"
#include "motor/PI_control.h"
#include "connectivity/uart/main.h"

GlobalState global_state = {
    .uart_tr_pkt_buf_h = &uart_tr_pkt_buf,
    .uart_rv_pkt_buf_h = &uart_rv_pkt_buf,
};

void StartDefaultTask(void *argument)
{
    adc_setup();
    map_setup();

    // vehicle_test_no_load_speed(1000);

    // vehicle_adjust_startup_heading ();

    /*測試用--------------------------------------*/
    // motor_set_speed_setpoint(&motor_right, 100);
    // motor_right.adc_value = HALL_MAGNITUTE_EDGE + 1;

    // vehicle_rotate_in_place();
    // vehicle_over_hall_fall_back();

    /*測試用--------------------------------------*/
    for(;;)
    {
        // if (hall_sensor_node > hall_strong_magnet_value) {
        //     decide_move_mode();

        // } else {
        //     if (map_data.status[map_data.current_count] == agv_next) {
        //         map_data.current_count++ ;

        //     } else {
        //         vehicle_track_mode();

        //     }
        // }

        osDelay(1000); // !DO NOT CANCEL THIS LINE
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
    // 60s
    if (user_sys_tick >= 60000) {
        user_sys_tick = 0;
    }
}

/* 決定移動MODE ------------------------------------------------------*/
int text_end = 0;
void decide_move_mode(void)
{
    switch(map_data.status[map_data.current_count])
    {
        case agv_straight:
            motor_set_speed_setpoint(&motor_right, setpoint_straight);
            motor_set_speed_setpoint (&motor_left, setpoint_straight);

            // 改為agv_next，直到離開HALL，使else之後能renew status
            map_data.status[map_data.current_count] = agv_next;
            break;

        case agv_rotate:
            // protect_over_hall();
            vehicle_rotate_in_place();

            // 改為agv_next，直到離開HALL，使else之後能renew status
            map_data.status[map_data.current_count] = agv_next;
            break;

        case agv_end:
            // protect_over_hall();
            map_data_init(map_data.direction[map_data.current_count - 1], map_data.address_id[map_data.current_count - 1]);
            // 終止目前沒有要做甚麼所以先停止動作
            while (1) {
                motor_set_speed_setpoint(&motor_right, 0);
                motor_set_speed_setpoint (&motor_left, 0);
                text_end = 1;
            }
            break;
    }
}

/* 保護未完成動作卻已超出hall範圍 -------------------------------------*/
void protect_over_hall(void)
{
    vehicle2_ensure_motor_stop();

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
