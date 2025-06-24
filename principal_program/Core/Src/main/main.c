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

    vehicle_adjust_startup_heading ();

    /*測試用--------------------------------------*/
    // motor_set_speed_setpoint(&motor_right, 100);
    motor_right.adc_value = HALL_MAGNITUTE_EDGE + 1;
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
    // }
    // 60s
    if (user_sys_tick >= 60000) {
        user_sys_tick = 0;
    }
}
