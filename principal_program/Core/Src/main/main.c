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

size_t defalt_running = 0;
void StartDefaultTask(void *argument)
{
    adc_setup();
    map_setup();

    // vehicle_test_no_load_speed(1000);

    /*測試用--------------------------------------*/
    // motor_set_speed_setpoint(&motor_right, 100);
    motor_right.adc_value = HALL_MAGNITUTE_EDGE + 1;
    // vehicle_over_hall_fall_back();
    init_map_data_direction_and_address(&map_data, 5, 7);
    /*測試用--------------------------------------*/

    vehicle_adjust_startup_heading();
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
        defalt_running++;
    }
}
