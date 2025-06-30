#include "main/main.h"
#include "main/config.h"
#include "main/adc.h"
#include "main/it.h"
#include "main/vehicle.h"
#include "motor/main.h"
#include "connectivity/uart/main.h"

static uint16_t htim2_tick = 0;

GlobalState global_state = {
    .uart_tr_pkt_buf_h = &uart_tr_pkt_buf,
    .uart_rv_pkt_buf_h = &uart_rv_pkt_buf,
};

void USER_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        if (htim2_tick % 1000 == 0)
        {
            htim2_tick = 0;
            motor_speed_calculate(&motor_right, 0.1f);
            motor_speed_calculate(&motor_left, 0.1f);
        }
        htim2_tick++;
    }
}

size_t defalt_running = 0;
void StartDefaultTask(void *argument)
{
    // adc_setup();
    // map_setup();

    // vehicle_test_no_load_speed(1000);

    /*測試用--------------------------------------*/
    // motor_set_duty(&motor_right, 50);
    motor_set_speed_setpoint(&motor_right, 100);
    // motor_set_speed_setpoint(&motor_left, 30);
    // motor_right.adc_value = HALL_MAGNITUTE_EDGE + 1;
    // vehicle_over_hall_fall_back();
    // init_map_data_direction_and_address(&map_data, 5, 7);
    /*測試用--------------------------------------*/

    // vehicle_adjust_startup_heading();
    for(;;)
    {
        // vehicle_main

        osDelay(10); // !DO NOT CANCEL THIS LINE
        defalt_running++;
    }
}
