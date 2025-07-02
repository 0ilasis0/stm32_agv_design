#include "main/main.h"
#include "main/config.h"
#include "main/adc.h"
#include "main/it.h"
#include "main/vehicle.h"
#include "main/map.h"
#include "motor/main.h"
#include "us_sensor/main.h"
#include "connectivity/uart/main.h"

static uint16_t motor_tim_tick = 0;

GlobalState global_state = {
    .uart_tr_pkt_buf_h = &uart_tr_pkt_buf,
    .uart_rv_pkt_buf_h = &uart_rv_pkt_buf,
};

void USER_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == MOTOR_HTIM)
    {
        if (motor_tim_tick % 1000 == 0)
        {
            motor_tim_tick = 0;
            motor_speed_calculate(&motor_right, 0.1f);
            motor_speed_calculate(&motor_left, 0.1f);
            adc_renew();
        }
        motor_tim_tick++;
    }
    // else if (htim == US_SENSOR_HTIM && htim->Channel == US_SENSOR_TIM_ACT_CH)
    // {
    //     us_sensor_overflow();
    //     us_sensor_start();
    // }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    // if (htim == US_SENSOR_HTIM)
    // {
    //     us_sensor_tri_off();
    // }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (
           (GPIO_Pin == motor_right.const_h->Hall_GPIO_Pin_x[0])
        || (GPIO_Pin == motor_right.const_h->Hall_GPIO_Pin_x[1])
        || (GPIO_Pin == motor_right.const_h->Hall_GPIO_Pin_x[2])
    ) {
        motor_add_step_count(&motor_right);
        motor_step_update(&motor_right);
    }
    else if (
           (GPIO_Pin == motor_left.const_h->Hall_GPIO_Pin_x[0])
        || (GPIO_Pin == motor_left.const_h->Hall_GPIO_Pin_x[1])
        || (GPIO_Pin == motor_left.const_h->Hall_GPIO_Pin_x[2])
    ) {
        motor_add_step_count(&motor_left);
        motor_step_update(&motor_left);
    }
    // else if (GPIO_Pin == us_sensor_head.const_h->echo_GPIO_Pin_x)
    // {
    //     us_sensor_stop(&us_sensor_head);
    // }
}

size_t defalt_running = 0;
void StartDefaultTask(void *argument)
{
    adc_setup();
    map_setup();

    // vehicle_test_no_load_speed(1000);

    /*測試用--------------------------------------*/
    // motor_set_duty(&motor_right, 50);
    // motor_set_duty(&motor_left, 50);
    // motor_set_speed_setpoint(&motor_left, 100);
    // motor_set_speed_setpoint(&motor_right, 100);

    // init_map_data_direction_and_address(&map_data, 11, 7);
    /*測試用--------------------------------------*/

    // vehicle_adjust_startup_heading();
    for(;;)
    {
        vehicle_main();

        osDelay(10); // !DO NOT CANCEL THIS LINE
        defalt_running++;
    }
}
