#include "main/main.h"
#include "vehicle/main.h"
#include "motor/main.h"
#include "us_sensor/main.h"
#include "adc/main.h"
#include "us_sensor/main.h"

static size_t motor_tick = 0;

void USER_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == MOTOR_HTIM1) // 100us
    {
        if (motor_tick % 500 == 0) // 50ms
        {
            motor_tick = 0;

        }
        motor_tick++;
    }
    else if (htim == US_SENSOR_HTIM)
    {
        us_sensor_overflow();
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == US_SENSOR_HTIM && htim->Channel == US_SENSOR_TIM_ACT_CH)
    {
        us_sensor_tri_off();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (
           (GPIO_Pin == motor_left.const_h.Hall_GPIO_Pin_x[0]) // 2
        || (GPIO_Pin == motor_left.const_h.Hall_GPIO_Pin_x[1]) // 12
        || (GPIO_Pin == motor_left.const_h.Hall_GPIO_Pin_x[2]) // 15
    ) {
        motor_HALL_EXTI(&motor_left);
    }
    else if (
           (GPIO_Pin == motor_right.const_h.Hall_GPIO_Pin_x[0]) // 8
        || (GPIO_Pin == motor_right.const_h.Hall_GPIO_Pin_x[1]) // 4
        || (GPIO_Pin == motor_right.const_h.Hall_GPIO_Pin_x[2]) // 5
    ) {
        motor_HALL_EXTI(&motor_right);
    }
    else if (GPIO_Pin == us_sensor_head.const_h.echo_GPIO_Pin_x) // 6
    {
        us_sensor_echo();
    }
}

uint32_t defalt_running;
void StartDefaultTask(void *argument)
{
    defalt_running = HAL_GetTick();
    while (
           !motor_ready
        || !vehicle_ready
    ) osDelay(50);
    // motor_set_state(&motor_left, MOTOR_STATE_FREE);
    // motor_set_direct(&motor_left, MOTOR_DIRECTION_CLW);
    // motor_set_rps_pcn(&motor_left, 50);
    // motor_set_state(&motor_right, MOTOR_STATE_FREE);
    // motor_set_direct(&motor_right, MOTOR_DIRECTION_CCLW);
    // motor_set_rps_pcn(&motor_right, 50);
    for(;;)
    {
        osDelay(100); // !DO NOT CANCEL THIS LINE
        defalt_running = HAL_GetTick();
    }
}
