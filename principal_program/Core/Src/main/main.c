#include "main/main.h"
#include "main/config.h"
#include "vehicle/main.h"
#include "motor/main.h"
#include "us_sensor/main.h"
#include "adc/main.h"
#include "us_sensor/main.h"
#include "connectivity/uart/main.h"

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
    else if (htim == US_SENSOR_HTIM && htim->Channel == US_SENSOR_TIM_ACT_CH)
    {
        us_sensor_overflow();
        us_sensor_start();
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == US_SENSOR_HTIM)
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
        us_sensor_stop(&us_sensor_head);
    }
}

size_t defalt_running = 0;
void StartDefaultTask(void *argument)
{
    defalt_running++;

    osDelay(1000);

    /*測試用--------------------------------------*/
    // vehicle_test_no_load_rps(1000);
    // motor_set_state(&motor_left, MOTOR_STATE_FREE);
    // motor_set_direct(&motor_left, MOTOR_DIRECTION_CLW);
    // motor_set_rps_pcn(&motor_left, 50);
    // motor_set_state(&motor_right, MOTOR_STATE_FREE);
    // motor_set_direct(&motor_right, MOTOR_DIRECTION_CCLW);
    // motor_set_rps_pcn(&motor_right, 50);
    vehicle_set_motion(VEHICLE_MOTION_CLOCKWISE);
    vehicle_set_speed(20);
    // map_data_renew_direction_and_address(&map_data_start, 11, 7);
    /*測試用--------------------------------------*/

    for(;;)
    {
        switch (vehicle_h.mode)
        {
            case VEHICLE_MODE_END:
            {
                vehicle_set_motion(VEHICLE_MOTION_STOP);
                vehicle_set_mode(VEHICLE_MODE_FREE);
                break;
            }
            case VEHICLE_MODE_TRACK:
            {
                vehicle_track_mode(1000);
                break;
            }
            case VEHICLE_MODE_SEARCH:
            {
                if (ERROR_CHECK_FNS_RAW(vehicle_search_mode(20, 2000)))
                {
                    vehicle_set_motion(VEHICLE_MOTION_STOP);
                    vehicle_ensure_stop();
                    vehicle_set_mode(VEHICLE_MODE_FREE);
                    break;
                }
                vehicle_ensure_stop();
                vehicle_set_mode(VEHICLE_MODE_TRACK);
                break;
            }
            case VEHICLE_MODE_ROTATE:
            {
                vehicle_rotate_in_place(1000);
                break;
            }
            default: break;
        }
        osDelay(50); // !DO NOT CANCEL THIS LINE
        defalt_running++;
    }
}
