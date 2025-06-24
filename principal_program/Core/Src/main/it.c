#include "main/it.h"
#include "main/config.h"
#include "main/vehicle.h"
#include "motor/main.h"
#include "motor/PI_control.h"
#include "connectivity/uart/main.h"
#include "connectivity/fdcan/main.h"

uint32_t temp_time1 = 0;
uint32_t temp_time2 = 0;
bool toggle1 = 1;
bool toggle2 = 0;

/**
  * PC13 按鈕中斷，用於測試 ，切換 hall_sensor_node
  *
  * Test interrupt for PC13 button (trigger on both edges), toggle hall_sensor_node
  */
void user_EXTI15_10_IRQHandler(void) {
    BOARD_LED_TOGGLE;
    if (HAL_GetTick() - temp_time1 >= 300) {
        temp_time1 = HAL_GetTick();

        if (toggle1 == 1) {
            hall_sensor_node = 0;
            toggle1 = 0;
        } else {
            hall_sensor_node = HALL_MAGNITUTE_EDGE + 1;
            toggle1 = 1;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (
           (GPIO_Pin == GPIO_PIN_1)
        || (GPIO_Pin == GPIO_PIN_2)
        || (GPIO_Pin == GPIO_PIN_3)
    ) {
        motor_add_step_count(&motor_right);
        motor_step_update(&motor_right);
    }
    else if (
           (GPIO_Pin == GPIO_PIN_5)
        || (GPIO_Pin == GPIO_PIN_6)
        || (GPIO_Pin == GPIO_PIN_8)
    ) {
        motor_add_step_count(&motor_left);
        motor_step_update(&motor_left);
    }
    else if (GPIO_Pin == GPIO_PIN_4)
    {
        BOARD_LED_TOGGLE;
        if (HAL_GetTick() - temp_time2 >= 300) {
            temp_time2 = HAL_GetTick();
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

            if (toggle2 == 1) {
                hall_sensor_direction = 0;
                toggle2 = 0;
            } else {
                hall_sensor_direction = HALL_MAGNITUTE_EDGE + 1;
                toggle2 = 1;
            }
        }
    }
}
