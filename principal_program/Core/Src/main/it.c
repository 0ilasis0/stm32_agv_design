#include "main/it.h"
#include "main/config.h"
#include "main/vehicle.h"
#include "motor/main.h"
#include "connectivity/uart/main.h"
#include "connectivity/fdcan/main.h"

/**
  * PC13 按鈕中斷，用於測試
  *
  * Test interrupt for PC13 button (trigger on both edges), toggle hall_sensor_node
  */
void user_EXTI15_10_IRQHandler(void) {
    vehicle2_ensure_motor_stop();
    Error_Handler();
}

size_t check[4] = {0};
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    check[0]++;
    if (
           (GPIO_Pin == motor_right.motor_const->Hall_GPIO_Pin_x[0])
        || (GPIO_Pin == motor_right.motor_const->Hall_GPIO_Pin_x[1])
        || (GPIO_Pin == motor_right.motor_const->Hall_GPIO_Pin_x[2])
    ) {
        motor_add_step_count(&motor_right);
        motor_step_update(&motor_right);
    }
    else if (
           (GPIO_Pin == motor_left.motor_const->Hall_GPIO_Pin_x[0])
        || (GPIO_Pin == motor_left.motor_const->Hall_GPIO_Pin_x[1])
        || (GPIO_Pin == motor_left.motor_const->Hall_GPIO_Pin_x[2])
    ) {
        motor_add_step_count(&motor_left);
        motor_step_update(&motor_left);
    }
}

