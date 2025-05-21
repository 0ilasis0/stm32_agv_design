#include "principal/principal_it.h"
#include "principal/const_and_error.h"
#include "principal/vehicle.h"
#include "principal/PI_control.h"
#include "stm32g4xx_hal_gpio.h"

uint32_t temp_time1 = 0;
uint32_t temp_time2 = 0;
uint8_t tim2_tick   = 0;
int toggle1 = 1;
int toggle2 = 0;

/**
  * 處理右側馬達霍爾感測 EXTI 中斷
  *
  * Handle EXTI interrupt for right motor Hall sensor
  */
void principal_EXTI3_IRQHandler(void) {
    motor_right.step_count++;
    update_motor_step(&motor_right);
}

/**
  * 處理左側馬達霍爾感測 EXTI 中斷
  *
  * Handle EXTI interrupt for left motor Hall sensor
  */
void principal_EXTI9_5_IRQHandler(void) {
    motor_left.step_count++;
    update_motor_step(&motor_left);
}

/**
  * TIM1 更新或 TIM16 中斷服務函式，用於呼叫速度計算
  *
  * TIM1 update or TIM16 interrupt handler to invoke speed calculation
  */
void principal_TIM1_UP_TIM16_IRQHandler(void) {
    speed_calculate(&motor_right);
    speed_calculate(&motor_left);
    PI_Controller(&motor_right);
    PI_Controller(&motor_left);
}

void principal_TIM2_IRQHandler(void) {
    tim2_tick++;
    if (tim2_tick >= 100) {
        tim2_tick = 0;
        update_motor_step(&motor_right);
        update_motor_step(&motor_left );
    }
}

/**
  * PC13 按鈕中斷，用於測試 (上下緣觸發)，切換 hall_sensor3
  *
  * Test interrupt for PC13 button (trigger on both edges), toggle hall_sensor3
  */
void EXTI15_10_IRQHandler_principal_it(void) {
    if (HAL_GetTick() - temp_time1 >= 300) {
        temp_time1 = HAL_GetTick();
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        if (toggle1 == 1) {
            hall_sensor3 = 0;
            toggle1 = 0;
        } else {
            hall_sensor3 = 16*16*16 + 16*16 + 16 + 1 + 1;
            toggle1 = 1;
        }
    }
}

/**
  * PC4 EXTI 中斷服務函式，用於測試，切換 hall_count_direction
  *
  * EXTI interrupt handler for PC4 test, toggle hall_count_direction
  */
void principal_EXTI4_IRQHandler(void) {
    if (HAL_GetTick() - temp_time2 >= 300) {
        temp_time2 = HAL_GetTick();
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        if (toggle2 == 1) {
            hall_count_direction = 0;
            toggle2 = 0;
        } else {
            hall_count_direction = 16*16*16 + 16*16 + 16 + 1 + 1;
            toggle2 = 1;
        }
    }
}
