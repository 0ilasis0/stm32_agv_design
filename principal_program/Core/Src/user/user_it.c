#include "user/user_it.h"
#include "user/const_and_error.h"
#include "user/vehicle.h"
#include "user/PI_control.h"
#include "user/uart_mod.h"
#include "user/uart_packet_proc_mod.h"
#include "stm32g4xx_hal_gpio.h"

uint32_t temp_time1 = 0;
uint32_t temp_time2 = 0;
uint32_t user_sys_tick = 0;
int toggle1 = 1;
int toggle2 = 0;

// trigger at 1ms
int text = 0;
void user_SysTick_Handler(void) {
    user_sys_tick++;
    // 10ms
    if (user_sys_tick % 10 == 0) {
        text = 1;
        motor_step_update(&motor_right);
        motor_step_update(&motor_left );
    }
    if (user_sys_tick % 50 == 0) {
        transceive_flags.uart_transmit = true;
        transceive_flags.uart_receive_pkt_proc = true;
    }
    if (user_sys_tick % 100 == 0) {
        speed_calculate(&motor_right);
        speed_calculate(&motor_left);
    }
    if (user_sys_tick % 500 == 0) {
        PI_Controller(&motor_right);
        PI_Controller(&motor_left);
    }
    if (user_sys_tick % 1000 == 0) {
        transceive_flags.uart_transmit_pkt_proc = true;
    }
    // 60s
    if (user_sys_tick >= 60000) {
        user_sys_tick = 0;
    }
}

/**
  * 處理右側馬達霍爾感測 EXTI 中斷
  *
  * Handle EXTI interrupt for right motor Hall sensor
  */
void user_EXTI3_IRQHandler(void) {
    motor_right.step_count++;
    motor_step_update(&motor_right);
}

/**
  * 處理左側馬達霍爾感測 EXTI 中斷
  *
  * Handle EXTI interrupt for left motor Hall sensor
  */
void user_EXTI9_5_IRQHandler(void) {
    motor_left.step_count++;
    motor_step_update(&motor_left);
}

/**
  * TIM1 更新或 TIM16 中斷服務函式，用於呼叫速度計算
  *
  * TIM1 update or TIM16 interrupt handler to invoke speed calculation
  */
void user_TIM1_UP_TIM16_IRQHandler(void) {

}

/**
  * PC13 按鈕中斷，用於測試 ，切換 hall_sensor_node
  *
  * Test interrupt for PC13 button (trigger on both edges), toggle hall_sensor_node
  */
void user_EXTI15_10_IRQHandler(void) {
    if (HAL_GetTick() - temp_time1 >= 300) {
        temp_time1 = HAL_GetTick();

        if (toggle1 == 1) {
            hall_sensor_node = 0;
            toggle1 = 0;
        } else {
            hall_sensor_node = 1300 + 1;
            toggle1 = 1;
        }
    }
}

/**
  * PC4 EXTI 中斷服務函式，用於測試(上下緣觸發)，切換 hall_sensor_direction
  *
  * EXTI interrupt handler for PC4 test, toggle hall_sensor_direction
  */
void user_EXTI4_IRQHandler(void) {
    if (HAL_GetTick() - temp_time2 >= 300) {
        temp_time2 = HAL_GetTick();
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        if (toggle2 == 1) {
            hall_sensor_direction = 0;
            toggle2 = 0;
        } else {
            hall_sensor_direction = 1300 + 1;
            toggle2 = 1;
        }
    }
}
