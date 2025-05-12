#include "principal/principal_it.h"
#include "principal/const.h"
#include "principal/vehicle.h"
#include "principal/PI_control.h"
#include "stm32g4xx_hal_gpio.h"

float temp_time1 = 0;
float temp_time2 = 0;
int toggle1 = 1;
int toggle2 = 0;

/* +hall exit count -------------------------------------------------*/
void principal_EXTI3_IRQHandler(void) {
    update_motor_step(&motor_right);
}

void principal_EXTI9_5_IRQHandler(void) {
    update_motor_step(&motor_left);
}

/* +PI speed control -----------------------------------------------*/
void principal_TIM1_UP_TIM16_IRQHandler(void) {                //計時到，進行temp_pwm更新
    time_rpm(&motor_right);
    time_rpm(&motor_left);

    motor_left.rpm_count = 0;                           //將rpm計速器歸零
    motor_right.rpm_count = 0;
}

void time_rpm(MOTOR_PARAMETER *motor) {
    if(motor->adc_value < track_hall_critical_value && !ADC_DISABLE) {
        return;
    }

    float real_speed = motor->rpm_count/6;
    real_speed /= dt;
    motor->present_speed = real_speed;                        // 紀錄當前速度
    
    PI_Controller(motor, real_speed);
}

/* 測試用PC13按鈕中斷 -----------------------------------------------*/
void EXTI15_10_IRQHandler_principal_it(void) {
    if (HAL_GetTick() - temp_time1 >= 300) {
        temp_time1 = HAL_GetTick();
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        if (toggle1 == 1) {
            hall_sensor3 = 0;
            toggle1 = 0;
        } else {
            hall_sensor3 = 16*16*16 + 16*16 + 16 + 1 +1;
            toggle1 = 1;
        }
    }
}

//為上下緣觸發 PC4
void principal_EXTI4_IRQHandler(void) {
    if (HAL_GetTick() - temp_time2 >= 300) {
        temp_time2 = HAL_GetTick();
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        if (toggle2 == 1) {
            hall_count_direction = 0;
            toggle2 = 0;
        } else {
            hall_count_direction = 16*16*16 + 16*16 + 16 + 1 +1;
            toggle2 = 1;
        }
    }
}
