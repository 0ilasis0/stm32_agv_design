#include "principal/principal_it.h"
#include "principal/const.h"
#include "principal/vehicle.h"
#include "principal/PI_control.h"

/* +hall exit count -------------------------------------------------*/
void principal_EXTI3_IRQHandler(void) {
    update_motor_step(&motor_right);
}

void principal_EXTI9_5_IRQHandler(void) {
    update_motor_step(&motor_left);
}

/* +PI speed control -----------------------------------------------*/
void principal_TIM1_UP_TIM16_IRQHandler(void) {                //計時到，進行temp_pwm更新
    // if(motor_right.adc_value >= track_hall_critical_value) {
    //     motor_right.pwmValue_temp = time_rpm(&motor_right);
    // }
    // if(motor_left.adc_value >= track_hall_critical_value) {
    //     motor_left.pwmValue_temp = time_rpm(&motor_left);
    // }
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

    // int pwmValue_renew = PI_Controller(motor, real_speed);
    // return pwmValue_renew;
    PI_Controller(motor, real_speed);
}
