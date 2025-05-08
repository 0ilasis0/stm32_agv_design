#include "principal/principal_it.h"

/* +hall exit count -------------------------------------------------*/
void principal_EXTI3_IRQHandler(void) {
    updateMotorStep(&motor_right);
    r_exit_rpm_count();
}

void principal_EXTI9_5_IRQHandler(void) {
    updateMotorStep(&motor_left);
    l_exit_rpm_count();
}

void r_exit_rpm_count() {
    motor_right.rpm_count++;
}

void l_exit_rpm_count() {
    motor_left.rpm_count++;
}



/* +PI speed control -----------------------------------------------*/
void TIM1_UP_TIM16_IRQHandler_use(void) {                //計時到，進行temp_pwm更新
    if(motor_right.adc_value >= track_hall_critical_value) {
        motor_right.pwmValue_temp = time_rpm(&motor_right);
    }
    if(motor_left.adc_value >= track_hall_critical_value) {
        motor_left.pwmValue_temp = time_rpm(&motor_left);
    }

    motor_left.rpm_count = 0;                           //將rpm計速器歸零
    motor_right.rpm_count = 0;
}

int time_rpm(MOTOR_PARAMETER *motor) {
    float real_speed = motor->rpm_count/6;
    real_speed/=dt;
    motor->present_speed = real_speed;                        // 紀錄當前速度

    int pwmValue_renew = PI_Controller(motor, real_speed);
    return pwmValue_renew;
}

