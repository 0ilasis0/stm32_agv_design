#include "user/PI_control.h"
#include <math.h>
#include "user/const_and_error.h"
#include "tim.h"

uint16_t max_speed = 10;
const float dt = 0.5;

/* +setup -----------------------------------------------------------*/
void PI_tim_setup(void){
    HAL_TIM_Base_Start_IT(&htim1);
}

/* +PI speed control ------------------------------------------------*/
void PI_Controller(MOTOR_PARAMETER *motor) {
    if (!PI_enable) return;

    int setpoint = max_speed * motor->speed_sepoint / 100;

    // 計算誤差
    float error = setpoint - motor->speed_present;
    motor->integral_record += error * dt;


    // 計算 P I 控制輸出
    int16_t output_pwm_Value = (Kp * error + Ki * motor->integral_record) * PI_feedbacck;

    set_motor_duty(motor, motor->duty_value + output_pwm_Value);
}
