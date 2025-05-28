#include "user/PI_control.h"
#include <math.h>
#include "user/const_and_error.h"
#include "user/user_it.h"
#include "tim.h"

float max_speed = 0.0;
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
    float itr = motor->integral_record + error * dt;

    // 計算 P I 控制輸出
    int16_t output_pwm_Value = (Kp * error + Ki * itr) * PI_feedbacck;

    if (!set_motor_duty(motor, motor->duty_value + output_pwm_Value)) return;

    motor->integral_record = itr;
}
