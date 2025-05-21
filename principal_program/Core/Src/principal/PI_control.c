#include "principal/PI_control.h"
#include <math.h>
#include "principal/const_and_error.h"
#include "tim.h"

double error_threshold = 0.7;               //限制積分累積，避免長時間造成積分風暴
uint16_t max_speed = 100;

/* +setup -----------------------------------------------------------*/
void PI_tim_setup(void){
    HAL_TIM_Base_Start_IT(&htim1);
}

int PI_Controller_max_speed = 0;
/* +PI speed control ------------------------------------------------*/
void PI_Controller(MOTOR_PARAMETER *motor) {
    if (!PI_enable) return;

    int setpoint = max_speed * motor->speed_sepoint / 100;
    PI_Controller_max_speed = max_speed;

    // 計算誤差
    float error = setpoint - motor->present_speed;
    if (fabs(error) > error_threshold) {                             // 積分項累積
        motor->integral_record += error * dt;
    }

    // 計算 P I 控制輸出
    int16_t output_pwm_Value = (Kp * error + Ki * motor->integral_record) * PI_feedbacck;

    set_motor_duty(motor, motor->duty_value + output_pwm_Value);
}
