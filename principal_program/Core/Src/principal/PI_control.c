#include "principal/PI_control.h"
#include <math.h>
#include "principal/const_and_error.h"
#include "tim.h"

double error_threshold = 0.7;               //限制積分累積，避免長時間造成積分風暴

/* +setup -----------------------------------------------------------*/
void PI_tim_setup(void){
    HAL_TIM_Base_Start_IT(&htim1);
}

/* +PI speed control ------------------------------------------------*/
void PI_Controller(MOTOR_PARAMETER *motor, float measurement) {
    if (!PI_CONTROL_DISABLE) return;

    int setpoint = max_speed / 100 * motor->speed_sepoint;

    // 計算誤差
    float error = setpoint - measurement;
    if (fabs(error) > error_threshold) {                             // 積分項累積
        motor->integral_record += error * dt;
    }

    // 計算 P I 控制輸出
    int output_pwm_Value = (Kp * error + Ki * motor->integral_record) * PI_feedbacck;

    // 限制PWM最大值
    uint32_t pwmValue_temp = motor->duty_value + output_pwm_Value;
    if (pwmValue_temp > max_duty) {
        motor->duty_value = max_duty;
    } else {
        motor->duty_value += output_pwm_Value;
    }
}
