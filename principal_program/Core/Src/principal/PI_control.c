#include "principal/PI_control.h"
#include <math.h>
#include "principal/const.h"
#include "tim.h"

int setpoint_current = setpoint_straight;
double error_threshold = 0.5;                                          //限制積分累積，避免長時間造成積分風暴

/* +setup -----------------------------------------------------------*/
void PI_tim_setup(void){
    HAL_TIM_Base_Start_IT(&htim1);
}

/* +PI speed control ------------------------------------------------*/
void PI_Controller(MOTOR_PARAMETER *motor, double measurement) {
    if (PI_CONTROL_DISABLE) return;

    double error = setpoint_current - measurement;                   // 計算誤差
    if (fabs(error) > error_threshold) {                             // 積分項累積
        motor->integral_record += error * dt;
    }
    int output_pwm_Value = Kp * error + Ki * motor->integral_record; // 計算 P I 控制輸出
    motor->pwmValue_temp += output_pwm_Value * PI_feedbacck;

    // return motor->pwmValue_temp;
}
