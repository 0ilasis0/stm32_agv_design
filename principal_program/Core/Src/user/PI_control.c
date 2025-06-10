#include "user/PI_control.h"
#include <math.h>
#include "user/const_and_error.h"
#include "user/user_it.h"
#include "tim.h"

float max_speed_pcn = 40.0;

float output_pwm_Value;
float error;
float integral_temp;
float setpoint;
/* +PI speed control ------------------------------------------------*/
void PI_Controller(MOTOR_PARAMETER *motor) {
    if (motor == &motor_left) return;
    if (!PI_enable) return;

    setpoint = (float)max_speed_pcn * motor->speed_sepoint_pcn / 100;

    // 計算誤差
    error = setpoint - motor->speed_present;
    integral_temp = motor->integral_record + error;
    // 計算 P I 控制輸出
    output_pwm_Value = (float)Kp * error + Ki * integral_temp;

    if (!set_motor_duty(motor, motor->duty_value + output_pwm_Value)) return;

    motor->integral_record = integral_temp;
}
