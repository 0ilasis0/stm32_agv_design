#include "user/PI_control.h"
#include <math.h>
#include "user/const_and_error.h"
#include "user/user_it.h"
#include "tim.h"

float max_speed_pcn = 40.0;
float setpoint = 0;

/* +PI speed control ------------------------------------------------*/
void PI_Controller(MOTOR_PARAMETER *motor) {
    if (motor == &motor_left) return;
    if (!PI_enable) return;

    setpoint = (float)max_speed_pcn * motor->speed_sepoint_pcn / 100;

    // 計算誤差
    float error = setpoint - motor->speed_present;
    float integral_temp = motor->integral_record + error;
    // 計算 P I 控制輸出
    float output_pwm_Value = (float)Kp * error + Ki * integral_temp;

    if (!motor_set_duty(motor, motor->duty_value + output_pwm_Value)) return;

    motor->integral_record = integral_temp;
}
