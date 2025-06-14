#include "motor/PI_control.h"
#include <math.h>
#include "tim.h"
#include "main/const_and_error.h"
#include "main/it.h"

float max_speed_pcn = 40.0;
float setpoint = 0;

/* +PI speed control ------------------------------------------------*/
void motor_PI_control(MOTOR_PARAMETER *motor) {
    if (!PI_enable) return;

    if (motor == &motor_left) return;

    setpoint = (float)max_speed_pcn * motor->speed_sepoint_pcn / 100;

    // 計算誤差
    float error = setpoint - motor->speed_present;
    float integral = motor->integral_record + error;
    // 計算 P I 控制輸出
    float output_pwm_Value = (float)Kp * error + Ki * integral;

    if (!motor_set_duty(motor, motor->duty_value + output_pwm_Value)) return;
    motor_set_integral_record(motor, integral);
}
