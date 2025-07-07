#pragma once

#include "main/config.h"
#include "main/fn_state.h"
#include "gpio.h"
#include "tim.h"

typedef struct MotorConst
{
    GPIO_TypeDef*       Hall_GPIOx[3];
    uint16_t            Hall_GPIO_Pin_x[3];
    TIM_HandleTypeDef*  htimx[3];
    uint32_t            TIM_CHANNEL_x[3];
    GPIO_TypeDef*       Coil_GPIOx[3];
    uint16_t            Coil_GPIO_Pin_x[3];
} MotorConst;

typedef uint8_t MotorState;
#define MOTOR_STATE_FREE    0
#define MOTOR_STATE_SLOW    1
#define MOTOR_STATE_COAST   2
#define MOTOR_STATE_BREAK   3
#define MOTOR_STATE_LOCK    4

typedef int8_t RotateState;
#define MOTOR_ROTATE_CLW    1
#define MOTOR_ROTATE_STOP   0
#define MOTOR_ROTATE_CCLW  -1

typedef struct MotorParameter
{
    const MotorConst const_h;
    MotorState state;
    // RPS setpoint
    Percentage rps_pcn_setpoint;
    float rps_max;
    float rps_present;
    Percentage pwm_duty;
    // direction setpoint
    RotateState direction_setpoint;
    RotateState direction_inner;
    RotateState direction_present;
    uint8_t hall_last;
    uint8_t hall_present;
    uint16_t step_count;
    float integral_record;
} MotorParameter;

extern MotorParameter motor_right;
extern MotorParameter motor_left;

void motor_set_max_rps(MotorParameter* motor, float value);
void motor_set_duty(MotorParameter *motor, uint8_t value);
void motor_set_state(MotorParameter *motor, MotorState state);
void motor_set_rps_pcn(MotorParameter* motor, Percentage value);
void motor_set_direction(MotorParameter *motor, RotateState direction);
void motor_tim_tick(float ms);
void motor_hall_exti(MotorParameter *motor);
