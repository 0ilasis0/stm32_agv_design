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

typedef enum MotorMode
{
    MOTOR_STATE_CONTROL,
    MOTOR_STATE_FREE,
    MOTOR_STATE_SLOW,
    MOTOR_STATE_SLOW_0,
    MOTOR_STATE_COAST,
    MOTOR_STATE_BREAK,
    MOTOR_STATE_LOCK = -1,
} MotorMode;

typedef enum MotorMotion
{
    MOTOR_DIRECTION_STOP,
    MOTOR_DIRECTION_CLW,
    MOTOR_DIRECTION_CCLW,
} MotorMotion;

typedef struct MotorParameter
{
    const MotorConst const_h;
    MotorMode state;
    MotorMode state_inner;
    // RPS setpoint
    Percentage rps_pcn;
    Percentage rps_pcn_inner;
    float rps_max;
    float rps_present;
    Percentage pwm_duty;
    // motion setpoint
    MotorMotion motion;
    MotorMotion motion_inner;
    MotorMotion motion_present;
    uint8_t hall_last;
    uint8_t hall_present;
    uint32_t step_count;
    float integral_record;
} MotorParameter;

extern MotorParameter motor_right;
extern MotorParameter motor_left;

extern bool motor_ready;

void motor_set_max_rps(MotorParameter* motor, float value);
void motor_set_state(MotorParameter *motor, MotorMode state);
void motor_set_rps_pcn(MotorParameter* motor, Percentage value);
void motor_set_direct(MotorParameter *motor, MotorMotion motion);
void motor_HALL_EXTI(MotorParameter *motor);
