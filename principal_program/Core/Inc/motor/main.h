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
#define MOTOR_STATE_CONTROL     0
#define MOTOR_STATE_FREE        1
#define MOTOR_STATE_SLOW        2
#define MOTOR_STATE_COAST       3
#define MOTOR_STATE_BREAK       4
#define MOTOR_STATE_LOCK        5

typedef int8_t MotorDirection;
#define MOTOR_DIRECTION_CLW     1
#define MOTOR_DIRECTION_STOP    0
#define MOTOR_DIRECTION_CCLW   -1

typedef struct MotorParameter
{
    const MotorConst const_h;
    MotorState state;
    MotorState state_inner;
    // RPS setpoint
    Percentage rps_pcn;
    Percentage rps_pcn_inner;
    float rps_max;
    float rps_present;
    Percentage pwm_duty;
    // direction setpoint
    MotorDirection direction;
    MotorDirection direction_inner;
    MotorDirection direction_present;
    uint8_t hall_last;
    uint8_t hall_present;
    uint32_t step_count;
    float integral_record;
} MotorParameter;

extern MotorParameter motor_right;
extern MotorParameter motor_left;

void motor_set_max_rps(MotorParameter* motor, float value);
void motor_set_duty(MotorParameter *motor, uint8_t value);
void motor_set_state(MotorParameter *motor, MotorState state);
void motor_set_rps_pcn(MotorParameter* motor, Percentage value);
void motor_set_direction(MotorParameter *motor, MotorDirection direction);
void motor_HALL_EXTI(MotorParameter *motor);
