#pragma once

#include "main/config.h"
#include "main/fn_state.h"
#include "gpio.h"
#include "tim.h"

extern float max_speed;


typedef int8_t RotateState;
#define MOTOR_ROTATE_CLW    1
#define MOTOR_ROTATE_STOP   0
#define MOTOR_ROTATE_CCLW  -1

typedef uint8_t MotorState;
#define MOTOR_STATE_COAST   0
#define MOTOR_STATE_LOCK    1
#define MOTOR_STATE_BREAK   2

typedef struct MotorConst
{
    GPIO_TypeDef*       Hall_GPIOx[3];
    uint16_t            Hall_GPIO_Pin_x[3];
    TIM_HandleTypeDef*  htimx[3];
    uint32_t            TIM_CHANNEL_x[3];
    GPIO_TypeDef*       Coil_GPIOx[3];
    uint16_t            Coil_GPIO_Pin_x[3];
} MotorConst;

typedef struct MotorParameter
{
    const MotorConst const_h;
    // RPS setpoint
    Percentage rps_setpoint;
    // real RPS setpoint (control by system)
    Percentage rps_inner;
    float rps_present;
    Percentage duty;
    // direction setpoint
    RotateState direction_setpoint;
    RotateState direction_inner;
    RotateState direction_present;

    uint8_t hall_state_last;
    uint8_t hall_state_present;
    uint16_t step_count;

    MotorState stop;
    float integral_record;
} MotorParameter;

extern MotorParameter motor_right;
extern MotorParameter motor_left;

void motor_step_update(MotorParameter *motor);
void motor_set_duty(MotorParameter *motor, uint8_t value);
void PI_control(MotorParameter *motor, float ms);
void motor_state_update(MotorParameter *motor, float ms);
void motor_set_stop(MotorParameter *motor, bool stop);
bool motor_set_speed(MotorParameter* motor, Percentage value);
void motor_set_direction(MotorParameter *motor, RotateState direction);
void motor_add_step_count(MotorParameter *motor);
