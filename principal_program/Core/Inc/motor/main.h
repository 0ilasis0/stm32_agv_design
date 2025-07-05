#pragma once

#include "main/config.h"
#include "main/fn_state.h"
#include "gpio.h"
#include "tim.h"

extern float max_speed;

typedef enum {
    rotate_clockwise,
    rotate_c_clockwise,
} ROTATE_STATUS;

typedef struct MotorConst
{
    GPIO_TypeDef* Hall_GPIOx[3];
    uint16_t Hall_GPIO_Pin_x[3];
    TIM_HandleTypeDef* htimx[3];
    uint32_t TIM_CHANNEL_x[3];
    GPIO_TypeDef* Coil_GPIOx[3];
    uint16_t Coil_GPIO_Pin_x[3];
} MotorConst;

typedef struct MotorParameter
{
    const MotorConst* const_h;
    // RPS setpoint
    Percentage rps_sepoint;
    // real RPS setpoint (control by system)
    Percentage rps_setpoint_inner;
    uint16_t step_count;
    float rps_present;
    // direction setpoint
    ROTATE_STATUS direction_setpoint;
    ROTATE_STATUS direction_present;
    Percentage duty;
    float integral_record;
    uint8_t current_step;
} MotorParameter;

extern MotorParameter motor_right;
extern MotorParameter motor_left;

void motor_step_update(MotorParameter *motor);
void motor_rps_calculate(MotorParameter *motor, float sec);
FnState motor_set_duty(MotorParameter *motor, uint8_t value);
bool motor_set_speed(MotorParameter* motor, Percentage value);
void motor_set_direction(MotorParameter *motor, ROTATE_STATUS direction);
void motor_add_step_count(MotorParameter *motor);
