#pragma once

#include "main/config.h"
#include "main/fn_state.h"
#include "main/map.h"
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
    uint8_t speed_sepoint_pcn;
    ROTATE_STATUS rotate_direction;
    float integral_record;
    uint16_t step_count;
    uint16_t adc_value;
    uint8_t duty_value;
    uint16_t speed_present;
    uint8_t currentStep;
} MotorParameter;
extern MotorParameter motor_right;
extern MotorParameter motor_left;

void motor_step_update(MotorParameter *motor);
void motor_speed_calculate(MotorParameter *motor, float sec);
FnState motor_set_duty(MotorParameter *motor, int8_t value);
bool motor_set_speed_setpoint(MotorParameter* motor, uint8_t value);
void motor_set_direction(MotorParameter *motor, ROTATE_STATUS direction);
void motor_set_integral_record(MotorParameter *motor, float integral);
void motor_set_adc_val(MotorParameter *motor, uint16_t value);
void motor_add_step_count(MotorParameter *motor);
