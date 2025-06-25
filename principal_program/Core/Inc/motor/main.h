#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "main/config.h"
#include "main/map.h"
#include "gpio.h"
#include "tim.h"

typedef struct ArmConst
{
    GPIO_TypeDef* Hall_GPIOx[3];
    uint16_t Hall_GPIO_Pin_x[3];
    TIM_HandleTypeDef* TIMx[3];
    uint32_t TIM_CHANNEL_x[3];
    GPIO_TypeDef* Coil_GPIOx[3];
    uint16_t Coil_GPIO_Pin_x[3];
} ArmConst;
typedef struct ArmParameter
{
    const ArmConst* motor_const;
    uint8_t speed_sepoint_pcn;
    ROTATE_STATUS rotate_direction;
    float integral_record;
    uint16_t step_count;
    uint16_t adc_value;
    uint8_t duty_value;
    uint16_t speed_present;
    uint8_t currentStep;
} ArmParameter;
extern ArmParameter motor_right;
extern ArmParameter motor_left;

void motor_step_update(ArmParameter *motor);
void motor_speed_calculate(ArmParameter *motor, float sec);
bool motor_set_duty(ArmParameter *motor, uint8_t value);
bool motor_set_speed_setpoint(ArmParameter* motor, uint8_t value);
void motor_set_direction(ArmParameter *motor, ROTATE_STATUS direction);
void motor_set_integral_record(ArmParameter *motor, float integral);
void motor_set_adc_val(ArmParameter *motor, uint16_t value);
void motor_add_step_count(ArmParameter *motor);
