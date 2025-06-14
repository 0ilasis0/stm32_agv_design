#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"
#include "stm32g431xx.h"
#include "main/map.h"

typedef struct MOTOR_PARAMETER {
    uint8_t speed_sepoint_pcn;
    ROTATE_STATUS rotate_direction;
    float integral_record;
    uint16_t step_count;
    uint16_t adc_value;
    uint8_t duty_value;
    float speed_present;
    uint8_t currentStep;

    GPIO_TypeDef* Hall_GPIOx[3];
    uint16_t Hall_GPIO_Pin_x[3];

    GPIO_TypeDef* M_GPIOx[3];
    uint16_t M_GPIO_Pin_x[3];

    TIM_HandleTypeDef* TIMx[3];
    uint32_t TIM_CHANNEL_x[3];

} MOTOR_PARAMETER;
extern MOTOR_PARAMETER motor_right;
extern MOTOR_PARAMETER motor_left;

void motor_setup(void);
void motor_step_update(MOTOR_PARAMETER *motor);
void motor_speed_calculate(MOTOR_PARAMETER *motor);
bool motor_set_duty(MOTOR_PARAMETER *motor, uint8_t value);
bool motor_set_speed_setpoint(MOTOR_PARAMETER* motor, uint8_t value);
void motor_set_direction(MOTOR_PARAMETER *motor, ROTATE_STATUS direction);
void motor_set_integral_record(MOTOR_PARAMETER *motor, float integral);
void motor_set_adc_val(MOTOR_PARAMETER *motor, uint16_t value);
void motor_add_step_count(MOTOR_PARAMETER *motor);
