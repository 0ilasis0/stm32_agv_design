#ifndef USER_MOTOR_H
#define USER_MOTOR_H

#include <stdint.h>
#include "gpio.h"
#include "stm32g431xx.h"

typedef enum {
    clockwise,
    counter_clockwise,
    either
} ROTATE_STATUS;

typedef struct{
    uint8_t speed_sepoint;
    ROTATE_STATUS rotate_direction;
    double integral_record;
    uint16_t step_count;
    uint16_t adc_value;
    uint8_t duty_value;
    float present_speed;
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
MOTOR_PARAMETER motor_new(
    // range 1~100
    uint8_t speed_sepoint,
    ROTATE_STATUS rotate_direction,

    double integral_record,
    uint16_t step_count,
    uint16_t adc_value,
    // range 1~100
    uint8_t duty_value,
    float present_speed,
    uint8_t currentStep,

    GPIO_TypeDef* Hall_GPIOx[3],
    uint16_t Hall_GPIO_Pin_x[3],

    GPIO_TypeDef* M_GPIOx[3],
    uint16_t M_GPIO_Pin_x[3],

    TIM_HandleTypeDef* TIMx[3],
    uint32_t TIM_CHANNEL_x[3]
);
void motor_setup(void);
void motor_tim_setup(const MOTOR_PARAMETER *motor);
void update_motor_step(MOTOR_PARAMETER *motor);
void commutate_motor(const MOTOR_PARAMETER *motor);
void set_motor_duty(MOTOR_PARAMETER *motor, int16_t value);
void speed_calculate(MOTOR_PARAMETER *motor);

#endif
