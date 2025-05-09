#ifndef PRINCIPAL_IT_H
#define PRINCIPAL_IT_H

#include "principal/motor.h"

void principal_EXTI9_5_IRQHandler(void);
void principal_EXTI3_IRQHandler(void);

void principal_TIM1_UP_TIM16_IRQHandler(void);
void time_rpm(MOTOR_PARAMETER *motor);

#endif
