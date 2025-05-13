#ifndef PRINCIPAL_IT_H
#define PRINCIPAL_IT_H

#include "principal/motor.h"

void principal_EXTI9_5_IRQHandler(void);
void principal_EXTI3_IRQHandler(void);
void principal_EXTI4_IRQHandler(void);
void EXTI15_10_IRQHandler_principal_it(void);
void principal_EXTI9_5_IRQHandler(void);
void principal_EXTI3_IRQHandler(void);

void principal_TIM1_UP_TIM16_IRQHandler(void);
void speed_calculate(MOTOR_PARAMETER *motor);

#endif
