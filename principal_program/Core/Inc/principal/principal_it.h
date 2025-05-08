#ifndef PRINCIPAL_IT_H
#define PRINCIPAL_IT_H

#include "common.h"
#include "motor.h"


void principal_EXTI9_5_IRQHandler(void);
void principal_EXTI3_IRQHandler(void);
void r_exit_rpm_count(void);
void l_exit_rpm_count(void);

void TIM1_UP_TIM16_IRQHandler_use(void);
int time_rpm(MOTOR_PARAMETER *motor);



#endif
