#ifndef USER_IT_H
#define USER_IT_H

#include "user/motor.h"

void user_EXTI9_5_IRQHandler(void);
void user_EXTI3_IRQHandler(void);
void user_EXTI4_IRQHandler(void);
void EXTI15_10_IRQHandler_user_it(void);
void user_EXTI9_5_IRQHandler(void);
void user_EXTI3_IRQHandler(void);

void user_TIM1_UP_TIM16_IRQHandler(void);
void user_TIM2_IRQHandler(void);

#endif
