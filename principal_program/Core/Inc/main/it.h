#ifndef USER_IT_H
#define USER_IT_H

#include "motor/main.h"

void user_SysTick_Handler(void);
void user_EXTI9_5_IRQHandler(void);
void user_EXTI3_IRQHandler(void);
void user_EXTI4_IRQHandler(void);
void user_EXTI15_10_IRQHandler(void);
void user_EXTI9_5_IRQHandler(void);
void user_EXTI3_IRQHandler(void);

void user_TIM1_UP_TIM16_IRQHandler(void);

#endif
