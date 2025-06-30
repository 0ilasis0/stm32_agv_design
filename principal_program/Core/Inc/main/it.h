#pragma once

#include "motor/main.h"
#include "main/config.h"

void user_EXTI15_10_IRQHandler(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
