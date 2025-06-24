/*
#include "main/config.h"
*/
#pragma once

#define BOARD_LED_TOGGLE HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5)

// ! SYSTEM config, Change CAREFULLY --------------------

#define TIM2_PSC      170
#define TIM2_ARR    20000
#define TIM3_PSC      170
#define TIM3_ARR    20000

// ! SYSTEM config END ------------------------------
