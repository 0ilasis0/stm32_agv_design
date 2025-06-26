/*
#include "main/config.h"
*/
#pragma once

#include <stdint.h>
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"

#define UNUSED_FUNC __attribute__((unused))
#define BOARD_LED_TOGGLE HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5)

// ! SYSTEM config, Change CAREFULLY --------------------

#define TIM2_PSC      170
#define TIM2_ARR    20000
#define TIM3_PSC      170
#define TIM3_ARR    20000

#define VEC_BYTE_MAX_CAP 256
#define TRCV_BUF_MAX_CAP 10

#define FDCAN_FILTER_ID_MIN 0x030
#define FDCAN_FILTER_ID_MAX 0x03F
#define FDCAN_VEC_BYTE_CAP  8
#define FDCAN_TRCV_BUF_CAP  10

#define UART_BAUDRATE       115200
#define UART_VEC_BYTE_CAP   128
#define UART_TRCV_BUF_CAP   10
#define UART_START_CODE     ((uint8_t) '>')
#define UART_END_CODE       ((uint8_t) '\n')

// ! SYSTEM config END ------------------------------

typedef int8_t FncState;
#define FNC_CANCEL  -1
#define FNC_DISABLE 0
#define FNC_ENABLE  1

#define ENABLE_CON_PKT_TEST
// #define DISABLE_FDCAN
#define DISABLE_UART
// #define DISABLE_UART_TRSM
// #define DISABLE_UART_RECV
