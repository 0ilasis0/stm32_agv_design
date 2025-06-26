/*
#include "main/config.h"
*/
#pragma once

#include <stdbool.h>
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"

#define UNUSED_FNC __attribute__((unused))
#define BOARD_LED_TOGGLE HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5)

// ! SYSTEM config, Change CAREFULLY --------------------

#define PRINCIPAL_PROGRAM

#define TIM1_PSC    17000
#define TIM1_ARR     5000
#define TIM2_PSC      170
#define TIM2_ARR      100
#define TIM3_PSC      170
#define TIM3_ARR      100

#define VEC_BYTE_MAX_CAP 256
#define TRCV_BUF_MAX_CAP 10

#define FDCAN_FILTER_COUNT  1
#define FDCAN_FILTER_ID_MIN 0x020
#define FDCAN_FILTER_ID_MAX 0x02F
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

#define error_timeout_time_limit 1000

typedef struct SYSTEM_RUNTIME_SWITCH
{
    bool enable_adc;
    bool enable_PI;
    bool enable_search_magnetic_path;
    bool enable_timeout_error;

    bool enable_debug_breakdown_all_hall_lost;
    bool enable_debug_test_no_load_speed;
} SYSTEM_RUNTIME_SWITCH;
extern SYSTEM_RUNTIME_SWITCH sys_run_switch;
