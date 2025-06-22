/*
#include "main/config.h"
*/
#pragma once

#include <stdbool.h>

#define BOARD_LED_TOGGLE HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5)

// ! SYSTEM config, Change CAREFULLY --------------------

#define TIM1_PSC 17000
#define TIM1_ARR 5000

#define TIM2_PSC 170
#define TIM2_ARR 100

#define TIM3_PSC 170
#define TIM3_ARR 100

#define VEC_BYTE_MAX_CAPACITY 256
#define TRCV_BUF_MAX_CAPACITY 10

#define UART_BAUDRATE 115200
#define UART_VEC_MAX 128
#define UART_TRCV_BUF_CAP 10
#define UART_START_CODE  ((uint8_t) '>')
#define UART_END_CODE    ((uint8_t) '\n')

// ! SYSTEM config END ------------------------------

// #define DISABLE_FDCAN
// #define DISABLE_UART
// #define DISABLE_UART_TRSM
// #define DISABLE_UART_RECV

typedef struct{
    bool enable_PI;
    bool enable_adc;
    bool enable_search_magnetic_path;
    bool enable_timeout_error;

    bool enable_debug_breakdown_all_hall_lost;
    bool enable_debug_test_no_load_speed;
} SYSTEM_RUNTIME_SWITCH;
extern SYSTEM_RUNTIME_SWITCH sys_run_switch;
