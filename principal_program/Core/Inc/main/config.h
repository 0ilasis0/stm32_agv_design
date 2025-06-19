/*
#include "main/config.h"
*/
#pragma once

#include <stdbool.h>

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

