/*
#include "main/config.h"
*/
#pragma once

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "cmsis_os.h"

#define UNUSED_FNC __attribute__((unused))
#define BOARD_LED_TOGGLE HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5)
#define ITS_CHECK(its, tag)  (((its) & (tag)) != RESET)

// ! SYSTEM config, Change CAREFULLY --------------------

#define PRINCIPAL_PROGRAM

#define MOTOR_PSC               170
#define MOTOR_ARR               100
#define TIM1_PSC                17000
#define TIM1_ARR                5000
#define TIM2_PSC                MOTOR_PSC   // 170
#define TIM2_ARR                MOTOR_ARR   // 100 MAX 2147483647
#define TIM3_PSC                MOTOR_PSC   // 170
#define TIM3_ARR                MOTOR_ARR   // 100 MAX 65535
#define TIM4_PSC                170
#define TIM4_ARR                65535       // MAX 65535
#define TIM4_CH1_CCR            10          // 10us

#define MOTOR_HTIM1             &htim2
#define MOTOR_HTIM2             &htim3
#define MOTOR_MAX_SPEED         100
#define MOTOR_PI_KP             0.25f    // 比例增益
#define MOTOR_PI_KI             0.001f  // 積分增益

#define ADC_COUNT                   4
#define ADC_NEED_LEN                9
#define ADC_MAGNETIC_STRIPE_VALUE 1700
#define ADC_STRONG_MAGNET_VALUE   1550

#define US_SENSOR_HTIM          &htim4
#define US_SENSOR_TIM_CH        TIM_CHANNEL_1
#define US_SENSOR_TIM_ACT_CH    HAL_TIM_ACTIVE_CHANNEL_1

#define VEHICLE_setpoint_straight  40   // 循跡速度目標
#define VEHICLE_setpoint_rotate    30   // 原地旋轉速度目標
#define VEHICLE_setpoint_fall_back 20   // 倒退速度目標

#define VEC_BYTE_MAX_CAP        256
#define TRCV_BUF_MAX_CAP        10

#define FDCAN_FILTER_COUNT      2
#define FDCAN_FILTER_ID_MIN     0x020
#define FDCAN_FILTER_ID_MAX     0x02F
#define FDCAN_VEC_BYTE_CAP      8
#define FDCAN_TRCV_BUF_CAP      10
#define FDCAN_TEST_ID           0x01
#define FDCAN_MOTOR_DATA_ID     0x01

#define UART_BAUDRATE           115200
#define UART_VEC_BYTE_CAP       128
#define UART_TRCV_BUF_CAP       10
#define UART_START_CODE         ((uint8_t) '>')
#define UART_END_CODE           ((uint8_t) '\n')

#define ENABLE_CON_PKT_TEST
// #define DISABLE_FDCAN
#define DISABLE_UART
// #define DISABLE_UART_TRSM
// #define DISABLE_UART_RECV

// ! SYSTEM config END ------------------------------

typedef uint8_t Percentage;

typedef int8_t FncState;
#define FNC_CANCEL  -1
#define FNC_DISABLE 0
#define FNC_ENABLE  1

typedef struct SYSTEM_RUNTIME_SWITCH
{
    bool enable_adc;
    bool enable_PI;
    bool enable_search_magnetic_path;
    bool enable_timeout_error;

    bool enable_debug_breakdown_all_hall_lost;
    bool enable_debug_protect_over_hall;
    bool enable_debug_test_no_load_speed;
} SYSTEM_RUNTIME_SWITCH;
extern SYSTEM_RUNTIME_SWITCH sys_run_switch;
