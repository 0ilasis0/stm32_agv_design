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
#include "FreeRTOS.h"
#include "task.h"
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
#define MOTOR_PI_KP             0.25f       // 比例增益
#define MOTOR_PI_KI             0.001f      // 積分增益
#define MOTOR_STOP_GATE         0.1f

#define ADC_COUNT                   4
#define ADC_NEED_LEN               30
#define ADC_MAGNETIC_STRIPE_VALUE 1850
#define ADC_STRONG_MAGNET_VALUE   1650

#define US_SENSOR_HTIM          &htim4
#define US_SENSOR_TIM_CH        TIM_CHANNEL_1
#define US_SENSOR_TIM_ACT_CH    HAL_TIM_ACTIVE_CHANNEL_1

#define VEHICLE_setpoint_straight  40   // 循跡速度目標
#define VEHICLE_setpoint_rotate    30   // 原地旋轉速度目標
#define VEHICLE_setpoint_fall_back 20   // 倒退速度目標

#define VEC_BYTE_MAX_CAP        256
#define TRCV_BUF_MAX_CAP        10

#define FDCAN_FILTER_COUNT      2
#define FDCAN_FILTER0_ID_MIN    0x020
#define FDCAN_FILTER0_ID_MAX    0x021
#define FDCAN_FILTER1_ID_MIN    0x022
#define FDCAN_FILTER1_ID_MAX    0x023
#define FDCAN_VEC_BYTE_CAP      8
#define FDCAN_TRCV_BUF_CAP      10
#define FDCAN_TEST_ID           0x001
#define FDCAN_MOTOR_DATA_ID     0x012

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

typedef struct RuntimeSwitch
{
    FncState adc;
    FncState rps_control;
    FncState search_magnetic_path;
    FncState timeout;

    // FncState debug_breakdown_all_hall_lost;
    FncState debug_protect_over_hall;
    FncState debug_test_no_load_speed;
} RuntimeSwitch;
extern RuntimeSwitch runtime_switch;
