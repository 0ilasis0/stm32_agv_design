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

#define ANCILLARY_PROGRAM

#define TIM2_PSC        1700
#define TIM2_ARR        2000
#define TIM3_PSC        170
#define TIM3_ARR        20000

#define VEC_BYTE_MAX_CAP        256
#define TRCV_BUF_MAX_CAP        10

#define VEHICLE_SETPOINT_TRACK     40   // 循跡速度目標
#define VEHICLE_SETPOINT_ROTATE    30   // 原地旋轉速度目標
#define VEHICLE_SETPOINT_FALL_BACK 20   // 倒退速度目標
#define VEHICLE_SETPOINT_STOP      0    // 倒退速度目標

#define FDCAN_FILTER_COUNT      2
#define FDCAN_FILTER0_ID_MIN    0x030
#define FDCAN_FILTER0_ID_MAX    0x031
#define FDCAN_FILTER1_ID_MIN    0x032
#define FDCAN_FILTER1_ID_MAX    0x033
#define FDCAN_VEC_BYTE_CAP      8
#define FDCAN_TRCV_BUF_CAP      10
#define FDCAN_TEST_ID           0x002
#define FDCAN_ARM_DATA_ID       0x013

#define UART_BAUDRATE           115200
#define UART_VEC_BYTE_CAP       128
#define UART_TRCV_BUF_CAP       10
#define UART_START_CODE         ((uint8_t) '>')
#define UART_END_CODE           ((uint8_t) '\n')

#define SPI2_RFID_SDA
#define SPI2_RFID_SCK
#define SPI2_RFID_MOSI
#define SPI2_RFID_MISO
#define SPI2_RFID_IRQ
#define SPI2_RFID_RST

// #define ENABLE_CON_PKT_TEST
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
