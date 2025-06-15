#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "usart.h"
#include "uart/packet.h"
#include "motor/main.h"

#define UART3_BAUDRATE 115200
#define UART_TIME_OUT 100

typedef struct {
    bool uart_transmit;
    bool uart_transmit_pkt_proc;
    bool uart_receive_pkt_proc;
    bool right_speed;
    bool right_adc;
} TransceiveFlags;
extern TransceiveFlags transceive_flags;
void USER_MX_USART3_UART_Init(void);
void uart_setup(void);
void USER_UART3_IRQHandler_Before(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void uart_main(void);
