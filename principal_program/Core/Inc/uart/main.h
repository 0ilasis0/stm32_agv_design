#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "usart.h"
#include "uart/packet.h"
#include "motor/main.h"

#define UART3_BAUDRATE 115200
#define UART_TIME_OUT 100

extern UartTrcvBuf uart_tr_pkt_buf;
extern UartTrcvBuf uart_rv_pkt_buf;

typedef struct {
    bool uart_transmit;
    bool uart_tr_pkt_proc;
    bool uart_re_pkt_proc;
    bool right_speed;
    bool right_adc;
} TransceiveFlags;
extern TransceiveFlags transceive_flags;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void uart_setup(void);
void uart_main(void);
