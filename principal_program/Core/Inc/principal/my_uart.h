#ifndef MY_UART_H
#define MY_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <usart.h>
#include <stm32g431xx.h>
#include <stm32g4xx_hal_uart.h>
#include "principal/packet.h"


#define UART3_BAUDRATE 115200

#define UART_TIME_OUT 100

typedef struct {
    uint8_t start;
    uint8_t data[8];
    uint8_t end;
} UART_BUFFER;

void uartInit(void);
void HYCodes_UART3_IRQHandler_Before(void);
void uart_RxCNDTR_reset(UART_HandleTypeDef *huart);

#endif
