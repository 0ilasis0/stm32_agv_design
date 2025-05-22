#ifndef USER_UART_H
#define USER_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "usart.h"

#define UART3_BAUDRATE 115200
#define UART_TIME_OUT 100

typedef struct {
    bool right_speed;
    bool right_adc;
} DataSendTrigger;

void uart_setup(void);
void USER_UART3_IRQHandler_Before(void);
void hysendtest(void);
void hyrecvtest(void);

#endif
