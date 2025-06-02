#ifndef USER_UART_H
#define USER_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "usart.h"
#include "user/motor.h"

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

void uart_setup(void);
void USER_UART3_IRQHandler_Before(void);
void uart_transmit(void);

#endif
