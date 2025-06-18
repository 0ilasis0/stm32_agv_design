#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "usart.h"
#include "uart/packet.h"
#include "uart/trcv_buffer.h"
#include "motor/main.h"

#define UART3_BAUDRATE 115200
#define UART_TIME_OUT 100

extern UartTrcvBuf uart_tr_pkt_buf;
extern UartTrcvBuf uart_rv_pkt_buf;

typedef struct {
    bool right_speed;
    bool right_adc;
} TransceiveFlags;
extern TransceiveFlags transceive_flags;
