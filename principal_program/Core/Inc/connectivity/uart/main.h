#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "usart.h"
#include "main/fn_state.h"
#include "main/config.h"
#include "connectivity/trcv_buffer.h"
#include "motor/main.h"

extern ByteTrcvBuf uart_tr_pkt_buf;
extern ByteTrcvBuf uart_rv_pkt_buf;

typedef struct
{
    bool right_speed;
    bool right_adc;
} TransceiveFlags;
extern TransceiveFlags transceive_flags;
