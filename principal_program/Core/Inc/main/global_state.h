#pragma once

#include "main/vec.h"
#include "uart/main.h"
#include "uart/trcv_buffer.h"

typedef struct GlobalState {
    UartTrcvBuf* uart_tr_pkt_buf_h;
    UartTrcvBuf* uart_rv_pkt_buf_h;
    TransceiveFlags* transceive_flags_h;
} GlobalState;
extern GlobalState global_state;
