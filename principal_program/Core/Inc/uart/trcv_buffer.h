#pragma once

#include <stdint.h>
#include "main/fn_state.h"
#include "main/config.h"
#include "main/vec.h"

typedef struct UartTrcvBuf
{
    Vec_U8  vecs[UART_TRCV_BUF_CAP];
    uint8_t head;
    uint8_t len;
} UartTrcvBuf;

FnState uart_trcv_buf_setup(UartTrcvBuf* self);
FnState uart_trcv_buf_push(UartTrcvBuf* self, const Vec_U8* vec_u8);
FnState uart_trcv_buf_pop(UartTrcvBuf* self, Vec_U8* vec_u8);
