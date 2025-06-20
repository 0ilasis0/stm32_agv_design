#pragma once

#include <stddef.h>
#include <stdint.h>
#include "main/fn_state.h"
#include "main/config.h"
#include "main/vec.h"

typedef struct ByteTrcvBuf
{
    VecByte  vecs[UART_TRCV_BUF_CAP];
    size_t cap;
    size_t head;
    size_t len;
} ByteTrcvBuf;

FnState uart_trcv_buf_setup(ByteTrcvBuf* self, size_t buf_size, size_t data_size);
FnState uart_trcv_buf_push(ByteTrcvBuf* self, const VecByte* vec_u8);
FnState uart_trcv_buf_pop(ByteTrcvBuf* self, VecByte* vec_u8);
