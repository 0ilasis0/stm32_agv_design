#include "uart/trcv_buffer.h"

FnState uart_trcv_buf_setup(UartTrcvBuf* self) {
    self->head = 0;
    self->len = 0;
    for (uint8_t i = 0; i < UART_TRCV_BUF_CAP; i++)
    {
        FNS_ERROR_CHECK(vec_u8_new(&self->vecs[i], UART_VEC_MAX));
    }
    return FNS_OK;
}

/**
 * @brief 將封包推入環形緩衝區，若已滿則返回 false
 *        Push a packet into the ring buffer; return false if buffer is full
 *
 * @param self 指向環形緩衝區的指標 (input/output ring buffer)
 * @param pkt 要推入緩衝區的 UART 封包 (input UART packet)
 * @return bool 是否推入成功 (true if push successful, false if buffer full)
 */
FnState uart_trcv_buf_push(UartTrcvBuf* self, const Vec_U8* vec_u8)
{
    if (self->len >= UART_TRCV_BUF_CAP) return FNS_BUF_OVERFLOW;
    uint8_t tail = (self->head + self->len) % UART_TRCV_BUF_CAP;
    vec_u8_rm_all(&self->vecs[tail]);
    FNS_ERROR_CHECK(vec_u8_push(&self->vecs[tail], vec_u8->data, vec_u8->len));
    self->len++;
    return FNS_OK;
}

/**
 * @brief 從環形緩衝區彈出一個封包資料
 *        Pop a packet from the ring buffer
 *
 * @param self 指向環形緩衝區的指標 (input/output ring buffer)
 * @param pkt 輸出參數，接收彈出的 UART 封包 (output popped UART packet)
 * @return bool 是否彈出成功 (true if pop successful, false if buffer empty)
 */
FnState uart_trcv_buf_pop(UartTrcvBuf* self, Vec_U8* vec_u8)
{
    if (self->len == 0) return FNS_BUF_EMPTY;
    FNS_ERROR_CHECK(vec_u8_push(vec_u8, self->vecs[self->head].data, self->vecs[self->head].len));
    if (--self->len == 0)
    {
        self->head = 0;
    }
    else
    {
        self->head = (self->head + 1) % UART_TRCV_BUF_CAP;
    }
    return FNS_OK;
}
