#include "connectivity/uart/trcv_buffer.h"

FnState uart_trcv_buf_setup(ByteTrcvBuf* self, size_t buf_size, size_t data_size)
{
    self->head = 0;
    self->len = 0;
    self->cap = buf_size;
    for (size_t i = 0; i < buf_size; i++)
    {
        FNS_ERROR_CHECK(vec_u8_new(&self->vecs[i], data_size));
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
FnState uart_trcv_buf_push(ByteTrcvBuf* self, const VecByte* vec_u8)
{
    if (self->len >= self->cap) return FNS_BUF_OVERFLOW;
    size_t tail = (self->head + self->len) % self->cap;
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
FnState uart_trcv_buf_pop(ByteTrcvBuf* self, VecByte* vec_u8)
{
    if (self->len == 0) return FNS_BUF_EMPTY;
    FNS_ERROR_CHECK(vec_u8_push(vec_u8, self->vecs[self->head].data, self->vecs[self->head].len));
    if (--self->len == 0)
    {
        self->head = 0;
    }
    else
    {
        self->head = (self->head + 1) % self->cap;
    }
    return FNS_OK;
}
