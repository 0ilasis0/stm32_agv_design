#include "connectivity/fdcan/basic.h"

FdcanPktPool fdcan_pkt_pool;

bool fdcan_bus_off = false;

FncState fdacn_data_store = FNC_DISABLE;

Result fdcan_pkt_get_byte(FdcanPkt* pkt, uint8_t id, uint8_t* container)
{
    if (pkt->len <= id) return RESULT_ERROR(RES_ERR_NOT_FOUND);
    *container = pkt->data[id];
    return RESULT_OK(container);
}

void fdcan_pkt_pool_init(void)
{
    fdcan_pkt_pool.head = NULL;
    for (size_t i = 0; i < FDCAN_PKT_POOL_CAP; i++) {
        fdcan_pkt_pool.pkt[i].next = fdcan_pkt_pool.head;
        fdcan_pkt_pool.head = &fdcan_pkt_pool.pkt[i];
    }
    fdcan_pkt_pool.remain = FDCAN_PKT_POOL_CAP;
}

Result fdcan_pkt_pool_alloc(void)
{
    if (fdcan_pkt_pool.head == NULL) {
        return RESULT_ERROR(RES_ERR_EMPTY);
    }
    FdcanPkt* pkt = fdcan_pkt_pool.head;
    fdcan_pkt_pool.head = pkt->next;
    pkt->next = NULL;
    fdcan_pkt_pool.remain--;
    return RESULT_OK(pkt);
}

void fdcan_pkt_pool_free(FdcanPkt* pkt)
{
    memset(pkt->data, 0, sizeof(pkt->data));
    pkt->len = 0;
    pkt->next = fdcan_pkt_pool.head;
    fdcan_pkt_pool.head = pkt;
    fdcan_pkt_pool.remain++;
}

static FdcanPkt* trsm_pkt_buf[FDCAN_TRSM_BUF_CAP];
FdcanPktBuf fdcan_trsm_pkt_buf = {
    .buf = trsm_pkt_buf,
    .cap = FDCAN_TRSM_BUF_CAP,
};
static FdcanPkt* recv_pkt_buf[FDCAN_RECV_BUF_CAP];
FdcanPktBuf fdcan_recv_pkt_buf = {
    .buf = recv_pkt_buf,
    .cap = FDCAN_RECV_BUF_CAP,
};

Result fdcan_pkt_buf_push(FdcanPktBuf* self, FdcanPkt* pkt)
{
    if (self->len >= self->cap) return RESULT_ERROR(RES_ERR_OVERFLOW);
    size_t tail = (self->head + self->len) % self->cap;
    self->buf[tail] = pkt;
    self->len++;
    return RESULT_OK(self);
}

Result fdcan_pkt_buf_pop(FdcanPktBuf* self)
{
    if (self->len == 0) return RESULT_ERROR(RES_ERR_EMPTY);
    size_t head = self->head;
    FdcanPkt* pkt = self->buf[head];
    if (--self->len == 0) self->head = 0;
    else self->head = (self->head + 1) % self->cap;
    return RESULT_OK(pkt);
}
