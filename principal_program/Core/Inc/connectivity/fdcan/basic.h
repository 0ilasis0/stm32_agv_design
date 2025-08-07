#pragma once

#include "main/config.h"
#include "main/fn_state.h"

typedef struct FdcanPkt
{
    uint32_t id;
    uint8_t data[FDCAN_VEC_BYTE_CAP];
    uint8_t len;
    struct FdcanPkt *next;
} FdcanPkt;
Result fdcan_pkt_get_byte(FdcanPkt* pkt, uint8_t id, uint8_t* container);

typedef struct FdcanPktPool
{
    FdcanPkt    pkt[FDCAN_PKT_POOL_CAP];
    FdcanPkt*   head;
    uint8_t     remain;
} FdcanPktPool;
extern FdcanPktPool fdcan_pkt_pool;
void fdcan_pkt_pool_init(void);
Result fdcan_pkt_pool_alloc(void);
void fdcan_pkt_pool_free(FdcanPkt* pkt);

typedef struct FdcanPktBuf
{
    FdcanPkt**  buf;
    size_t      head;
    size_t      len;
    size_t      cap;
} FdcanPktBuf;
extern FdcanPktBuf fdcan_trsm_pkt_buf;
extern FdcanPktBuf fdcan_recv_pkt_buf;
Result fdcan_pkt_buf_push(FdcanPktBuf* self, FdcanPkt* pkt);
Result fdcan_pkt_buf_pop(FdcanPktBuf* self);

extern bool fdcan_bus_off;
extern FncState fdacn_data_store;
