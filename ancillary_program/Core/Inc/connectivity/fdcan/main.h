#pragma once

#include <stdint.h>
#include "main/config.h"
#include "connectivity/fdcan/trcv_buffer.h"

extern FncState fdcan_enable;
extern FncState fdacn_data_trsm_ready;

extern FdcanByteTrcvBuf fdcan_tr_pkt_buf;
extern FdcanByteTrcvBuf fdcan_rv_pkt_buf;

#define FDCAN_FilterTypeDef_DEFALT()            \
((FDCAN_FilterTypeDef){                         \
    .IdType = FDCAN_STANDARD_ID,                \
    .FilterIndex = 0,                           \
    .FilterType = FDCAN_FILTER_RANGE,           \
    .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,    \
    .FilterID1 = FDCAN_FILTER_ID_MIN,           \
    .FilterID2 = FDCAN_FILTER_ID_MAX,           \
})
