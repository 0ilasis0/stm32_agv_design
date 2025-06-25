#pragma once

#include <stdint.h>
#include "main/config.h"
#include "main/vec.h"
#include "connectivity/fdcan/trcv_buffer.h"

#define FDCAN_DEVICE_ID_MIN 0x020
#define FDCAN_DEVICE_ID_MAX 0x02F

extern bool fdacn_data_trsm_ready;

extern FDCAN_TxHeaderTypeDef fdcanTxHeader;
extern FDCAN_RxHeaderTypeDef fdcanRxHeader;

extern VecByte fdcan_tr_buf;
extern VecByte fdcan_rv_buf;

extern FdcanByteTrcvBuf fdcan_tr_pkt_buf;
extern FdcanByteTrcvBuf fdcan_rv_pkt_buf;

#define FDCAN_FilterTypeDef_DEFALT()            \
((FDCAN_FilterTypeDef){                         \
    .IdType = FDCAN_STANDARD_ID,                \
    .FilterIndex = 0,                           \
    .FilterType = FDCAN_FILTER_RANGE,           \
    .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,    \
    .FilterID1 = FDCAN_DEVICE_ID_MIN,           \
    .FilterID2 = FDCAN_DEVICE_ID_MAX,           \
})
