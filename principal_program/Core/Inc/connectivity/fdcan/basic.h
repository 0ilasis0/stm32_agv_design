#pragma once

#include "main/config.h"
#include "connectivity/fdcan/trcv_buffer.h"

extern bool fdcan_bus_off;
extern FncState fdacn_data_trsm_ready;

extern FDCAN_TxHeaderTypeDef fdcan_TxHeader;
extern FDCAN_RxHeaderTypeDef fdcan_RxHeader;

extern VecByte fdcan_trsm_buf;
extern VecByte fdcan_recv0_buf;
extern VecByte fdcan_recv1_buf;

extern FdcanByteTrcvBuf fdcan_trsm_pkt_buf;
extern FdcanByteTrcvBuf fdcan_recv_pkt_buf;
