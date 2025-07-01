#pragma once

#include "main/config.h"
#include "connectivity/fdcan/trcv_buffer.h"

extern FncState fdcan_enable;
extern FncState fdacn_data_trsm_ready;

extern FdcanByteTrcvBuf fdcan_trsm_pkt_buf;
extern FdcanByteTrcvBuf fdcan_recv_pkt_buf;
