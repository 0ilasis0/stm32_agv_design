#pragma once

#include <stdint.h>
#include "main/config.h"
#include "connectivity/fdcan/trcv_buffer.h"

extern FncState fdcan_enable;
extern FncState fdacn_data_trsm_ready;

extern FdcanByteTrcvBuf fdcan_tr_pkt_buf;
extern FdcanByteTrcvBuf fdcan_rv_pkt_buf;
