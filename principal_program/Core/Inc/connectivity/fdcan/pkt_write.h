#pragma once

#include "main/config.h"
#include "main/fn_state.h"
#include "connectivity/fdcan/basic.h"

typedef enum DataType
{
    DATA_TYPE_TEST,
    DATA_TYPE_LEFT_SPEED,
    DATA_TYPE_LEFT_DUTY,
    DATA_TYPE_RIGHT_SPEED,
    DATA_TYPE_RIGHT_DUTY,
} DataType;

void fdcan_pkt_write(FdcanPkt* pkt, DataType type);
