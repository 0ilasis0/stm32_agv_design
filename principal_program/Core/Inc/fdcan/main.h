#pragma once

#include <stdint.h>
#include "stm32g431xx.h"
#include "main/vec.h"

#define FDCAN_DEVICE_ID 0x321

#define FDCAN_FilterTypeDef_DEFALT() ((FDCAN_FilterTypeDef){ \
    .IdType = FDCAN_STANDARD_ID, \
    .FilterIndex = 0, \
    .FilterType = FDCAN_FILTER_RANGE, \
    .FilterConfig = FDCAN_FILTER_TO_RXFIFO0, \
    .FilterID1 = FDCAN_DEVICE_ID, \
    .FilterID2 = 0x7FF, \
})

void user_MX_FDCAN1_Init(void);
void fdcan_setup(void);
void fdcan_transmit(void);
