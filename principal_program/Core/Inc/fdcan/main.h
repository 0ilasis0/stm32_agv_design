#pragma once

#include <stdint.h>
#include "stm32g431xx.h"

#define FDCAN_DEVICE_ID 0x321

void user_MX_FDCAN1_Init(void);
void fdcan_setup(void);
void fdcan_transmit(void);
