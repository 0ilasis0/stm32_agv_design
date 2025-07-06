#pragma once

#include "main/config.h"
#include "main/fn_state.h"
#include "rfid/MFRC522_STM32.h"

typedef struct RfidTrcvBuf
{
    uint8_t sector;
    uint8_t block;
    uint8_t data[18];
    uint8_t flags;
} RfidTrcvBuf;

