#pragma once

#include "spi.h"
#include "main/config.h"
#include "main/fn_state.h"
#include "rfid/MFRC522_STM32.h"
#include "main/vec.h"

#define SECTOR_MASK(s)      ((uint16_t)1 << (s))

typedef struct RC522State
{
    const RC522Const const_h;
    RC522Uid uid;
    uint8_t state;
    uint16_t secter1k_open;
} RC522State;

extern uint8_t data_store[];
extern uint8_t date_write;
