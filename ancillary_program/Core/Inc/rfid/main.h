#pragma once

#include "spi.h"
#include "main/config.h"
#include "main/fn_state.h"
#include "rfid/MFRC522_STM32.h"
#include "main/vec.h"

typedef struct RC522State
{
    const RC522Const* const_h;
    RC522Uid uid;
    uint32_t secter_open;
    uint8_t state;
} RC522State;

extern const RC522Const rfid_const;
extern uint8_t data_store[];
extern uint8_t date_write;
