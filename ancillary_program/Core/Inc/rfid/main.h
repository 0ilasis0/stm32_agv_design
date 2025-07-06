#pragma once

#include "spi.h"
#include "main/config.h"
#include "main/fn_state.h"
#include "rfid/MFRC522_STM32.h"
#include "main/vec.h"

extern const RC522Const rfid_const;
extern uint8_t data_store[];
extern uint8_t date_write;
