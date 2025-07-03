#pragma once

#include "spi.h"
#include "main/config.h"
#include "main/fn_state.h"
#include "rfid/MFRC522_STM32.h"

extern const RC522Const rfid_const;

void rc522_setup(void);
void rc522_main(void);
