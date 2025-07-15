#pragma once

#include "spi.h"
#include "main/config.h"
#include "main/fn_state.h"
#include "rfid/MFRC522_STM32.h"
#include "main/vec.h"

#define SECTOR_MASK(s)      ((uint16_t)1 << (s))

typedef enum CardState
{
    CARD_STATE_NONE,
    CARD_STATE_EXIST,
} CardState;

typedef struct RC522State
{
    const RC522Const const_h;
    RC522Uid uid;
    uint32_t uid32;
    CardState state;
    uint16_t secter1k_open;
    uint8_t addr_0x0[RFID_BLOCK_BYTE_CAP];
    uint8_t addr_0x0_s;
} RC522State;

typedef struct RfidTrcvBuf
{
    uint8_t sector;
    uint8_t block;
    uint8_t data[18];
    uint16_t flags;
    uint8_t send;
    RC522MIFARE_Key key;
} RfidTrcvBuf;

extern RfidTrcvBuf rfid_trsm_buf;
extern RfidTrcvBuf rfid_recv_buf;
extern RC522State spi2_rfid;

FnState rfid_trcv_buf_setaddr(RfidTrcvBuf* trcv_buf, uint8_t sector, uint8_t block, uint8_t send);
FnState rfid_trcv_buf_setdata(RfidTrcvBuf* trcv_buf, uint8_t id, uint8_t *data, uint8_t len);
