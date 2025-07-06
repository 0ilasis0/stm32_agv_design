#include "rfid/main.h"

RC522State spi2_rfid = {
    .const_h = {
        .hspi = &hspi2,
        .SDA_GPIOx = GPIOB,
        .SDA_GPIO_PIN_x = GPIO_PIN_2,
        .IRQ_GPIOx = GPIOB,
        .IRQ_GPIO_PIN_x = GPIO_PIN_1,
        .RST_GPIOx = GPIOA,
        .RST_GPIO_PIN_x = GPIO_PIN_9,
    },
};

RfidTrcvBuf rfid_trsm_buf = {0};
RfidTrcvBuf rfid_recv_buf = {0};

RC522Status status;

FnState rfid_trcv_buf_setaddr(RfidTrcvBuf* trcv_buf, uint8_t sector, uint8_t block, uint8_t send)
{
    if (
           sector >= 16
        || block >= 3
        || send > 2
    ) return FNS_NO_MATCH;
    trcv_buf->sector = sector;
    trcv_buf->block = block;
    trcv_buf->send = send;
    return FNS_OK;
}

FnState rfid_trcv_buf_setdata(RfidTrcvBuf* trcv_buf, uint8_t id, uint8_t *data, uint8_t len)
{
    if (id + len > 16) return FNS_OVERFLOW;
    memcpy(&trcv_buf->data[id], data, len);
    uint8_t i;
    for(i = 0; i < len; i++)
    {
        trcv_buf->flags |= ((uint16_t)1 << (id + i));
    }
    return FNS_OK;
}

static FnState secter_open(RC522State* state, RfidTrcvBuf* trcv_buf)
{
    if ((state->secter1k_open & SECTOR_MASK(trcv_buf->sector)) == 0)
    {
        if (RC522_PCD_Authenticate(&state->const_h, PICC_CMD_MF_AUTH_KEY_A, trcv_buf->sector * 4, &trcv_buf->key, &state->uid) != STATUS_Code_OK) 
            return FNS_FAIL;
        state->secter1k_open |= SECTOR_MASK(trcv_buf->sector);
    }
    return FNS_OK;
}

static UNUSED_FNC FnState buf_write(RC522State* state, RfidTrcvBuf* trcv_buf)
{
    if (trcv_buf->send == 0) return FNS_INVALID;
    ERROR_CHECK_FNS_RETURN(secter_open(state, trcv_buf));
    if (RC522_MIFARE_Write(&state->const_h, (trcv_buf->sector * 4) + trcv_buf->block, trcv_buf->data, 16) != STATUS_Code_OK)
        return FNS_FAIL;
    trcv_buf->send = 0;
    trcv_buf->flags = 0;
    return FNS_OK;
}

static UNUSED_FNC FnState buf_read(RC522State* state, RfidTrcvBuf* trcv_buf)
{
    ERROR_CHECK_FNS_RETURN(secter_open(state, trcv_buf));
    uint8_t size = 18;
    memset(trcv_buf->data, 0, size);
    if (RC522_MIFARE_Read(&state->const_h, (trcv_buf->sector * 4) + trcv_buf->block, trcv_buf->data, &size) != STATUS_Code_OK)
        return FNS_FAIL;
    return FNS_OK;
}

static UNUSED_FNC void rfid_init(void)
{
    RC522_PCD_Init(&spi2_rfid.const_h);
    memcpy(&rfid_trsm_buf.key, &rc522_default_key, sizeof(RC522MIFARE_Key));
    memcpy(&rfid_recv_buf.key, &rc522_default_key, sizeof(RC522MIFARE_Key));
    if (RC522_PCD_PerformSelfTest(&spi2_rfid.const_h))
    {
    }
}

void StartRfidTask(void *argument)
{
    rfid_init();
    for(;;)
    {
        if (
               !RC522_PICC_IsNewCardPresent(&spi2_rfid.const_h) 
            || !RC522_PICC_ReadCardSerial(&spi2_rfid.const_h)
        ) {
            osDelay(1);
            continue;
        }
        spi2_rfid.state = 1;
        spi2_rfid.secter1k_open = 0;
        memcpy(&spi2_rfid.uid, &rc522_uid, sizeof(RC522Uid));

        buf_write(&spi2_rfid, &rfid_trsm_buf);
        rfid_recv_buf.sector = rfid_trsm_buf.sector;
        rfid_recv_buf.block = rfid_trsm_buf.block;
        buf_read(&spi2_rfid, &rfid_recv_buf);

        status = RC522_PICC_HaltA(&spi2_rfid.const_h);
        if (status != STATUS_Code_OK)
        {
            osDelay(1);
            continue;
        }
        spi2_rfid.state = 0;
        spi2_rfid.secter1k_open = 0;
        RC522_PCD_StopCrypto1(&spi2_rfid.const_h);
    }
}

