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

FnState rfid_trcv_buf_setaddr(RfidTrcvBuf* trcv_buf, uint8_t sector, uint8_t block, uint8_t send)
{
    if (
           sector >= 16
        || block >= 3
        || send > 2
    ) return FNS_NOT_FOUND;
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

static UNUSED_FNC FnState buf_write(RC522State* state, RfidTrcvBuf* trcv_buf)
{
    if (trcv_buf->send == 0) return FNS_INVALID;
    RC522_PCD_Authenticate(&state->const_h, PICC_CMD_MF_AUTH_KEY_A, trcv_buf->sector*4, &trcv_buf->key, &state->uid);
    if (RC522_MIFARE_Write(&state->const_h, (trcv_buf->sector * 4) + trcv_buf->block, trcv_buf->data, 16) != STATUS_Code_OK)
        return FNS_FAIL;
    trcv_buf->send = 0;
    trcv_buf->flags = 0;
    return FNS_OK;
}

static UNUSED_FNC FnState buf_read(RC522State* state, RfidTrcvBuf* trcv_buf)
{
    RC522_PCD_Authenticate(&state->const_h, PICC_CMD_MF_AUTH_KEY_A, trcv_buf->sector*4, &trcv_buf->key, &state->uid);
    trcv_buf->size = 18;
    memset(trcv_buf->data, 0, trcv_buf->size);
    if (RC522_MIFARE_Read(&state->const_h, (trcv_buf->sector * 4) + trcv_buf->block, trcv_buf->data, &trcv_buf->size) != STATUS_Code_OK)
        return FNS_FAIL;
    return FNS_OK;
}

void StartRfidTask(void *argument)
{
    RC522_PCD_Init(&spi2_rfid.const_h);
    memcpy(&rfid_trsm_buf.key, &rc522_default_key, sizeof(RC522MIFARE_Key));
    memcpy(&rfid_recv_buf.key, &rc522_default_key, sizeof(RC522MIFARE_Key));
    if (RC522_PCD_PerformSelfTest(&spi2_rfid.const_h))
    {
    }
    for(;;)
    {
        switch (spi2_rfid.state)
        {
            case CARD_STATE_NONE:
            {
                if (
                       !RC522_PICC_IsNewCardPresent(&spi2_rfid.const_h)
                    || !RC522_PICC_ReadCardSerial(&spi2_rfid.const_h)
                ) break;
                spi2_rfid.state = CARD_STATE_EXIST_T;
                spi2_rfid.err_count = 0;
                memcpy(&spi2_rfid.uid, &rc522_uid, sizeof(RC522Uid));
                spi2_rfid.uid32 =
                      ((uint32_t)spi2_rfid.uid.uidByte[0] << 24)
                    | ((uint32_t)spi2_rfid.uid.uidByte[1] << 16)
                    | ((uint32_t)spi2_rfid.uid.uidByte[2] <<  8)
                    | ((uint32_t)spi2_rfid.uid.uidByte[3]      );
                break;
            }
            case CARD_STATE_EXIST_T:
            {
                spi2_rfid.state = CARD_STATE_EXIST;
                break;
            }
            case CARD_STATE_EXIST:
            {
                uint8_t atqa_answer[2];
	            uint8_t atqa_size = 2;
	            RC522_PICC_WakeupA(&spi2_rfid.const_h, atqa_answer, &atqa_size);
                RC522_PCD_Authenticate(&spi2_rfid.const_h, PICC_CMD_MF_AUTH_KEY_A, 0, &rc522_default_key, &spi2_rfid.uid);
                uint8_t buffer[18];
                uint8_t size = sizeof(buffer);
                if (RC522_MIFARE_Read(&spi2_rfid, 0, buffer, &size) != STATUS_Code_OK)
                {
                    spi2_rfid.err_count++;
                    if (spi2_rfid.err_count > 3)
                    {
                        RC522_PICC_HaltA(&spi2_rfid.const_h);
                        RC522_PCD_StopCrypto1(&spi2_rfid.const_h);
                        spi2_rfid.state = CARD_STATE_NONE;
                    }
                    break;
                }
                spi2_rfid.err_count = 0;
                buf_write(&spi2_rfid, &rfid_trsm_buf);
                rfid_recv_buf.sector = rfid_trsm_buf.sector;
                rfid_recv_buf.block  = rfid_trsm_buf.block;
                buf_read(&spi2_rfid, &rfid_recv_buf);
                // osDelay(50);
                break;
            }
            default: break;
        }
        osDelay(50);
    }
}

            // buf_write(&spi2_rfid, &rfid_trsm_buf);
            // rfid_recv_buf.sector = rfid_trsm_buf.sector;
            // rfid_recv_buf.block = rfid_trsm_buf.block;
            // buf_read(&spi2_rfid, &rfid_recv_buf);