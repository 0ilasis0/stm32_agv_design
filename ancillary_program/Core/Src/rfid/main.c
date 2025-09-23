#include "rfid/main.h"
#include "connectivity/fdcan/main.h"
#include "map/simple.h"

bool new_card = 0;

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

Result rfid_trcv_buf_setaddr(RfidTrcvBuf* trcv_buf, uint8_t sector, uint8_t block, uint8_t send)
{
    if (
           sector >= 16
        || block >= 3
        || send > 2
    ) return RESULT_ERROR(RES_ERR_NOT_FOUND);
    trcv_buf->sector = sector;
    trcv_buf->block = block;
    trcv_buf->send = send;
    return RESULT_OK(NULL);
}

Result rfid_trcv_buf_setdata(RfidTrcvBuf* trcv_buf, uint8_t id, uint8_t *data, uint8_t len)
{
    if (id + len > 16) return RESULT_ERROR(RES_ERR_OVERFLOW);
    memcpy(&trcv_buf->data[id], data, len);
    uint8_t i;
    for(i = 0; i < len; i++)
    {
        trcv_buf->flags |= ((uint16_t)1 << (id + i));
    }
    return RESULT_OK(NULL);
}

static UNUSED_FNC Result buf_write(RC522State* state, RfidTrcvBuf* trcv_buf)
{
    if (trcv_buf->send == 0) return RESULT_ERROR(RES_ERR_INVALID);
    RC522_PCD_Authenticate(&state->const_h, PICC_CMD_MF_AUTH_KEY_A, trcv_buf->sector*4, &trcv_buf->key, &state->uid);
    if (RC522_MIFARE_Write(&state->const_h, (trcv_buf->sector * 4) + trcv_buf->block, trcv_buf->data, 16) != STATUS_Code_OK)
        return RESULT_ERROR(RES_ERR_FAIL);
    trcv_buf->send = 0;
    trcv_buf->flags = 0;
    return RESULT_OK(NULL);
}

static UNUSED_FNC Result buf_read(RC522State* state, RfidTrcvBuf* trcv_buf)
{
    RC522_PCD_Authenticate(&state->const_h, PICC_CMD_MF_AUTH_KEY_A, trcv_buf->sector*4, &trcv_buf->key, &state->uid);
    trcv_buf->size = 18;
    memset(trcv_buf->data, 0, trcv_buf->size);
    if (RC522_MIFARE_Read(&state->const_h, (trcv_buf->sector * 4) + trcv_buf->block, trcv_buf->data, &trcv_buf->size) != STATUS_Code_OK)
        return RESULT_ERROR(RES_ERR_FAIL);
    return RESULT_OK(NULL);
}

uint32_t defalt_running;
void StartRfidTask(void *argument)
{
    RC522_PCD_Init(&spi2_rfid.const_h);
    memcpy(&rfid_trsm_buf.key, &rc522_default_key, sizeof(RC522MIFARE_Key));
    memcpy(&rfid_recv_buf.key, &rc522_default_key, sizeof(RC522MIFARE_Key));
    // if (RC522_PCD_PerformSelfTest(&spi2_rfid.const_h)) {};
    // Result result;
    for(;;)
    {
        defalt_running++;
        switch (spi2_rfid.state)
        {
            case CARD_STATE_NONE:
            {
                if (
                       !RC522_PICC_IsNewCardPresent(&spi2_rfid.const_h)
                    || !RC522_PICC_ReadCardSerial(&spi2_rfid.const_h)
                ) break;
                spi2_rfid.state = CARD_STATE_EXIST;
                spi2_rfid.err_count = 0;
                memcpy(&spi2_rfid.uid, &rc522_uid, sizeof(RC522Uid));
                spi2_rfid.uid32 =
                      ((uint32_t)spi2_rfid.uid.uidByte[0] << 24)
                    | ((uint32_t)spi2_rfid.uid.uidByte[1] << 16)
                    | ((uint32_t)spi2_rfid.uid.uidByte[2] <<  8)
                    | ((uint32_t)spi2_rfid.uid.uidByte[3]      );
                new_card = 1;
                RC522_PICC_HaltA(&spi2_rfid.const_h);
                // simple_point_select(spi2_rfid.uid32);
                // VecByte vec_byte;
                // RESULT_CHECK_CLEANUP(vec_byte_new(&vec_byte, FDCAN_VEC_BYTE_CAP));
                // RESULT_CHECK_CLEANUP(pkt_map_info(&vec_byte, spi2_rfid.uid32, 0));
                // RESULT_CHECK_CLEANUP(fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, 0x11));
                // cleanup:
                // vec_byte_free(&vec_byte);
                break;
            }
            case CARD_STATE_EXIST:
            {
                uint8_t atqa_answer[2];
	            uint8_t atqa_size = 2;
                if (RC522_PICC_WakeupA(&spi2_rfid.const_h, atqa_answer, &atqa_size) != STATUS_Code_OK)
                {
                    if (++spi2_rfid.err_count > 9)
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
                RC522_PICC_HaltA(&spi2_rfid.const_h);
                break;
            }
            default: break;
        }
        osDelay(50);
    }
}
