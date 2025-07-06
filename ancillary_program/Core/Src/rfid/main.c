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

uint8_t data_store[16] = {0};
uint8_t date_write = 0;

RC522Status status;

uint8_t dataBuf0[18];
uint8_t dataBuf1[18];

uint8_t tttt = 0;

static UNUSED_FNC void rfid_init(void)
{
    RC522_PCD_Init(&spi2_rfid.const_h);
    if (RC522_PCD_PerformSelfTest(&spi2_rfid.const_h))
    {
        tttt = 1;
    }
}

void test(RC522State* state)
{
    if ((state->secter1k_open & SECTOR_MASK()) != 0)
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

        RC522MIFARE_Key key;
        memcpy(&key, &rc522_default_key, sizeof(RC522MIFARE_Key));
        uint8_t sector    = 1;              // 扇區號（0~15）
        uint8_t blockAddr = sector * 4;
        status = RC522_PCD_Authenticate(&spi2_rfid.const_h, PICC_CMD_MF_AUTH_KEY_A, blockAddr, &key, &rc522_uid);
        if (status != STATUS_Code_OK)
        {
            osDelay(1);
            continue;
        }
        spi2_rfid.secter1k_open |= SECTOR_MASK(sector);

        uint8_t buf_size0 = sizeof(dataBuf0);
        memset(&dataBuf0, 0, buf_size0);
        status = RC522_MIFARE_Read(&spi2_rfid.const_h, blockAddr, dataBuf0, &buf_size0);
        if (date_write)
        {
            date_write = 0;
            status = RC522_MIFARE_Write(&spi2_rfid.const_h, blockAddr, data_store, sizeof(data_store));
        }
        uint8_t buf_size1 = sizeof(dataBuf1);
        memset(&dataBuf1, 0, buf_size1);
        status = RC522_MIFARE_Read(&spi2_rfid.const_h, blockAddr, dataBuf1, &buf_size1);

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
