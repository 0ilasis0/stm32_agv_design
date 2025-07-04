#include "rfid/main.h"

const RC522Const rfid_const = {
    .hspi = &hspi2,
    .SDA_GPIOx = GPIOB,
    .SDA_GPIO_PIN_x = GPIO_PIN_2,
    .IRQ_GPIOx = GPIOB,
    .IRQ_GPIO_PIN_x = GPIO_PIN_1,
    .RST_GPIOx = GPIOA,
    .RST_GPIO_PIN_x = GPIO_PIN_9,
};

StatusCode status;
uint8_t dataBuf[18];
RC522Uid uid_test;

void rc522_setup(void)
{
    RC522_PCD_Init(&rfid_const);
    if (!RC522_PCD_PerformSelfTest(&rfid_const))
    {
    }
}

void StartRfidTask(void *argument)
{
    rc522_setup();
    size_t i;
    bool same_card = true;
    for(;;)
    {
        if (
               !RC522_PICC_IsNewCardPresent(&rfid_const) 
            || !RC522_PICC_ReadCardSerial(&rfid_const)
        ) {
            osDelay(1);
            continue;
        }
        for (i = 0; i < rc522_uid.size; i++)
        {
            if (rc522_uid.uidByte[i] != uid_test.uidByte[i])
            {
                same_card = false;
                memcpy(&uid_test, &rc522_uid, sizeof(RC522Uid));
            }
        }
        if (same_card)
        {
            osDelay(1);
            continue;
        }
        
        uint8_t sector    = 1;               // 扇區號（0~15）
        uint8_t blockAddr = sector * 4;      // 該扇區的第一個塊
        uint8_t buf_size = sizeof(dataBuf);
        memset(&dataBuf, 0, buf_size);
        RC522MIFARE_Key key;
        memcpy(&key, &defaultKey, sizeof(RC522MIFARE_Key));
        status = RC522_PCD_Authenticate(&rfid_const, PICC_CMD_MF_AUTH_KEY_A, blockAddr, &key, &rc522_uid);
        if (status != STATUS_Code_OK)
        {
            osDelay(1);
            continue;
        }
        status = RC522_MIFARE_Read(&rfid_const, blockAddr, dataBuf, &buf_size);
        if (status != STATUS_Code_OK)
        {
            osDelay(1);
            continue;
        }
        osDelay(1);
        RC522_PICC_HaltA(&rfid_const);
        RC522_PCD_StopCrypto1(&rfid_const);
        osDelay(1);
    }
}
