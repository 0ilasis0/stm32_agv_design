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

RC522Status status;
uint8_t dataBuf0[18];
uint8_t dataBuf1[18];
uint8_t dataBuf2[16] = {
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
};

uint8_t tttt = 0;

void StartRfidTask(void *argument)
{
    RC522_PCD_Init(&rfid_const);
    if (RC522_PCD_PerformSelfTest(&rfid_const))
    {
        tttt = 1;
    }

    for(;;)
    {
        if (
               !RC522_PICC_IsNewCardPresent(&rfid_const) 
            || !RC522_PICC_ReadCardSerial(&rfid_const)
        ) {
            osDelay(1);
            continue;
        }

        RC522MIFARE_Key key;
        memcpy(&key, &rc522_default_key, sizeof(RC522MIFARE_Key));
        uint8_t sector    = 1;              // 扇區號（0~15）
        uint8_t blockAddr = sector * 4;
        status = RC522_PCD_Authenticate(&rfid_const, PICC_CMD_MF_AUTH_KEY_A, blockAddr, &key, &rc522_uid);
        if (status != STATUS_Code_OK)
        {
            osDelay(1);
            continue;
        }
        uint8_t buf_size0 = sizeof(dataBuf0);
        memset(&dataBuf0, 0, buf_size0);
        status = RC522_MIFARE_Read(&rfid_const, blockAddr, dataBuf0, &buf_size0);
        dataBuf2[3]++;
        status = RC522_MIFARE_Write(&rfid_const, blockAddr, dataBuf2, sizeof(dataBuf2));
        uint8_t buf_size1 = sizeof(dataBuf1);
        memset(&dataBuf1, 0, buf_size1);
        status = RC522_MIFARE_Read(&rfid_const, blockAddr, dataBuf1, &buf_size1);

        status = RC522_PICC_HaltA(&rfid_const);
        if (status != STATUS_Code_OK)
        {
            osDelay(1);
            continue;
        }
        RC522_PCD_StopCrypto1(&rfid_const);
    }
}
