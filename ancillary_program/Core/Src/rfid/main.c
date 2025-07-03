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

RC522Uid uid_test;
void rc522_setup(void)
{
    RC522_PCD_Init(&rfid_const);
    if (!RC522_PCD_PerformSelfTest(&rfid_const))
    {
    }
}

void rc522_main(void)
{
    if (
           RC522_PICC_IsNewCardPresent(&rfid_const) 
        && RC522_PICC_ReadCardSerial(&rfid_const)
    ) {
        size_t i;
        for (i = 0; i < uid.size; i++)
        {
            if (uid.uidByte[i] != uid_test.uidByte[i]) {
                memcpy(&uid_test, &uid, sizeof(RC522Uid));
            }
        }
    }
}
