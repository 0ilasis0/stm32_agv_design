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
void rc522_main(void)
{
    RC522_PCD_Init(&rfid_const);
}

void rc522_detected_card(void)
{
    if (RC522_PICC_IsNewCardPresent(&rfid_const)) {  // :contentReference[oaicite:5]{index=5}
        // 2. 讀取卡片序號（抗碰撞 + Select）
        if (RC522_PICC_ReadCardSerial(&rfid_const)) {  // :contentReference[oaicite:6]{index=6}
            // 3. 複製到應用層暫存結構
            memcpy(&uid_test, &uid, sizeof(RC522Uid));
            // TODO: 在此處理讀到的 uid_test.uidByte[0..uid_test.size-1]
        }
    }
}
