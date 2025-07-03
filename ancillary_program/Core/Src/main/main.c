#include "main/main.h"
#include "main/config.h"
#include "rfid/main.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == rfid_const.IRQ_GPIO_PIN_x)
    {
        rc522_detected_card();
    }
}

size_t defalt_running = 0;
void StartDefaultTask(void *argument)
{
    rc522_main();
    for(;;)
    {
        osDelay(10);
        defalt_running++;
    }
}
