#include "main/main.h"
#include "main/config.h"
#include "rfid/main.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == rfid_const.IRQ_GPIO_PIN_x)
    {
    }
}

size_t defalt_running = 0;
void StartDefaultTask(void *argument)
{
    rc522_setup();
    for(;;)
    {
        rc522_main();
        osDelay(10);
        defalt_running++;
    }
}
