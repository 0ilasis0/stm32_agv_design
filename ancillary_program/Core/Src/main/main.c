#include "main/main.h"
#include "main/config.h"
#include "rfid/main.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
}

size_t defalt_running = 0;
void StartDefaultTask(void *argument)
{
    for(;;)
    {
        osDelay(10);
        defalt_running++;
    }
}
