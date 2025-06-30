#include "main/main.h"
#include "main/config.h"

size_t defalt_running = 0;
void StartDefaultTask(void *argument)
{
    for(;;)
    {
        osDelay(10);
        defalt_running++;
    }
}
