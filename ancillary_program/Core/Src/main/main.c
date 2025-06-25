#include "main/main.h"
#include "cmsis_os.h"

size_t defalt_running = 0;
StartDefaultTask(void *argument)
{
    for(;;)
    {
        osDelay(10);
        defalt_running++;
    }
}
