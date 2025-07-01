#include "main/it.h"
#include "main/config.h"
#include "main/vehicle.h"
#include "motor/main.h"

/**
  * PC13 按鈕中斷，用於測試
  *
  * Test interrupt for PC13 button (trigger on both edges), toggle hall_sensor_node
  */
void user_EXTI15_10_IRQHandler(void) {
}
