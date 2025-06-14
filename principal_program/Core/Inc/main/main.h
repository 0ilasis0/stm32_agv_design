#ifndef USER_MAIN_H
#define USER_MAIN_H     //使不重複 include

#include <stdint.h>
#include "main/vehicle.h"

void user_main(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void decide_move_mode(void);
void protect_over_hall(void);

#endif
