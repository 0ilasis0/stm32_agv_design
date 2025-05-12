#ifndef PRINCIPAL_MAIN_H
#define PRINCIPAL_MAIN_H     //使不重複 include

#include <stdint.h>

void principal_main(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void decide_move_mode(void);
void protect_over_hall(void);
void ensure_notor_stop(void);

#endif
