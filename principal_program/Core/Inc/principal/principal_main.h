#ifndef MY_MYCODE_H
#define MY_MYCODE_H     //使不重複 include

#include "common.h"
#include "PI_control.h"
#include "principal_it.h"
#include "principal_adc.h"
#include "vehicle.h"
#include "motor.h"

#include "packet.h"
#include "vec.h"
#include "my_uart.h"

void MCmain(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void decide_move_mode(void);

#endif
