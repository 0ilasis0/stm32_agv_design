#pragma once     //使不重複 include

#include <stdint.h>
#include "connectivity/uart/main.h"

extern __IO uint32_t uwTick;

typedef struct GlobalState
{
    ByteTrcvBuf* uart_tr_pkt_buf_h;
    ByteTrcvBuf* uart_rv_pkt_buf_h;
} GlobalState;
extern GlobalState global_state;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void decide_move_mode(void);
void protect_over_hall(void);
