#pragma once     //使不重複 include

#include <stdint.h>
#include "connectivity/uart/main.h"

typedef struct GlobalState
{
    ByteTrcvBuf* uart_tr_pkt_buf_h;
    ByteTrcvBuf* uart_rv_pkt_buf_h;
} GlobalState;
extern GlobalState global_state;

void USER_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
