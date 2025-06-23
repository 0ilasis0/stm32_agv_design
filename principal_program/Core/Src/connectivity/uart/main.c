#include "connectivity/uart/main.h"
#include <string.h>
#include "cmsis_os.h"
#include "usart.h"
#include "connectivity/cmds.h"
#include "connectivity/write_pkt.h"
#include "main/config.h"
#include "main/fn_state.h"

static bool data_tranmit = false;
static VecByte uart_tr_buf;
static VecByte uart_rv_buf;

ByteTrcvBuf uart_tr_pkt_buf;
ByteTrcvBuf uart_rv_pkt_buf;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    if (huart->Instance == USART3)
    {
        if (Size != 0)
        {
            uart_rv_buf.len = Size;
            if (
                (uart_rv_buf.data[0] == UART_START_CODE)
                && (uart_rv_buf.data[uart_rv_buf.len-1] == UART_END_CODE)
            ) {
                vec_rm_range(&uart_rv_buf, 0, 1);
                vec_rm_range(&uart_rv_buf, uart_rv_buf.len-1, 1);
                connect_trcv_buf_push(&uart_rv_pkt_buf, &uart_rv_buf);
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_rv_buf.data, uart_rv_buf.cap);
    }
}

static FnState uart_transmit(void)
{
    if (HAL_DMA_GetState(huart3.hdmatx) == HAL_DMA_STATE_BUSY) return FNS_FAIL;
    vec_rm_all(&uart_tr_buf);
    FNS_ERROR_CHECK(vec_byte_push_byte(&uart_tr_buf, UART_START_CODE));
    FNS_ERROR_CHECK(connect_trcv_buf_pop(&uart_tr_pkt_buf, &uart_tr_buf));
    FNS_ERROR_CHECK(vec_byte_push_byte(&uart_tr_buf, UART_END_CODE));
    HAL_UART_Transmit_DMA(&huart3, uart_tr_buf.data, uart_tr_buf.len);
    return FNS_OK;
}

static FnState tr_pkt_proc(void)
{
    if (!data_tranmit) return FNS_INVALID;
    pkt_left_speed(&uart_tr_pkt_buf);
    pkt_right_speed(&uart_tr_pkt_buf);
    pkt_left_duty(&uart_tr_pkt_buf);
    pkt_right_duty(&uart_tr_pkt_buf);
    return FNS_OK;
}

static FnState rv_pkt_proc(size_t count)
{
    VecByte vec_byte;
    FNS_ERROR_CHECK(vec_byte_new(&vec_byte, UART_VEC_MAX));
    for (size_t i = 0; i < count; i++)
    {
        FNS_ERROR_CHECK_CLEAN(connect_trcv_buf_pop(&uart_rv_pkt_buf, &vec_byte), vec_byte_free(&vec_byte));
        uint8_t code = vec_byte.data[vec_byte.head];
        vec_rm_range(&vec_byte, 0, 1);
        switch (code)
        {
            case CMD_B0_DATA_STOP:
                data_tranmit = false;
                break;
            case CMD_B0_DATA_START:
                data_tranmit = true;
                break;
            default:
                last_error = FNS_NO_MATCH;
                break;
        }
    }
    vec_byte_free(&vec_byte);
    return FNS_OK;
}

static FnState uart_setup(void)
{
    // Tx:PB9(R5) Rx:PB11(R18)
    FNS_ERROR_CHECK(vec_byte_new(&uart_tr_buf, UART_VEC_MAX + 2));
    FNS_ERROR_CHECK(vec_byte_new(&uart_rv_buf, UART_VEC_MAX + 2));
    FNS_ERROR_CHECK(connect_trcv_buf_setup(&uart_tr_pkt_buf, UART_TRCV_BUF_CAP, UART_VEC_MAX));
    FNS_ERROR_CHECK(connect_trcv_buf_setup(&uart_rv_pkt_buf, UART_TRCV_BUF_CAP, UART_VEC_MAX));
    return FNS_OK;
}

#ifndef DISABLE_UART
void StartUartTask(void *argument)
{
    if (uart_setup() != FNS_OK) {
        for (;;)
        {
            osDelay(1000);
        }
    }
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
#ifndef DISABLE_UART_RECV
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_rv_buf.data, uart_rv_buf.cap);
#endif
    size_t tick = 0;
    for (;;)
    {
#ifndef DISABLE_UART_TRSM
        uart_transmit();
#endif
#ifndef DISABLE_UART_RECV
        rv_pkt_proc(5);
#endif
        if (tick % 500 == 0)
        {
            tick = 0;
#ifndef DISABLE_UART_TRSM
            tr_pkt_proc();
#endif
        }
        osDelay(10);
        tick++;
    }
}
#endif
