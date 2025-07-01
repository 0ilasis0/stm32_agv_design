#include "connectivity/uart/main.h"
#include <string.h>
#include "usart.h"
#include "main/config.h"
#include "main/fn_state.h"
#include "connectivity/cmds.h"

FncState uart_enable_trsm = FNC_DISABLE;
FncState uart_enable_recv = FNC_DISABLE;
FncState uart_data_trsm_ready = FNC_DISABLE;

static VecByte uart_tr_buf;
static VecByte uart_rv_buf;

ByteTrcvBuf uart_tr_pkt_buf;
ByteTrcvBuf uart_rv_pkt_buf;

#ifdef ENABLE_CON_PKT_TEST
uint32_t uart_test_pkt_c = 0;
#endif

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    if (huart->Instance != USART1) return;
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

static UNUSED_FNC void uart_setup(void)
{
    // Tx:PB9(R5) Rx:PB11(R18)
    if (
           (vec_byte_new(&uart_tr_buf, UART_VEC_BYTE_CAP + 2) == FNS_OK)
        && (connect_trcv_buf_setup(&uart_tr_pkt_buf, UART_TRCV_BUF_CAP, UART_VEC_BYTE_CAP) == FNS_OK)
    ) uart_enable_trsm = FNC_ENABLE;
    if (
           (vec_byte_new(&uart_rv_buf, UART_VEC_BYTE_CAP + 2) == FNS_OK)
        && (connect_trcv_buf_setup(&uart_rv_pkt_buf, UART_TRCV_BUF_CAP, UART_VEC_BYTE_CAP) == FNS_OK)
    ) uart_enable_recv = FNC_ENABLE;
}

static UNUSED_FNC FnState uart_transmit(void)
{
    if (HAL_DMA_GetState(huart1.hdmatx) == HAL_DMA_STATE_BUSY) return FNS_FAIL;
    vec_rm_all(&uart_tr_buf);
    ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(&uart_tr_buf, UART_START_CODE));
    ERROR_CHECK_FNS_RETURN(connect_trcv_buf_pop(&uart_tr_pkt_buf, &uart_tr_buf));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(&uart_tr_buf, UART_END_CODE));
    HAL_UART_Transmit_DMA(&huart1, uart_tr_buf.data, uart_tr_buf.len);
    return FNS_OK;
}

static UNUSED_FNC FnState trsm_pkt_proc(void)
{
    if (uart_data_trsm_ready == FNC_ENABLE)
    {
        VecByte vec_byte;
        ERROR_CHECK_FNS_RETURN(vec_byte_new(&vec_byte, 8));
        #ifdef ENABLE_CON_PKT_TEST
        ERROR_CHECK_FNS_WRI_PUSH(pkt_test(&vec_byte, &uart_test_pkt_c),
            connect_trcv_buf_push(&uart_tr_pkt_buf, &vec_byte), vec_byte_free(&vec_byte));
        #endif
        #ifdef PRINCIPAL_PROGRAM
        ERROR_CHECK_FNS_WRI_PUSH(pkt_left_speed(&vec_byte),
            connect_trcv_buf_push(&uart_tr_pkt_buf, &vec_byte), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_right_speed(&vec_byte),
            connect_trcv_buf_push(&uart_tr_pkt_buf, &vec_byte), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_left_duty(&vec_byte),
            connect_trcv_buf_push(&uart_tr_pkt_buf, &vec_byte), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_right_duty(&vec_byte),
            connect_trcv_buf_push(&uart_tr_pkt_buf, &vec_byte), vec_byte_free(&vec_byte));
        #endif
        vec_byte_free(&vec_byte);
    }
    return FNS_OK;
}

static UNUSED_FNC FnState recv_pkt_proc(size_t count)
{
    VecByte vec_byte;
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&vec_byte, UART_VEC_BYTE_CAP));
    for (size_t i = 0; i < count; i++)
    {
        if (ERROR_CHECK_FNS_RAW(connect_trcv_buf_pop(&uart_rv_pkt_buf, &vec_byte))) break;
        uint8_t code = vec_byte.data[vec_byte.head];
        vec_rm_range(&vec_byte, 0, 1);
        switch (code)
        {
            case CMD_B0_DATA_STOP:
                uart_data_trsm_ready = false;
                break;
            case CMD_B0_DATA_START:
                uart_data_trsm_ready = true;
                break;
            default:
                last_error = FNS_NO_MATCH;
                break;
        }
    }
    vec_byte_free(&vec_byte);
    return FNS_OK;
}

#ifndef DISABLE_UART
void StartUartTask(void *argument)
{
    uart_setup();
    if (uart_enable_recv == FNC_ENABLE)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rv_buf.data, uart_rv_buf.cap);
    }
    size_t tick = 0;
    for (;;)
    {
        if (uart_enable_trsm == FNC_ENABLE) uart_transmit();
        recv_pkt_proc(5);
        if (tick % 500 == 0)
        {
            tick = 0;
            trsm_pkt_proc();
        }
        osDelay(10);
        tick++;
    }
}
#endif
