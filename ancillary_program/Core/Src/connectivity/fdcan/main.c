#include "connectivity/fdcan/main.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "connectivity/cmds.h"
#include "main/config.h"
#include "connectivity/write_pkt.h"

bool fdcan_enable = false;
bool fdacn_data_trsm_ready = true;

static FDCAN_TxHeaderTypeDef fdcanTxHeader = {
    .Identifier = 0x000,
    .IdType = FDCAN_STANDARD_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_8,
    .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
    .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN,
    .TxEventFifoControl = FDCAN_STORE_TX_EVENTS,
};
static FDCAN_RxHeaderTypeDef fdcanRxHeader = {0};

static VecByte fdcan_tr_buf;
static VecByte fdcan_rv_buf;

FdcanByteTrcvBuf fdcan_tr_pkt_buf;
FdcanByteTrcvBuf fdcan_rv_pkt_buf;

static inline bool fifo_it_check(uint32_t its, uint32_t tag) {
    return (its & tag) != RESET;
}

void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
{
    if (fifo_it_check(TxEventFifoITs, FDCAN_IT_TX_EVT_FIFO_NEW_DATA))
    {
        BOARD_LED_TOGGLE;
    }
    if (fifo_it_check(TxEventFifoITs, FDCAN_IT_TX_EVT_FIFO_FULL))
    {
        FDCAN_TxEventFifoTypeDef txEvent;
        HAL_FDCAN_GetTxEvent(hfdcan, &txEvent);
    }
    if (fifo_it_check(TxEventFifoITs, FDCAN_IT_TX_EVT_FIFO_ELT_LOST))
    {
        Error_Handler();
    }
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(fifo_it_check(RxFifo0ITs, FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
    {
        ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcanRxHeader, fdcan_rv_buf.data));
        fdcan_rv_buf.len = fdcanRxHeader.DataLength;
        fdcan_trcv_buf_push(&fdcan_rv_pkt_buf, &fdcan_rv_buf, fdcanRxHeader.Identifier);
    }
}

static UNUSED_FUNC FnState fdcan_setup(void)
{
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&fdcan_tr_buf, 8));
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&fdcan_rv_buf, 8));
    ERROR_CHECK_FNS_RETURN(fdcan_trcv_buf_setup(&fdcan_tr_pkt_buf, FDCAN_TRCV_BUF_CAP, FDCAN_VEC_BYTE_CAP));
    ERROR_CHECK_FNS_RETURN(fdcan_trcv_buf_setup(&fdcan_rv_pkt_buf, FDCAN_TRCV_BUF_CAP, FDCAN_VEC_BYTE_CAP));
    return FNS_OK;
}

static UNUSED_FUNC FnState fdcan_transmit(void)
{
    if (!fdcan_enable) return FNS_INVALID;
    vec_rm_all(&fdcan_tr_buf);
    ERROR_CHECK_FNS_RETURN(fdcan_trcv_buf_pop(&fdcan_tr_pkt_buf, &fdcan_tr_buf, &fdcanTxHeader.Identifier));
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &fdcanTxHeader, fdcan_tr_buf.data) == HAL_OK)
    {
    }
    return FNS_OK;
}

static UNUSED_FUNC FnState tr_pkt_proc(void)
{
    if (!fdacn_data_trsm_ready) return FNS_INVALID;
    VecByte vec_byte;
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&vec_byte, 8));
#ifdef ENABLE_CON_PKT_TEST
    pkt_test(&vec_byte);
    ERROR_CHECK_FNS_CLEAN(fdcan_trcv_buf_push(&fdcan_tr_pkt_buf, &vec_byte, 0x000), vec_byte_free(&vec_byte));
#endif
    ERROR_CHECK_FNS_RETURN(vec_byte_free(&vec_byte));
    return FNS_OK;
}

static UNUSED_FUNC FnState rv_pkt_proc(size_t count)
{
    VecByte vec_byte;
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&vec_byte, UART_VEC_BYTE_CAP));
    for (size_t i = 0; i < count; i++)
    {
        uint32_t id;
        ERROR_CHECK_FNS_CLEAN(fdcan_trcv_buf_pop(&fdcan_rv_pkt_buf, &vec_byte, &id), vec_byte_free(&vec_byte));
        uint8_t code = vec_byte.data[vec_byte.head];
        vec_rm_range(&vec_byte, 0, 1);
        switch (code)
        {
            case CMD_B0_DATA_STOP:
                fdacn_data_trsm_ready = false;
                break;
            case CMD_B0_DATA_START:
                fdacn_data_trsm_ready = true;
                break;
            default:
                last_error = FNS_NO_MATCH;
                break;
        }
    }
    vec_byte_free(&vec_byte);
    return FNS_OK;
}

#ifndef DISABLE_FDCAN
void StartFdCanTask(void *argument)
{
    ERROR_CHECK_FNS_VOID(fdcan_setup());
    FDCAN_FilterTypeDef sFilter0 = FDCAN_FilterTypeDef_DEFALT();
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter0));
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_Start(&hfdcan1));
    ERROR_CHECK_HAL_HANDLE(
        HAL_FDCAN_ActivateNotification(&hfdcan1,
            FDCAN_IT_TX_EVT_FIFO_NEW_DATA|FDCAN_IT_TX_EVT_FIFO_FULL|FDCAN_IT_TX_EVT_FIFO_ELT_LOST, 0)
    );
    ERROR_CHECK_HAL_HANDLE(
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE,
            FDCAN_TX_BUFFER0|FDCAN_TX_BUFFER1|FDCAN_TX_BUFFER2)
    );
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0));
    fdcan_enable = true;
    size_t tick = 0;
    for(;;)
    {
        fdcan_transmit();
        rv_pkt_proc(5);
        if (tick % 100 == 0)
        {
            tick = 0;
            tr_pkt_proc();
        }
        osDelay(10);
        tick++;
    }
}
#endif
