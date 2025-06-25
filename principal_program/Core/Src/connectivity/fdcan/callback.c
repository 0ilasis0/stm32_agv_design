#include "connectivity/fdcan/callback.h"
#include "fdcan.h"
#include "main/config.h"
#include "connectivity/fdcan/main.h"

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
        HAL_FDCAN_GetTxEvent(&hfdcan, &txEvent);
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
