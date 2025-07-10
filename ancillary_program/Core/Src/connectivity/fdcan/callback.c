#include "connectivity/fdcan/callback.h"
#include "fdcan.h"
#include "connectivity/fdcan/main.h"

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if (hfdcan == &hfdcan1)
    {
        if (ITS_CHECK(ErrorStatusITs, FDCAN_IT_BUS_OFF))
        {
            fdcan_bus_off = true;
        }
    }
}

void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
{
    if (ITS_CHECK(TxEventFifoITs, FDCAN_IT_TX_EVT_FIFO_NEW_DATA))
    {

    }
    if (ITS_CHECK(TxEventFifoITs, FDCAN_IT_TX_EVT_FIFO_FULL))
    {
        FDCAN_TxEventFifoTypeDef txEvent;
        HAL_FDCAN_GetTxEvent(hfdcan, &txEvent);
    }
    if (ITS_CHECK(TxEventFifoITs, FDCAN_IT_TX_EVT_FIFO_ELT_LOST))
    {
        Error_Handler();
    }
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(ITS_CHECK(RxFifo0ITs, FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
    {
        vec_rm_all(&fdcan_recv0_buf);
        ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_RxHeader, fdcan_recv0_buf.data));
        fdcan_recv0_buf.len = fdcan_RxHeader.DataLength;
        last_error = instant_recv_proc(&fdcan_recv0_buf);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if(ITS_CHECK(RxFifo1ITs, FDCAN_IT_RX_FIFO1_NEW_MESSAGE))
    {
        vec_rm_all(&fdcan_recv1_buf);
        ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &fdcan_RxHeader, fdcan_recv1_buf.data));
        fdcan_recv1_buf.len = fdcan_RxHeader.DataLength;
        last_error = fdcan_trcv_buf_push(&fdcan_recv_pkt_buf, &fdcan_recv1_buf, fdcan_RxHeader.Identifier);
    }
}
