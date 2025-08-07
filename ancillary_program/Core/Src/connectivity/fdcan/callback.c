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

FDCAN_RxHeaderTypeDef RxHeader0 = {0};
FDCAN_RxHeaderTypeDef RxHeader1 = {0};
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(ITS_CHECK(RxFifo0ITs, FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
    {
        FdcanPkt* pkt = RESULT_UNWRAP_HANDLE(fdcan_pkt_pool_alloc());
        ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_GetRxMessage(
            hfdcan, FDCAN_RX_FIFO0, &RxHeader0, pkt->data));
        pkt->id = RxHeader0.Identifier;
        pkt->len = RxHeader0.DataLength;
        instant_recv_proc(pkt);
        fdcan_pkt_pool_free(pkt);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if(ITS_CHECK(RxFifo1ITs, FDCAN_IT_RX_FIFO1_NEW_MESSAGE))
    {
        // FdcanPkt* pkt = RESULT_UNWRAP_HANDLE(fdcan_pkt_pool_alloc());
        // ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_GetRxMessage(
        //     hfdcan, FDCAN_RX_FIFO1, &RxHeader1, pkt->data));
        // pkt->id = RxHeader1.Identifier;
        // pkt->len = RxHeader1.DataLength;
        // RESULT_CHECK_HANDLE(fdcan_pkt_buf_push(&fdcan_recv_pkt_buf, pkt));
    }
}
