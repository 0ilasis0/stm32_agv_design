#include "connectivity/fdcan/main.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "connectivity/cmds.h"
#include "main/config.h"

static FDCAN_TxHeaderTypeDef TxHeader = {
    .Identifier = FDCAN_DEVICE_ID,
    .IdType = FDCAN_STANDARD_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_8,
    .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
    .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN,
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
};
static FDCAN_RxHeaderTypeDef RxHeader;

VecByte TxData;
VecByte RxData;

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
    BOARD_LED_TOGGLE;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData.data) != HAL_OK) Error_Handler();
        RxData.len = RxHeader.DataLength;
        if ((RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.Identifier == FDCAN_DEVICE_ID))
        {
            BOARD_LED_TOGGLE;
        }
    }
}

static FnState fdcan_transmit(void)
{
    TxData.data[0]++;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData.data) == HAL_OK)
    {
    }
    return FNS_OK;
}

static FnState fdcan_setup(void)
{
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&TxData, 8));
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&RxData, 8));
    ERROR_CHECK_FNS_RETURN(vec_byte_push(&TxData, (uint8_t[]){0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}, 8));
    return FNS_OK;
}

#ifndef DISABLE_FDCAN
void StartFdCanTask(void *argument)
{
    ERROR_CHECK_FNS_VOID(fdcan_setup());
    FDCAN_FilterTypeDef sFilter0 = FDCAN_FilterTypeDef_DEFALT();
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter0));
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_Start(&hfdcan1));
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0 |FDCAN_TX_BUFFER1 |FDCAN_TX_BUFFER2));
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0));
    size_t tick = 0;
    for(;;)
    {
        if (tick % 20 == 0)
        {
            fdcan_transmit();
            tick = 0;
        }
        osDelay(50);
        tick++;
    }
}
#endif
