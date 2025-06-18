#include "fdcan/main.h"
#include "fdcan.h"

static FDCAN_RxHeaderTypeDef RxHeader;
VecU8 RxData = VEC_U8_NEW();
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
VecU8 TxData = VEC_U8_NEW();

void user_MX_FDCAN1_Init(void)
{
    vec_u8_push(&TxData, (uint8_t[]){0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}, 8);
    FDCAN_FilterTypeDef sFilter0 = FDCAN_FilterTypeDef_DEFALT();
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter0) != HAL_OK) Error_Handler();
}

void fdcan_setup(void) {
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
        Error_Handler();
    }
}

#define BOARD_LED_TOGGLE HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5)

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

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
}

void fdcan_transmit(void)
{
    int i;
    for (i = 0; i < 8; i++)
    {
        TxData.data[i]++;
    }
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData.data);
}

void fdcan_loop(void)
{
}
