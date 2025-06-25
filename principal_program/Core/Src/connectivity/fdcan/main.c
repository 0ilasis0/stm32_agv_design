#include "connectivity/fdcan/main.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "connectivity/cmds.h"
#include "main/config.h"

bool fdcan_enable = false;
bool fdacn_data_trsm_ready = false;

FDCAN_TxHeaderTypeDef fdcanTxHeader = {
    .Identifier = 0x000,
    .IdType = FDCAN_STANDARD_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_8,
    .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
    .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN,
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
};
FDCAN_RxHeaderTypeDef fdcanRxHeader = {0};

VecByte fdcan_tr_buf;
VecByte fdcan_rv_buf;

FdcanByteTrcvBuf fdcan_tr_pkt_buf;
FdcanByteTrcvBuf fdcan_rv_pkt_buf;

static FnState fdcan_setup(void)
{
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&fdcan_tr_buf, 8));
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&fdcan_rv_buf, 8));
    ERROR_CHECK_FNS_RETURN(vec_byte_push(&fdcan_tr_buf, (uint8_t[]){0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}, 8));
    ERROR_CHECK_FNS_RETURN(fdcan_trcv_buf_setup(&fdcan_tr_pkt_buf, FDCAN_TRCV_BUF_CAP, FDCAN_VEC_BYTE_CAP));
    ERROR_CHECK_FNS_RETURN(fdcan_trcv_buf_setup(&fdcan_rv_pkt_buf, FDCAN_TRCV_BUF_CAP, FDCAN_VEC_BYTE_CAP));
    return FNS_OK;
}

static FnState fdcan_transmit(void)
{
    if (!fdcan_enable) return FNS_INVALID;
    fdcan_tr_buf.data[0]++;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &fdcanTxHeader, fdcan_tr_buf.data) == HAL_OK)
    {
    }
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
