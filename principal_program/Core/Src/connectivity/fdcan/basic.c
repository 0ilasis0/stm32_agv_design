#include "connectivity/fdcan/basic.h"

bool fdcan_bus_off = false;

FncState fdacn_data_store = FNC_DISABLE;

FDCAN_TxHeaderTypeDef fdcan_TxHeader = {
    .Identifier = 0x000,
    .IdType = FDCAN_STANDARD_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_8,
    .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
    .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN,
    .TxEventFifoControl = FDCAN_STORE_TX_EVENTS,
};
FDCAN_RxHeaderTypeDef fdcan_RxHeader = {0};

VecByte fdcan_trsm_buf;
VecByte fdcan_recv0_buf;
VecByte fdcan_recv1_buf;

FdcanByteTrcvBuf fdcan_trsm_pkt_buf;
FdcanByteTrcvBuf fdcan_recv_pkt_buf;
