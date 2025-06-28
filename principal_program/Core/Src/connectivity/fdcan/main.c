#include "connectivity/fdcan/main.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "connectivity/cmds.h"

#ifdef PRINCIPAL_PROGRAM
#include "main/vehicle2.h"
#include "motor/main.h"
#endif

FncState fdcan_enable_trsm = FNC_DISABLE;
FncState fdcan_enable_recv = FNC_DISABLE;
FncState fdacn_data_trsm_ready = FNC_DISABLE;

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
static VecByte fdcan_rv0_buf;
static VecByte fdcan_rv1_buf;

FdcanByteTrcvBuf fdcan_tr_pkt_buf;
FdcanByteTrcvBuf fdcan_rv_pkt_buf;

size_t fdcant[3] = {0};

#ifdef ENABLE_CON_PKT_TEST
uint32_t fdcan_test_pkt_c = 0;
#endif

static inline bool fifo_it_check(uint32_t its, uint32_t tag) {
    return (its & tag) != RESET;
}

void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
{
    fdcant[1]++;
    if (fifo_it_check(TxEventFifoITs, FDCAN_IT_TX_EVT_FIFO_NEW_DATA))
    {
        
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
    fdcant[0]++;
    BOARD_LED_TOGGLE;
}

static FnState fifo0_proc0(VecByte* vec_byte);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(fifo_it_check(RxFifo0ITs, FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
    {
        ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcanRxHeader, fdcan_rv1_buf.data));
        fdcan_rv1_buf.len = fdcanRxHeader.DataLength;
        fifo0_proc0(&fdcan_rv1_buf);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    fdcant[2]++;
    if(fifo_it_check(RxFifo1ITs, FDCAN_IT_RX_FIFO1_NEW_MESSAGE))
    {
        ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &fdcanRxHeader, fdcan_rv0_buf.data));
        fdcan_rv0_buf.len = fdcanRxHeader.DataLength;
        fdcan_trcv_buf_push(&fdcan_rv_pkt_buf, &fdcan_rv0_buf, fdcanRxHeader.Identifier);
    }
}

static UNUSED_FNC void init(void)
{
    if (
           (vec_byte_new(&fdcan_tr_buf, 8) == FNS_OK)
        && (fdcan_trcv_buf_setup(&fdcan_tr_pkt_buf, FDCAN_TRCV_BUF_CAP, FDCAN_VEC_BYTE_CAP) == FNS_OK)
    ) fdcan_enable_trsm = FNC_ENABLE;
    if (
           (vec_byte_new(&fdcan_rv0_buf, 8) == FNS_OK)
        && (vec_byte_new(&fdcan_rv1_buf, 8) == FNS_OK)
        && (fdcan_trcv_buf_setup(&fdcan_rv_pkt_buf, FDCAN_TRCV_BUF_CAP, FDCAN_VEC_BYTE_CAP) == FNS_OK)
    ) fdcan_enable_recv = FNC_ENABLE;
}

static UNUSED_FNC FnState pkt_transmit(void)
{
    vec_rm_all(&fdcan_tr_buf);
    ERROR_CHECK_FNS_RETURN(fdcan_trcv_buf_pop(&fdcan_tr_pkt_buf, &fdcan_tr_buf, &fdcanTxHeader.Identifier));
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &fdcanTxHeader, fdcan_tr_buf.data) == HAL_OK)
    {
    }
    return FNS_OK;
}

static UNUSED_FNC FnState tr_pkt_proc(void)
{
    if (fdacn_data_trsm_ready == FNC_ENABLE)
    {
        VecByte vec_byte;
        ERROR_CHECK_FNS_RETURN(vec_byte_new(&vec_byte, 8));
        #ifdef ENABLE_CON_PKT_TEST
        ERROR_CHECK_FNS_WRI_PUSH(pkt_test(&vec_byte, &fdcan_test_pkt_c),
            fdcan_trcv_buf_push(&fdcan_tr_pkt_buf, &vec_byte, FDCAN_TEST_ID), vec_byte_free(&vec_byte));
        #endif
        #ifndef ENABLE_CON_PKT_TEST
        #ifdef PRINCIPAL_PROGRAM
        ERROR_CHECK_FNS_WRI_PUSH(pkt_left_speed(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_tr_pkt_buf, &vec_byte, FDCAN_MOTOR_DATA_ID), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_right_speed(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_tr_pkt_buf, &vec_byte, FDCAN_MOTOR_DATA_ID), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_left_duty(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_tr_pkt_buf, &vec_byte, FDCAN_MOTOR_DATA_ID), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_right_duty(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_tr_pkt_buf, &vec_byte, FDCAN_MOTOR_DATA_ID), vec_byte_free(&vec_byte));
        #endif
        #endif
        ERROR_CHECK_FNS_RETURN(vec_byte_free(&vec_byte));
    }
    return FNS_OK;
}

static FnState fifo0_proc0(VecByte* vec_byte)
{
    uint8_t code = vec_byte->data[vec_byte->head];
    vec_rm_range(vec_byte, 0, 1);
    switch (code)
    {
        #ifdef PRINCIPAL_PROGRAM
        case CMD_B0_VECH_CONTROL:
        {
            code = vec_byte->data[vec_byte->head];
            vec_rm_range(vec_byte, 0, 1);
            switch (code)
            {
                case CMD_B1_VEHICLE:
                {
                    code = vec_byte->data[vec_byte->head];
                    MOTIONCOMMAND mode = motion_unchange;
                    uint8_t value = vec_byte->data[vec_byte->head];
                    vec_rm_range(vec_byte, 0, 2);
                    switch (code)
                    {
                        case CMD_B2_STOP:
                            value = 0;
                            vehicle2_motion_and_speed_control(mode, value);
                            break;
                        case CMD_B2_FOWARD:
                            mode = motion_forward;
                            vehicle2_motion_and_speed_control(mode, value);
                            break;
                        case CMD_B2_BACKWARD:
                            mode = motion_backward;
                            vehicle2_motion_and_speed_control(mode, value);
                            break;
                        default:
                            last_error = FNS_NO_MATCH;
                            break;
                    }
                    break;
                }
                case CMD_B1_LEFT_MOTOR:
                {
                    code = vec_byte->data[vec_byte->head];
                    MotorParameter* motor = &motor_left;
                    uint8_t value = vec_byte->data[vec_byte->head];
                    vec_rm_range(vec_byte, 0, 2);
                    switch (code)
                    {
                        case CMD_B2_STOP:
                            value = 0;
                            motor_set_speed_setpoint(motor, value);
                            break;
                        case CMD_B2_FOWARD:
                            // ? need check direction
                            motor_set_direction(motor, rotate_clockwise);
                            motor_set_speed_setpoint(motor, value);
                            break;
                        case CMD_B2_BACKWARD:
                            // ? need check direction
                            motor_set_direction(motor, rotate_c_clockwise);
                            motor_set_speed_setpoint(motor, value);
                            break;
                        default:
                            last_error = FNS_NO_MATCH;
                            break;
                    }
                    break;
                }
                case CMD_B1_RIGHT_MOTOR:
                {
                    code = vec_byte->data[vec_byte->head];
                    MotorParameter* motor = &motor_right;
                    uint8_t value = vec_byte->data[vec_byte->head];
                    vec_rm_range(vec_byte, 0, 2);
                    switch (code)
                    {
                        case CMD_B2_STOP:
                            value = 0;
                            motor_set_speed_setpoint(motor, value);
                            break;
                        case CMD_B2_FOWARD:
                            // ? need check direction
                            motor_set_direction(motor, rotate_clockwise);
                            motor_set_speed_setpoint(motor, value);
                            break;
                        case CMD_B2_BACKWARD:
                            // ? need check direction
                            motor_set_direction(motor, rotate_c_clockwise);
                            motor_set_speed_setpoint(motor, value);
                            break;
                        default:
                            last_error = FNS_NO_MATCH;
                            break;
                    }
                    break;
                }
                default:
                {
                    last_error = FNS_NO_MATCH;
                    return FNS_NO_MATCH;
                }
            }
            break;
        }
        #endif
        #ifdef ANCILLARY_PROGRAM
        case CMD_B0_ARM_CONTROL:
        {
            break;
        }
        #endif
        default:
        {
            last_error = FNS_NO_MATCH;
            return FNS_NO_MATCH;
        }
    }
    return FNS_OK;
}

static FnState fifo1_proc0(VecByte* vec_byte)
{
    uint8_t code = vec_byte->data[vec_byte->head];
    vec_rm_range(vec_byte, 0, 1);
    switch (code)
    {
        case CMD_B0_DATA_STOP:
        {
            fdacn_data_trsm_ready = false;
            break;
        }
        case CMD_B0_DATA_START:
        {
            fdacn_data_trsm_ready = true;
            break;
        }
        default:
        {
            last_error = FNS_NO_MATCH;
            return FNS_NO_MATCH;
        }
    }
    return FNS_OK;
}

static UNUSED_FNC FnState rv_pkt_proc(size_t count)
{
    VecByte vec_byte;
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&vec_byte, UART_VEC_BYTE_CAP));
    for (size_t i = 0; i < count; i++)
    {
        uint32_t id;
        ERROR_CHECK_FNS_CLEAN(fdcan_trcv_buf_pop(&fdcan_rv_pkt_buf, &vec_byte, &id), vec_byte_free(&vec_byte));
        fifo1_proc0(&vec_byte);
    }
    vec_byte_free(&vec_byte);
    return FNS_OK;
}

#ifndef DISABLE_FDCAN
void StartFdCanTask(void *argument)
{
    init();
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE));
    FDCAN_FilterTypeDef sFilter0 = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,
        .FilterType = FDCAN_FILTER_RANGE,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP,
        .FilterID1 = FDCAN_FILTER_ID_MIN,
        .FilterID2 = FDCAN_FILTER_ID_MIN + 3,
    };
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter0));
    FDCAN_FilterTypeDef sFilter1 = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 1,
        .FilterType = FDCAN_FILTER_RANGE,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO1,
        .FilterID1 = FDCAN_FILTER_ID_MIN + 4,
        .FilterID2 = FDCAN_FILTER_ID_MAX,
    };
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter1));
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_Start(&hfdcan1));
    ERROR_CHECK_HAL_HANDLE(
        HAL_FDCAN_ActivateNotification(&hfdcan1,
            FDCAN_IT_TX_EVT_FIFO_NEW_DATA|FDCAN_IT_TX_EVT_FIFO_FULL|FDCAN_IT_TX_EVT_FIFO_ELT_LOST, 0)
    );
    ERROR_CHECK_HAL_HANDLE(
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE,
            FDCAN_TX_BUFFER0|FDCAN_TX_BUFFER1|FDCAN_TX_BUFFER2)
    );
    if (fdcan_enable_recv == FNC_ENABLE)
    {
        ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0));
        ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0));
    }
    size_t tick = 0;
    for(;;)
    {
        if (fdcan_enable_trsm == FNC_ENABLE) pkt_transmit();
        rv_pkt_proc(5);
        if (tick % 50 == 0)
        {
            tick = 0;
            tr_pkt_proc();
        }
        osDelay(10);
        tick++;
    }
}
#endif
