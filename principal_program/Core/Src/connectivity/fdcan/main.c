#include "connectivity/fdcan/main.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "connectivity/cmds.h"

#ifdef PRINCIPAL_PROGRAM
#include "main/vehicle2.h"
#include "motor/main.h"
#endif
#ifdef ANCILLARY_PROGRAM
#include "robotic_arm/main.h"
#endif

FncState fdacn_data_trsm_ready = FNC_DISABLE;

static bool fdcan_bus_off = false;

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

static VecByte fdcan_trsm_buf;
static VecByte fdcan_recv0_buf;
static VecByte fdcan_recv1_buf;

FdcanByteTrcvBuf fdcan_trsm_pkt_buf;
FdcanByteTrcvBuf fdcan_recv_pkt_buf;

#ifdef ENABLE_CON_PKT_TEST
uint32_t fdcan_test_pkt_c = 0;
#endif

static UNUSED_FNC void fdcan_init(void)
{
    ERROR_CHECK_FNS_HANDLE(vec_byte_new(&fdcan_trsm_buf, 8));
    ERROR_CHECK_FNS_HANDLE(fdcan_trcv_buf_setup(&fdcan_trsm_pkt_buf, FDCAN_TRCV_BUF_CAP, FDCAN_VEC_BYTE_CAP));
    ERROR_CHECK_FNS_HANDLE(vec_byte_new(&fdcan_recv0_buf, 8));
    ERROR_CHECK_FNS_HANDLE(vec_byte_new(&fdcan_recv1_buf, 8));
    ERROR_CHECK_FNS_HANDLE(fdcan_trcv_buf_setup(&fdcan_recv_pkt_buf, FDCAN_TRCV_BUF_CAP, FDCAN_VEC_BYTE_CAP));
}

static UNUSED_FNC void fdcan_set_filter(FDCAN_HandleTypeDef *hfdcan)
{
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE));
    FDCAN_FilterTypeDef sFilter0 = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,
        .FilterType = FDCAN_FILTER_RANGE,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP,
        .FilterID1 = FDCAN_FILTER_ID_MIN,
        .FilterID2 = FDCAN_FILTER_ID_MIN + 3,
    };
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ConfigFilter(hfdcan, &sFilter0));
    FDCAN_FilterTypeDef sFilter1 = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 1,
        .FilterType = FDCAN_FILTER_RANGE,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO1,
        .FilterID1 = FDCAN_FILTER_ID_MIN + 4,
        .FilterID2 = FDCAN_FILTER_ID_MAX,
    };
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ConfigFilter(hfdcan, &sFilter1));
}

static UNUSED_FNC void fdcan_set_notification(FDCAN_HandleTypeDef *hfdcan)
{
    ERROR_CHECK_HAL_HANDLE(
        HAL_FDCAN_ActivateNotification(hfdcan,
              FDCAN_IT_BUS_OFF
            | FDCAN_IT_TX_EVT_FIFO_NEW_DATA
            | FDCAN_IT_TX_EVT_FIFO_FULL
            | FDCAN_IT_TX_EVT_FIFO_ELT_LOST
        , 0)
    );
    ERROR_CHECK_HAL_HANDLE(
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_COMPLETE,
              FDCAN_TX_BUFFER0
            | FDCAN_TX_BUFFER1
            | FDCAN_TX_BUFFER2
        )
    );
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0));
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0));
}

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
    BOARD_LED_TOGGLE;
}

static FnState fifo0_recv_pkt_proc(VecByte* vec_byte)
{
    uint8_t code;
    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, &code, 0));
    ERROR_CHECK_FNS_RETURN(vec_rm_range(vec_byte, 0, 1));
    switch (code)
    {
        #ifdef PRINCIPAL_PROGRAM
        case CMD_B0_VECH_CONTROL:
        {
            ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, &code, 0));
            ERROR_CHECK_FNS_RETURN(vec_rm_range(vec_byte, 0, 1));
            switch (code)
            {
                case CMD_B1_VEHICLE:
                {
                    uint8_t value;
                    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, &code, 0));
                    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, &value, 1));
                    ERROR_CHECK_FNS_RETURN(vec_rm_range(vec_byte, 0, 2));
                    MOTIONCOMMAND mode = motion_unchange;
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
                    uint8_t value;
                    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, &code, 0));
                    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, &value, 1));
                    ERROR_CHECK_FNS_RETURN(vec_rm_range(vec_byte, 0, 2));
                    MotorParameter* motor = &motor_left;
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
                    uint8_t value;
                    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, &code, 0));
                    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, &value, 1));
                    ERROR_CHECK_FNS_RETURN(vec_rm_range(vec_byte, 0, 2));
                    MotorParameter* motor = &motor_right;
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
            ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, &code, 0));
            ERROR_CHECK_FNS_RETURN(vec_rm_range(vec_byte, 0, 1));
            switch (code)
            {
                case CMD_B1_ARM:
                {
                    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, &code, 0));
                    ERROR_CHECK_FNS_RETURN(vec_rm_range(vec_byte, 0, 1));
                    switch (code)
                    {
                        case CMD_B2_STOP:
                        {
                            arm_set_tim(&arm_bottom, arm_bottom.tim_current);
                            arm_set_tim(&arm_shoulder, arm_shoulder.tim_current);
                            arm_set_tim(&arm_elbow_btm, arm_elbow_btm.tim_current);
                            arm_set_tim(&arm_elbow_top, arm_elbow_top.tim_current);
                            arm_set_tim(&arm_wrist, arm_wrist.tim_current);
                            arm_set_tim(&arm_finger, arm_finger.tim_current);
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
                case CMD_B1_BOTTOM:
                {
                    proc_arm_set(vec_byte, &arm_bottom);
                    break;
                }
                case CMD_B1_SHOULDER:
                {
                    proc_arm_set(vec_byte, &arm_shoulder);
                    break;
                }
                case CMD_B1_ELBOW_BTM:
                {
                    proc_arm_set(vec_byte, &arm_elbow_btm);
                    break;
                }
                case CMD_B1_ELBOW_TOP:
                {
                    proc_arm_set(vec_byte, &arm_elbow_top);
                    break;
                }
                case CMD_B1_WRIST:
                {
                    proc_arm_set(vec_byte, &arm_wrist);
                    break;
                }
                case CMD_B1_FINGER:
                {
                    proc_arm_set(vec_byte, &arm_finger);
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
        default:
        {
            last_error = FNS_NO_MATCH;
            return FNS_NO_MATCH;
        }
    }
    return FNS_OK;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(ITS_CHECK(RxFifo0ITs, FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
    {
        vec_rm_all(&fdcan_recv0_buf);
        ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcanRxHeader, fdcan_recv0_buf.data));
        fdcan_recv0_buf.len = fdcanRxHeader.DataLength;
        fifo0_recv_pkt_proc(&fdcan_recv0_buf);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if(ITS_CHECK(RxFifo1ITs, FDCAN_IT_RX_FIFO1_NEW_MESSAGE))
    {
        vec_rm_all(&fdcan_recv1_buf);
        ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &fdcanRxHeader, fdcan_recv1_buf.data));
        fdcan_recv1_buf.len = fdcanRxHeader.DataLength;
        fdcan_trcv_buf_push(&fdcan_recv_pkt_buf, &fdcan_recv1_buf, fdcanRxHeader.Identifier);
    }
}

static UNUSED_FNC FnState pkt_transmit(void)
{
    vec_rm_all(&fdcan_trsm_buf);
    ERROR_CHECK_FNS_RETURN(fdcan_trcv_buf_pop(&fdcan_trsm_pkt_buf, &fdcan_trsm_buf, &fdcanTxHeader.Identifier));
    fdcanTxHeader.DataLength = fdcan_trsm_buf.len;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &fdcanTxHeader, fdcan_trsm_buf.data) == HAL_OK)
    {
    }
    return FNS_OK;
}

static UNUSED_FNC FnState trsm_pkt_proc(void)
{
    VecByte vec_byte;
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&vec_byte, 8));
    if (fdacn_data_trsm_ready == FNC_ENABLE)
    {
        #ifdef ENABLE_CON_PKT_TEST
        ERROR_CHECK_FNS_WRI_PUSH(pkt_test(&vec_byte, &fdcan_test_pkt_c),
            fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_TEST_ID), vec_byte_free(&vec_byte));
        #endif
        #ifndef ENABLE_CON_PKT_TEST
        #ifdef PRINCIPAL_PROGRAM
        ERROR_CHECK_FNS_WRI_PUSH(pkt_left_speed(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_MOTOR_DATA_ID), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_right_speed(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_MOTOR_DATA_ID), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_left_duty(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_MOTOR_DATA_ID), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_right_duty(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_MOTOR_DATA_ID), vec_byte_free(&vec_byte));
        #endif
        #endif
    }
    ERROR_CHECK_FNS_RETURN(vec_byte_free(&vec_byte));
    return FNS_OK;
}

#ifdef ANCILLARY_PROGRAM
static FnState proc_arm_set(VecByte* vec_byte, ArmParameter* arm)
{
    uint8_t code;
    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, &code, 0));
    ERROR_CHECK_FNS_RETURN(vec_rm_range(vec_byte, 0, 1));
    switch (code)
    {
        case CMD_B2_STOP:
        {
            arm_set_tim(arm, arm->tim_current);
            break;
        }
        case CMD_B2_SET:
        {
            arm_set_pos(arm, vec_byte->data[vec_byte->head]);
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
#endif

static FnState recv_pkt_proc_inner(VecByte* vec_byte)
{
    uint8_t code;
    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, &code, 0));
    ERROR_CHECK_FNS_RETURN(vec_rm_range(vec_byte, 0, 1));
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

static UNUSED_FNC FnState recv_pkt_proc(size_t count)
{
    VecByte vec_byte;
    uint32_t id;
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&vec_byte, UART_VEC_BYTE_CAP));
    for (size_t i = 0; i < count; i++)
    {
        ERROR_CHECK_FNS_BREAK(fdcan_trcv_buf_pop(&fdcan_recv_pkt_buf, &vec_byte, &id));
        recv_pkt_proc_inner(&vec_byte);
    }
    vec_byte_free(&vec_byte);
    return FNS_OK;
}

#ifndef DISABLE_FDCAN
void StartFdCanTask(void *argument)
{
    fdcan_init();
    fdcan_set_filter(&hfdcan1);
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_Start(&hfdcan1));
    fdcan_set_notification(&hfdcan1);
    size_t tick = 0;
    for(;;)
    {
        if (fdcan_bus_off)
        {
            fdcan_bus_off = false;
            HAL_FDCAN_Stop(&hfdcan1);
            HAL_FDCAN_Start(&hfdcan1);
        }
        pkt_transmit();
        recv_pkt_proc(5);
        if (tick % 50 == 0)
        {
            tick = 0;
            trsm_pkt_proc();
        }
        osDelay(10);
        tick++;
    }
}
#endif
