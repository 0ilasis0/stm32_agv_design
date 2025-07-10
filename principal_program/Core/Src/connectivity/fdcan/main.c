#include "connectivity/fdcan/main.h"
#include "fdcan.h"
#include "connectivity/cmds.h"

#ifdef ENABLE_CON_PKT_TEST
uint32_t fdcan_test_pkt_c = 0;
#endif

#ifdef PRINCIPAL_PROGRAM
#include "vehicle/basic.h"
#include "motor/main.h"
#endif

#ifdef ANCILLARY_PROGRAM
#include "robotic_arm/main.h"
#include "rfid/main.h"
static FnState proc_arm_set(VecByte* vec_byte, ArmParameter* arm)
{
    uint8_t code;
    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 2, &code));
    switch (code)
    {
        case CMD_ARM_B2_STOP:
        {
            arm_set_tim(arm, arm->tim_current);
            return FNS_OK;
        }
        case CMD_ARM_B2_SET:
        {
            arm_set_pos(arm, vec_byte->data[vec_byte->head]);
            return FNS_OK;
        }
        default: break;
    }
    return FNS_NOT_FOUND;
}
#endif

FnState instant_recv_proc(VecByte* vec_byte)
{
    uint8_t code;
    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 0, &code));
    switch (code)
    {
        #ifdef PRINCIPAL_PROGRAM
        case CMD_VEHI_B0_CONTROL:
        {
            ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 1, &code));
            switch (code)
            {
                case CMD_VEHI_B1_VEHICLE:
                {
                    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 2, &code));
                    switch (code)
                    {
                        case CMD_VEHI_B2_MODE:
                        {
                            ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 3, &code));
                            switch (code)
                            {
                                case CMD_VEHI_B3_STOP:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    return FNS_OK;
                                }
                                case CMD_VEHI_B3_FOWARD:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_TRACK);
                                    return FNS_OK;
                                }
                                case CMD_VEHI_B3_BACKWARD:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_SEARCH);
                                    return FNS_OK;
                                }
                                default: break;
                            }
                            break;
                        }
                        case CMD_VEHI_B2_DIRECT:
                        {
                            ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 3, &code));
                            switch (code)
                            {
                                case CMD_VEHI_B3_STOP:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    vehicle_set_direct(VEHICLE_DIRECT_STOP);
                                    return FNS_OK;
                                }
                                case CMD_VEHI_B3_FOWARD:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    vehicle_set_direct(VEHICLE_DIRECT_FORWARD);
                                    return FNS_OK;
                                }
                                case CMD_VEHI_B3_BACKWARD:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    vehicle_set_direct(VEHICLE_DIRECT_BACKWARD);
                                    return FNS_OK;
                                }
                                case CMD_VEHI_B3_C_CLOCK:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    vehicle_set_direct(VEHICLE_DIRECT_C_CLOCKWISE);
                                    return FNS_OK;
                                }
                                case CMD_VEHI_B3_CLOCK:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    vehicle_set_direct(VEHICLE_DIRECT_CLOCKWISE);
                                    return FNS_OK;
                                }
                                default: break;
                            }
                            break;
                        }
                        case CMD_VEHI_B2_SPEED:
                        {
                            ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 3, &code));
                            vehicle_set_mode(VEHICLE_MODE_FREE);
                            vehicle_set_speed(code);
                            return FNS_OK;
                        }
                        default: break;
                    }
                    break;
                }
                case CMD_VEHI_B1_LEFT_MOTOR:
                {
                    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 2, &code));
                    MotorParameter* motor = &motor_left;
                    switch (code)
                    {
                        case CMD_VEHI_B2_MODE:
                        {
                            ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 3, &code));
                            switch (code)
                            {
                                case CMD_VEHI_B3_STOP:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    motor_set_state(motor, MOTOR_STATE_CONTROL);
                                    return FNS_OK;
                                }
                                case CMD_VEHI_B3_FOWARD:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    motor_set_state(motor, MOTOR_STATE_FREE);
                                    return FNS_OK;
                                }
                                case CMD_VEHI_B3_BACKWARD:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    motor_set_state(motor, MOTOR_STATE_SLOW);
                                    return FNS_OK;
                                }
                                default: break;
                            }
                            break;
                        }
                        case CMD_VEHI_B2_DIRECT:
                        {
                            ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 3, &code));
                            switch (code)
                            {
                                case CMD_VEHI_B3_STOP:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    motor_set_direct(motor, MOTOR_DIRECTION_STOP);
                                    return FNS_OK;
                                }
                                case CMD_VEHI_B3_FOWARD:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    motor_set_direct(motor, MOTOR_DIRECTION_CCLW);
                                    return FNS_OK;
                                }
                                case CMD_VEHI_B3_BACKWARD:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    motor_set_direct(motor, MOTOR_DIRECTION_CLW);
                                    return FNS_OK;
                                }
                                default: break;
                            }
                            break;
                        }
                        case CMD_VEHI_B2_SPEED:
                        {
                            ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 3, &code));
                            vehicle_set_mode(VEHICLE_MODE_FREE);
                            motor_set_rps_pcn(motor, code);
                            return FNS_OK;
                        }
                        default: break;
                    }
                    break;
                }
                case CMD_VEHI_B1_RIGHT_MOTOR:
                {
                    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 2, &code));
                    MotorParameter* motor = &motor_left;
                    switch (code)
                    {
                        case CMD_VEHI_B2_MODE:
                        {
                            ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 3, &code));
                            switch (code)
                            {
                                case CMD_VEHI_B3_STOP:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    motor_set_state(motor, MOTOR_STATE_CONTROL);
                                    return FNS_OK;
                                }
                                case CMD_VEHI_B3_FOWARD:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    motor_set_state(motor, MOTOR_STATE_FREE);
                                    return FNS_OK;
                                }
                                case CMD_VEHI_B3_BACKWARD:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    motor_set_state(motor, MOTOR_STATE_SLOW);
                                    return FNS_OK;
                                }
                                default: break;
                            }
                            break;
                        }
                        case CMD_VEHI_B2_DIRECT:
                        {
                            ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 3, &code));
                            switch (code)
                            {
                                case CMD_VEHI_B3_STOP:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    motor_set_direct(motor, MOTOR_DIRECTION_STOP);
                                    return FNS_OK;
                                }
                                case CMD_VEHI_B3_FOWARD:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    motor_set_direct(motor, MOTOR_DIRECTION_CLW);
                                    return FNS_OK;
                                }
                                case CMD_VEHI_B3_BACKWARD:
                                {
                                    vehicle_set_mode(VEHICLE_MODE_FREE);
                                    motor_set_direct(motor, MOTOR_DIRECTION_CCLW);
                                    return FNS_OK;
                                }
                                default: break;
                            }
                            break;
                        }
                        case CMD_VEHI_B2_SPEED:
                        {
                            ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 3, &code));
                            vehicle_set_mode(VEHICLE_MODE_FREE);
                            motor_set_rps_pcn(motor, code);
                            return FNS_OK;
                        }
                        default: break;
                    }
                    break;
                }
                default: break;
            }
            break;
        }
        #endif
        #ifdef ANCILLARY_PROGRAM
        case CMD_ARM_B0_CONTROL:
        {
            ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 1, &code));
            switch (code)
            {
                case CMD_ARM_B1_ARM:
                {
                    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 2, &code));
                    switch (code)
                    {
                        case CMD_ARM_B2_STOP:
                        {
                            arm_set_tim(&arm_bottom, arm_bottom.tim_current);
                            arm_set_tim(&arm_shoulder, arm_shoulder.tim_current);
                            arm_set_tim(&arm_elbow_btm, arm_elbow_btm.tim_current);
                            arm_set_tim(&arm_elbow_top, arm_elbow_top.tim_current);
                            arm_set_tim(&arm_wrist, arm_wrist.tim_current);
                            arm_set_tim(&arm_finger, arm_finger.tim_current);
                            return FNS_OK;
                        }
                        default: break;
                    }
                    break;
                }
                case CMD_ARM_B1_BOTTOM:
                {
                    return proc_arm_set(vec_byte, &arm_bottom);
                }
                case CMD_ARM_B1_SHOULDER:
                {
                    return proc_arm_set(vec_byte, &arm_shoulder);
                }
                case CMD_ARM_B1_ELBOW_BTM:
                {
                    return proc_arm_set(vec_byte, &arm_elbow_btm);
                }
                case CMD_ARM_B1_ELBOW_TOP:
                {
                    return proc_arm_set(vec_byte, &arm_elbow_top);
                }
                case CMD_ARM_B1_WRIST:
                {
                    return proc_arm_set(vec_byte, &arm_wrist);
                }
                case CMD_ARM_B1_FINGER:
                {
                    return proc_arm_set(vec_byte, &arm_finger);
                }
                default: break;
            }
            break;
        }
        #endif
        default: break;
    }
    return FNS_NOT_FOUND;
}

static UNUSED_FNC FnState pkt_transmit(void)
{
    vec_rm_all(&fdcan_trsm_buf);
    ERROR_CHECK_FNS_RETURN(fdcan_trcv_buf_pop(&fdcan_trsm_pkt_buf, &fdcan_trsm_buf, &fdcan_TxHeader.Identifier));
    fdcan_TxHeader.DataLength = fdcan_trsm_buf.len;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &fdcan_TxHeader, fdcan_trsm_buf.data);
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
        #else
        #ifdef PRINCIPAL_PROGRAM
        ERROR_CHECK_FNS_WRI_PUSH(pkt_left_speed(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_ARM_DATA_ID), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_right_speed(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_ARM_DATA_ID), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_left_duty(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_ARM_DATA_ID), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_right_duty(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_ARM_DATA_ID), vec_byte_free(&vec_byte));
        #endif
        #ifdef ANCILLARY_PROGRAM
        ERROR_CHECK_FNS_WRI_PUSH(pkt_arm_bottom(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_ARM_DATA_ID), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_arm_shoulder(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_ARM_DATA_ID), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_arm_elbow_btm(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_ARM_DATA_ID), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_arm_elbow_top(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_ARM_DATA_ID), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_arm_wrist(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_ARM_DATA_ID), vec_byte_free(&vec_byte));
        ERROR_CHECK_FNS_WRI_PUSH(pkt_arm_finger(&vec_byte),
            fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_ARM_DATA_ID), vec_byte_free(&vec_byte));
        #endif
        #endif
    }
    vec_byte_free(&vec_byte);
    return FNS_OK;
}

static FnState recv_pkt_proc_inner(VecByte* vec_byte)
{
    uint8_t code;
    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 0, &code));
    switch (code)
    {
        case CMD_DATA_B0_STOP:
        {
            fdacn_data_trsm_ready = false;
            return FNS_OK;
        }
        case CMD_DATA_B0_START:
        {
            fdacn_data_trsm_ready = true;
            return FNS_OK;
        }
        #ifdef ANCILLARY_PROGRAM
        case CMD_RFID_B0_CONTROL:
        {
            ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 1, &code));
            switch (code)
            {
                case CMD_RFID_B1_SELECT:
                {
                    uint8_t secter, block;
                    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 2, &secter));
                    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 3, &block));
                    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 4, &code));
                    switch (code)
                    {
                        case CMD_RFID_B4_ONLY_SET:
                        {
                            return rfid_trcv_buf_setaddr(&rfid_trsm_buf, secter, block, 0);
                        }
                        case CMD_RFID_B4_WRITE:
                        {
                            return rfid_trcv_buf_setaddr(&rfid_trsm_buf, secter, block, 1);
                        }
                        default: break;
                    }
                    break;
                }
                case CMD_RFID_B1_INP_DATA:
                {
                    ERROR_CHECK_FNS_RETURN(vec_byte_get_byte(vec_byte, 2, &code));
                    ERROR_CHECK_FNS_RETURN(vec_rm_range(vec_byte, 0, 3));
                    if (vec_byte->len < 4) return FNS_NOT_FOUND;
                    return rfid_trcv_buf_setdata(&rfid_trsm_buf, code * 4, vec_byte->data + vec_byte->head, 4);
                }
                default: break;
            }
            break;
        }
        #endif
        default: break;
    }
    return FNS_NOT_FOUND;
}

static UNUSED_FNC FnState recv_pkt_proc(size_t count)
{
    VecByte vec_byte;
    uint32_t id;
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&vec_byte, UART_VEC_BYTE_CAP));
    for (size_t i = 0; i < count; i++)
    {
        if (ERROR_CHECK_FNS_RAW(fdcan_trcv_buf_pop(&fdcan_recv_pkt_buf, &vec_byte, &id))) break;
        last_error = recv_pkt_proc_inner(&vec_byte);
    }
    vec_byte_free(&vec_byte);
    return FNS_OK;
}

void StartFdCanTask(void *argument)
{
    #ifdef DISABLE_FDCAN
    osThreadExit();
    #else
    ERROR_CHECK_FNS_HANDLE(vec_byte_new(&fdcan_trsm_buf, 8));
    ERROR_CHECK_FNS_HANDLE(fdcan_trcv_buf_setup(&fdcan_trsm_pkt_buf, FDCAN_TRCV_BUF_CAP, FDCAN_VEC_BYTE_CAP));
    ERROR_CHECK_FNS_HANDLE(vec_byte_new(&fdcan_recv0_buf, 8));
    ERROR_CHECK_FNS_HANDLE(vec_byte_new(&fdcan_recv1_buf, 8));
    ERROR_CHECK_FNS_HANDLE(fdcan_trcv_buf_setup(&fdcan_recv_pkt_buf, FDCAN_TRCV_BUF_CAP, FDCAN_VEC_BYTE_CAP));
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE));
    FDCAN_FilterTypeDef sFilter0 = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,
        .FilterType = FDCAN_FILTER_RANGE,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP,
        .FilterID1 = FDCAN_FILTER0_ID_MIN,
        .FilterID2 = FDCAN_FILTER0_ID_MAX,
    };
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter0));
    FDCAN_FilterTypeDef sFilter1 = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 1,
        .FilterType = FDCAN_FILTER_RANGE,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO1,
        .FilterID1 = FDCAN_FILTER1_ID_MIN,
        .FilterID2 = FDCAN_FILTER1_ID_MAX,
    };
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter1));
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_Start(&hfdcan1));
    ERROR_CHECK_HAL_HANDLE(
        HAL_FDCAN_ActivateNotification(&hfdcan1,
              FDCAN_IT_BUS_OFF
            | FDCAN_IT_TX_EVT_FIFO_NEW_DATA
            | FDCAN_IT_TX_EVT_FIFO_FULL
            | FDCAN_IT_TX_EVT_FIFO_ELT_LOST
        , 0)
    );
    ERROR_CHECK_HAL_HANDLE(
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE,
              FDCAN_TX_BUFFER0
            | FDCAN_TX_BUFFER1
            | FDCAN_TX_BUFFER2
        )
    );
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0));
    ERROR_CHECK_HAL_HANDLE(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0));
    size_t tick = 0;
    for(;;)
    {
        if (fdcan_bus_off)
        {
            fdcan_bus_off = false;
            HAL_FDCAN_Stop(&hfdcan1);
            HAL_FDCAN_Start(&hfdcan1);
        }
        last_error = pkt_transmit();
        last_error = recv_pkt_proc(5);
        if (tick % 50 == 0)
        {
            tick = 0;
            last_error = trsm_pkt_proc();
        }
        osDelay(10);
        tick++;
    }
    #endif
}
