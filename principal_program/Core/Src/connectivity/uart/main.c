#include "connectivity/uart/main.h"
#include <string.h>
#include "cmsis_os.h"
#include "usart.h"
#include "main/config.h"
#include "main/mcu_const.h"

static VecByte uart_tr_buf;
static VecByte uart_rv_buf;

ByteTrcvBuf uart_tr_pkt_buf;
ByteTrcvBuf uart_rv_pkt_buf;

TransceiveFlags transceive_flags = {0};

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    if (huart->Instance == USART3)
    {
        if (Size != 0)
        {
            uart_rv_buf.len = Size;
            if (
                (uart_rv_buf.data[0] == UART_START_CODE)
                && (uart_rv_buf.data[uart_rv_buf.len-1] == UART_END_CODE)
            ) {
                connect_trcv_buf_push(&uart_rv_pkt_buf, &uart_rv_buf);
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_rv_buf.data, uart_rv_buf.cap);
    }
}

float f32_test = 0;
uint16_t u16_test = 0;

static FnState rspdw(void)
{
    f32_test++;
    VecByte vec_u8;
    FNS_ERROR_CHECK(vec_byte_new(&vec_u8, UART_VEC_MAX));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_CODE_DATA_TRRE), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push(&vec_u8, CMD_RIGHT_SPEED_STORE, sizeof(CMD_RIGHT_SPEED_STORE)), vec_byte_free(&vec_u8));
    // FNS_ERROR_CHECK_CLEAN(vec_byte_push_f32(&vec_u8, motor_right.speed_present), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_f32(&vec_u8, f32_test), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(connect_trcv_buf_push(&uart_tr_pkt_buf, &vec_u8), vec_byte_free(&vec_u8));
    vec_byte_free(&vec_u8);
    return FNS_OK;
}

static FnState radcw(void)
{
    u16_test++;
    VecByte vec_u8;
    FNS_ERROR_CHECK(vec_byte_new(&vec_u8, UART_VEC_MAX));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_CODE_DATA_TRRE), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push(&vec_u8, CMD_RIGHT_ADC_STORE, sizeof(CMD_RIGHT_ADC_STORE)), vec_byte_free(&vec_u8));
    // FNS_ERROR_CHECK_CLEAN(vec_byte_push_u16(&vec_u8, motor_right.adc_value), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_u16(&vec_u8, u16_test), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(connect_trcv_buf_push(&uart_tr_pkt_buf, &vec_u8), vec_byte_free(&vec_u8));
    vec_byte_free(&vec_u8);
    return FNS_OK;
}

static FnState uart_transmit(void)
{
    if (HAL_DMA_GetState(huart3.hdmatx) == HAL_DMA_STATE_BUSY) return FNS_FAIL;
    vec_rm_all(&uart_tr_buf);
    FNS_ERROR_CHECK(vec_byte_push_byte(&uart_tr_buf, UART_START_CODE));
    FNS_ERROR_CHECK(connect_trcv_buf_pop(&uart_tr_pkt_buf, &uart_tr_buf));
    FNS_ERROR_CHECK(vec_byte_push_byte(&uart_tr_buf, UART_END_CODE));
    HAL_UART_Transmit_DMA(&huart3, uart_tr_buf.data, uart_tr_buf.len);
    return FNS_OK;
}

/**
 * @brief 組合並傳輸封包至傳輸緩衝區
 *        Assemble and transmit packet into transfer buffer
 *
 * @note 根據 transceive_flags 決定回應內容
 *
 * @return void
 */
static FnState uart_tr_pkt_proc(void)
{
    // if (transceive_flags.right_speed) {
    //     rspdw();
    // }
    // if (transceive_flags.right_adc) {
    //     radcw();
    // }
    rspdw();
    radcw();
    return FNS_OK;
}

/**
 * @brief 處理接收命令並存儲/回應資料
 *        Process received commands and store or respond data
 *
 * @param vec_u8 指向去除命令碼後的資料向量 (input vector without command code)
 * @return void
 */
static FnState uart_re_pkt_proc_data_store(VecByte *vec_u8)
{
    bool data_proc_flag = true;
    while (data_proc_flag)
    {
        data_proc_flag = false;
        if (vec_byte_starts_with(vec_u8, CMD_RIGHT_SPEED_STOP, sizeof(CMD_RIGHT_SPEED_STOP)) == FNS_OK)
        {
            FNS_ERROR_CHECK(vec_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_SPEED_STOP)));
            data_proc_flag = true;
            transceive_flags.right_speed = false;
        }
        else if (vec_byte_starts_with(vec_u8, CMD_RIGHT_SPEED_ONCE, sizeof(CMD_RIGHT_SPEED_ONCE)) == FNS_OK)
        {
            FNS_ERROR_CHECK(vec_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_SPEED_ONCE)));
            data_proc_flag = true;
            rspdw();
        }
        else if (vec_byte_starts_with(vec_u8, CMD_RIGHT_SPEED_START, sizeof(CMD_RIGHT_SPEED_START)) == FNS_OK)
        {
            FNS_ERROR_CHECK(vec_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_SPEED_START)));
            data_proc_flag = true;
            transceive_flags.right_speed = true;
        }
        else if (vec_byte_starts_with(vec_u8, CMD_RIGHT_ADC_STOP, sizeof(CMD_RIGHT_ADC_STOP)) == FNS_OK)
        {
            FNS_ERROR_CHECK(vec_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_ADC_STOP)));
            data_proc_flag = true;
            transceive_flags.right_adc = false;
        }
        else if (vec_byte_starts_with(vec_u8, CMD_RIGHT_ADC_ONCE, sizeof(CMD_RIGHT_ADC_ONCE)) == FNS_OK)
        {
            FNS_ERROR_CHECK(vec_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_ADC_ONCE)));
            data_proc_flag = true;
            radcw();
        }
        else if (vec_byte_starts_with(vec_u8, CMD_RIGHT_ADC_START, sizeof(CMD_RIGHT_ADC_START)) == FNS_OK)
        {
            FNS_ERROR_CHECK(vec_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_ADC_START)));
            data_proc_flag = true;
            transceive_flags.right_adc = true;
        }
    }
    return FNS_OK;
}

/**
 * @brief 從接收緩衝區反覆讀取封包並處理
 *        Pop vecs from receive buffer and process them
 *
 * @param count 單次最大處理封包數量 (input maximum number of vecs to process per time)
 * @return void
 */
static FnState uart_re_pkt_proc(size_t count)
{
    VecByte vec_u8;
    FNS_ERROR_CHECK(vec_byte_new(&vec_u8, UART_VEC_MAX));
    for (size_t i = 0; i < count; i++)
    {
        FNS_ERROR_CHECK_CLEAN(connect_trcv_buf_pop(&uart_rv_pkt_buf, &vec_u8), vec_byte_free(&vec_u8));
        uint8_t code = vec_u8.data[vec_u8.head];
        vec_rm_range(&vec_u8, 0, 1);
        switch (code)
        {
            case CMD_CODE_DATA_TRRE:
                uart_re_pkt_proc_data_store(&vec_u8);
                break;
            default:
                break;
        }
    }
    vec_byte_free(&vec_u8);
    return FNS_OK;
}

static FnState uart_setup(void)
{
    // Tx:PB9(R5) Rx:PB11(R18)
    FNS_ERROR_CHECK(vec_byte_new(&uart_tr_buf, UART_VEC_MAX + 2));
    FNS_ERROR_CHECK(vec_byte_new(&uart_rv_buf, UART_VEC_MAX + 2));
    FNS_ERROR_CHECK(connect_trcv_buf_setup(&uart_tr_pkt_buf, UART_TRCV_BUF_CAP, UART_VEC_MAX));
    FNS_ERROR_CHECK(connect_trcv_buf_setup(&uart_rv_pkt_buf, UART_TRCV_BUF_CAP, UART_VEC_MAX));
    return FNS_OK;
}

#ifndef DISABLE_UART
void StartUartTask(void *argument)
{
    if (uart_setup() != FNS_OK) {
        for (;;)
        {
            osDelay(1000);
        }
    }
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
#ifndef DISABLE_UART_RECV
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_rv_buf.data, uart_rv_buf.cap);
#endif
    size_t tick = 0;
    for (;;)
    {
#ifndef DISABLE_UART_TRSM
        uart_transmit();
#endif
#ifndef DISABLE_UART_RECV
        uart_re_pkt_proc(5);
#endif
        if (tick % 100 == 0)
        {
            tick = 0;
#ifndef DISABLE_UART_TRSM
            uart_tr_pkt_proc();
#endif
        }
        osDelay(10);
        tick++;
    }
}
#endif
