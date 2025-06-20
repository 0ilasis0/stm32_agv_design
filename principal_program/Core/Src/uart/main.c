#include "uart/main.h"
#include <string.h>
#include "cmsis_os.h"
#include "usart.h"
#include "main/config.h"
#include "main/mcu_const.h"

static Vec_U8 uart_dma_tr_buf;
static Vec_U8 uart_dma_rv_buf;

UartTrcvBuf uart_tr_pkt_buf;
UartTrcvBuf uart_rv_pkt_buf;

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
            uart_dma_rv_buf.len = Size;
            if (
                (uart_dma_rv_buf.data[0] == UART_START_CODE)
                && (uart_dma_rv_buf.data[uart_dma_rv_buf.len-1] == UART_END_CODE)
            ) {
                uart_trcv_buf_push(&uart_rv_pkt_buf, &uart_dma_rv_buf);
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_dma_rv_buf.data, uart_dma_rv_buf.capacity);
    }
}

float f32_test = 0;
uint16_t u16_test = 0;

/**
 * @brief 將右側馬達當前速度回應至資料向量
 *        Push current right motor speed response into byte vector
 *
 * @param vec_u8 指向要寫入資料的 Vec_U8 (input/output vector to receive response data)
 * @return void
 */
static FnState rspdw(void)
{
    f32_test++;
    Vec_U8 vec_u8;
    FNS_ERROR_CHECK(vec_u8_new(&vec_u8, UART_VEC_MAX));
    vec_u8_push_byte(&vec_u8, CMD_CODE_DATA_TRRE);
    vec_u8_push(&vec_u8, CMD_RIGHT_SPEED_STORE, sizeof(CMD_RIGHT_SPEED_STORE));
    // vec_u8_push_f32(&vec_u8, motor_right.speed_present);
    vec_u8_push_f32(&vec_u8, f32_test);
    uart_trcv_buf_push(&uart_tr_pkt_buf, &vec_u8);
    vec_u8_free(&vec_u8);
    return FNS_OK;
}

/**
 * @brief 將右側馬達 ADC 值回應至資料向量
 *        Push right motor ADC value response into byte vector
 *
 * @param vec_u8 指向要寫入資料的 Vec_U8 (input/output vector to receive ADC data)
 * @return void
 */
static FnState radcw(void)
{
    u16_test++;
    Vec_U8 vec_u8;
    FNS_ERROR_CHECK(vec_u8_new(&vec_u8, UART_VEC_MAX));
    vec_u8_push_byte(&vec_u8, CMD_CODE_DATA_TRRE);
    vec_u8_push(&vec_u8, CMD_RIGHT_ADC_STORE, sizeof(CMD_RIGHT_ADC_STORE));
    // vec_u8_push_u16(&vec_u8, motor_right.adc_value);
    vec_u8_push_u16(&vec_u8, u16_test);
    uart_trcv_buf_push(&uart_tr_pkt_buf, &vec_u8);
    vec_u8_free(&vec_u8);
    return FNS_OK;
}

static FnState uart_transmit(void)
{
    if (HAL_DMA_GetState(huart3.hdmatx) == HAL_DMA_STATE_BUSY) return FNS_FAIL;
    vec_u8_rm_all(&uart_dma_tr_buf);
    FNS_ERROR_CHECK(vec_u8_push_byte(&uart_dma_tr_buf, UART_START_CODE));
    FNS_ERROR_CHECK(uart_trcv_buf_pop(&uart_tr_pkt_buf, &uart_dma_tr_buf));
    FNS_ERROR_CHECK(vec_u8_push_byte(&uart_dma_tr_buf, UART_END_CODE));
    HAL_UART_Transmit_DMA(&huart3, uart_dma_tr_buf.data, uart_dma_tr_buf.len);
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
static FnState uart_re_pkt_proc_data_store(Vec_U8 *vec_u8)
{
    bool data_proc_flag = true;
    while (data_proc_flag)
    {
        data_proc_flag = false;
        if (vec_u8_starts_with(vec_u8, CMD_RIGHT_SPEED_STOP, sizeof(CMD_RIGHT_SPEED_STOP)) == FNS_OK)
        {
            FNS_ERROR_CHECK(vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_SPEED_STOP)));
            data_proc_flag = true;
            transceive_flags.right_speed = false;
        }
        else if (vec_u8_starts_with(vec_u8, CMD_RIGHT_SPEED_ONCE, sizeof(CMD_RIGHT_SPEED_ONCE)) == FNS_OK)
        {
            FNS_ERROR_CHECK(vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_SPEED_ONCE)));
            data_proc_flag = true;
            rspdw();
        }
        else if (vec_u8_starts_with(vec_u8, CMD_RIGHT_SPEED_START, sizeof(CMD_RIGHT_SPEED_START)) == FNS_OK)
        {
            FNS_ERROR_CHECK(vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_SPEED_START)));
            data_proc_flag = true;
            transceive_flags.right_speed = true;
        }
        else if (vec_u8_starts_with(vec_u8, CMD_RIGHT_ADC_STOP, sizeof(CMD_RIGHT_ADC_STOP)) == FNS_OK)
        {
            FNS_ERROR_CHECK(vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_ADC_STOP)));
            data_proc_flag = true;
            transceive_flags.right_adc = false;
        }
        else if (vec_u8_starts_with(vec_u8, CMD_RIGHT_ADC_ONCE, sizeof(CMD_RIGHT_ADC_ONCE)) == FNS_OK)
        {
            FNS_ERROR_CHECK(vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_ADC_ONCE)));
            data_proc_flag = true;
            radcw();
        }
        else if (vec_u8_starts_with(vec_u8, CMD_RIGHT_ADC_START, sizeof(CMD_RIGHT_ADC_START)) == FNS_OK)
        {
            FNS_ERROR_CHECK(vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_ADC_START)));
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
static FnState uart_re_pkt_proc(void)
{
    Vec_U8 vec_u8;
    FNS_ERROR_CHECK(vec_u8_new(&vec_u8, UART_VEC_MAX));
    vec_u8_rm_all(&vec_u8);
    if (uart_trcv_buf_pop(&uart_rv_pkt_buf, &vec_u8) == FNS_OK) {
        uint8_t code = vec_u8.data[vec_u8.head];
        vec_u8_rm_range(&vec_u8, 0, 1);
        switch (code)
        {
            case CMD_CODE_DATA_TRRE:
                uart_re_pkt_proc_data_store(&vec_u8);
                break;
            default:
                break;
        }
    }
    vec_u8_free(&vec_u8);
    return FNS_OK;
}

static FnState uart_setup(void)
{
    // Tx:PB9(R5) Rx:PB11(R18)
    FNS_ERROR_CHECK(vec_u8_new(&uart_dma_tr_buf, UART_VEC_MAX + 2));
    FNS_ERROR_CHECK(vec_u8_new(&uart_dma_rv_buf, UART_VEC_MAX + 2));
    FNS_ERROR_CHECK(uart_trcv_buf_setup(&uart_tr_pkt_buf));
    FNS_ERROR_CHECK(uart_trcv_buf_setup(&uart_rv_pkt_buf));
    return FNS_OK;
}

#ifndef DISABLE_UART
void StartUartTask(void *argument)
{
    if (uart_setup() != FNS_OK) return;
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
#ifndef DISABLE_UART_RECV
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_dma_rv_buf.data, uart_dma_rv_buf.capacity);
#endif
    uint8_t tick = 0;
    for(;;)
    {
#ifndef DISABLE_UART_TRSM
        uart_transmit();
#endif
#ifndef DISABLE_UART_RECV
        uart_re_pkt_proc();
#endif
        if (tick % 20 == 0)
        {
            tick = 0;
#ifndef DISABLE_UART_TRSM
            uart_tr_pkt_proc();
#endif
        }
        osDelay(50);
        tick++;
    }
}
#endif
