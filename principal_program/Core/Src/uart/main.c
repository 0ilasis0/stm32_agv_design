#include "uart/main.h"
#include <string.h>
#include "cmsis_os.h"
#include "usart.h"
#include "main/mcu_const.h"

VecU8 uart_dma_tr_buf = VEC_U8_NEW();
VecU8 uart_dma_rv_buf = VEC_U8_NEW();

/**
 * @brief 全域傳輸/接收緩衝區
 *        Global transmit/receive ring buffer
 */
UartTrcvBuf uart_tr_pkt_buf;
UartTrcvBuf uart_rv_pkt_buf;

TransceiveFlags transceive_flags = {0};

/**
 * @brief UART 傳輸完成回調：移除已傳輸封包
 *        UART TX complete callback: pop transmitted packet
 *
 * @param huart 指向 UART 處理器結構體的指標 (input UART handle pointer)
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {}
}

/**
 * @brief UART 空閒接收事件回調：處理接收資料並推入接收緩衝
 *        UART IDLE reception event callback: process received data and push to receive buffer
 *
 * @param huart 指向 UART 處理器結構體的指標 (input UART handle pointer)
 * @param Size 接收到的資料長度 (input number of received bytes)
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    if (huart->Instance == USART3)
    {
        if (Size == 0)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_dma_rv_buf.data, VECU8_MAX_CAPACITY);
            return;
        }
        uart_dma_rv_buf.len = Size;
        UartPacket packet = UART_PKT_NEW();
        if (uart_pkt_pack(&packet, &uart_dma_rv_buf) != FNS_OK)
        {
            vec_u8_rm_all(&uart_dma_rv_buf);
            uart_trcv_buf_push(&uart_rv_pkt_buf, &packet);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_dma_rv_buf.data, VECU8_MAX_CAPACITY);
    }
}

float f32_test = 0;
uint16_t u16_test = 0;

/**
 * @brief 將右側馬達當前速度回應至資料向量
 *        Push current right motor speed response into byte vector
 *
 * @param vec_u8 指向要寫入資料的 VecU8 (input/output vector to receive response data)
 * @return void
 */
static FnState rspdw(void)
{
    f32_test++;
    VecU8 vec_u8 = VEC_U8_NEW();
    FNS_ERROR_CHECK(vec_u8_push_byte(&vec_u8, CMD_CODE_DATA_TRRE));
    FNS_ERROR_CHECK(vec_u8_push(&vec_u8, CMD_RIGHT_SPEED_STORE, sizeof(CMD_RIGHT_SPEED_STORE)));
    // FNS_ERROR_CHECK(vec_u8_push_f32(&vec_u8, motor_right.speed_present));
    FNS_ERROR_CHECK(vec_u8_push_f32(&vec_u8, f32_test));
    UartPacket packet = UART_PKT_NEW();
    FNS_ERROR_CHECK(uart_pkt_add_data(&packet, &vec_u8));
    FNS_ERROR_CHECK(uart_trcv_buf_push(&uart_tr_pkt_buf, &packet));
    return FNS_OK;
}

/**
 * @brief 將右側馬達 ADC 值回應至資料向量
 *        Push right motor ADC value response into byte vector
 *
 * @param vec_u8 指向要寫入資料的 VecU8 (input/output vector to receive ADC data)
 * @return void
 */
static FnState radcw(void)
{
    u16_test++;
    VecU8 vec_u8 = VEC_U8_NEW();
    FNS_ERROR_CHECK(vec_u8_push_byte(&vec_u8, CMD_CODE_DATA_TRRE));
    FNS_ERROR_CHECK(vec_u8_push(&vec_u8, CMD_RIGHT_ADC_STORE, sizeof(CMD_RIGHT_ADC_STORE)));
    // FNS_ERROR_CHECK(vec_u8_push_u16(&vec_u8, motor_right.adc_value));
    FNS_ERROR_CHECK(vec_u8_push_u16(&vec_u8, u16_test));
    UartPacket packet = UART_PKT_NEW();
    FNS_ERROR_CHECK(uart_pkt_add_data(&packet, &vec_u8));
    FNS_ERROR_CHECK(uart_trcv_buf_push(&uart_tr_pkt_buf, &packet));
    return FNS_OK;
}

static inline FnState uart_transmit(void)
{
    if (HAL_DMA_GetState(huart3.hdmatx) == HAL_DMA_STATE_BUSY) return FNS_ERROR;
    UartPacket packet = UART_PKT_NEW();
    FNS_ERROR_CHECK(uart_trcv_buf_pop(&uart_tr_pkt_buf, &packet));
    FNS_ERROR_CHECK(uart_pkt_unpack(&packet, &uart_dma_tr_buf));
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
static inline FnState uart_tr_pkt_proc(void)
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
static FnState uart_re_pkt_proc_data_store(VecU8 *vec_u8)
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
 *        Pop packets from receive buffer and process them
 *
 * @param count 單次最大處理封包數量 (input maximum number of packets to process per time)
 * @return void
 */
static inline FnState uart_re_pkt_proc()
{
    uint8_t i;
    for (i = 0; i < 5; i++)
    {
        UartPacket packet = UART_PKT_NEW();
        FNS_ERROR_CHECK(uart_trcv_buf_pop(&uart_rv_pkt_buf, &packet));
        VecU8 vec_u8 = VEC_U8_NEW();
        FNS_ERROR_CHECK(uart_pkt_get_data(&packet, &vec_u8));
        uint8_t code = vec_u8.data[vec_u8.head];
        FNS_ERROR_CHECK(vec_u8_rm_range(&vec_u8, 0, 1));
        switch (code)
        {
            case CMD_CODE_DATA_TRRE:
                FNS_ERROR_CHECK(uart_re_pkt_proc_data_store(&vec_u8));
                break;
            default:
                break;
        }
    }
    return FNS_OK;
}

void StartUartTask(void *argument)
{
    // Tx:PB9(R5) Rx:PB11(R18)
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_dma_rv_buf.data, VECU8_MAX_CAPACITY);
    uint16_t uart_task_tick = 0;
    for(;;)
    {
        if (uart_task_tick % 50 == 0)
        {
            uart_transmit();
            uart_re_pkt_proc();
        }
        if (uart_task_tick % 1000 == 0)
        {
            uart_tr_pkt_proc();
        }
        if (uart_task_tick % 10000 == 0)
        {
            uart_task_tick = 0;
        }
        osDelay(1);
        uart_task_tick++;
    }
}
