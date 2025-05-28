#include "user/uart_mod.h"
#include <string.h>
#include "usart.h"
#include "user/uart_packet_proc_mod.h"

/**
 * @brief UART 初始化完成旗標
 *        UART initialization flag
 *
 * @details 用於跳過第一次 DMA IDLE 中斷 (Used to skip the first DMA IDLE interrupt)
 */
bool uart_init = 0;

/**
 * @brief 傳輸/接收操作旗標
 *        Transmit/receive operation flags
 *
 * @details 控制資料處理流程 (Control data processing flow)
 */
TransceiveFlags transceive_flags = {0};

/**
 * @brief UART 接收 DMA 緩衝區
 *        UART receive DMA buffer
 *
 * @details 大小為 PACKET_MAX_SIZE，用於儲存 DMA 接收的原始資料
 *          Size is PACKET_MAX_SIZE; used to store raw data received by DMA
 */
uint8_t uart_receive_buffer[PACKET_MAX_SIZE];

/**
 * @brief 設置 UART，清零接收緩衝並啟用 DMA 接收於 IDLE 中斷
 *        Configure UART: clear receive buffer and enable DMA reception on IDLE interrupt
 *
 * @return void
 */
void uart_setup(void) {
    memset(uart_receive_buffer, 0, sizeof(uart_receive_buffer));
    // Rx:PB11(R18) Tx:PB9(R5)
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_receive_buffer, PACKET_MAX_SIZE);
}

/**
 * @brief UART3 中斷前置處理：檢查並清除 IDLE 標誌，重新配置 DMA
 *        Pre-handler for UART3 interrupt: check and clear IDLE flag, reconfigure DMA
 *
 * @return void
 */
void USER_UART3_IRQHandler_Before(void) {
    UART_HandleTypeDef *huart = &huart3;
    if (!__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) return;
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    uint16_t size = PACKET_MAX_SIZE - huart->hdmarx->Instance->CNDTR;
    __HAL_DMA_DISABLE(huart->hdmarx);
    huart->hdmarx->Instance->CNDTR = PACKET_MAX_SIZE;
    __HAL_DMA_ENABLE(huart->hdmarx);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UARTEx_RxEventCallback(huart, size);
}

/**
 * @brief UART 傳輸完成回調：移除已傳輸封包
 *        UART TX complete callback: pop transmitted packet
 *
 * @param huart 指向 UART 處理器結構體的指標 (input UART handle pointer)
 * @return void
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        transceive_buffer_pop_secondHalf(&transfer_buffer);
    }
}

/**
 * @brief UART 空閒接收事件回調：處理接收資料並推入接收緩衝
 *        UART IDLE reception event callback: process received data and push to receive buffer
 *
 * @param huart 指向 UART 處理器結構體的指標 (input UART handle pointer)
 * @param Size 接收到的資料長度 (input number of received bytes)
 * @return void
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART3) {
        if (!uart_init) {
            uart_init = true;
            return;
        }
        VecU8 re_vec_u8 = {0};
        vec_u8_push(&re_vec_u8, uart_receive_buffer, Size);
        memset(uart_receive_buffer, 0, PACKET_MAX_SIZE);
        UartPacket re_packet;
        if (!uart_packet_pack(&re_vec_u8, &re_packet)) {
            return;
        }
        transceive_buffer_push(&receive_buffer, &re_packet);

        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_receive_buffer, PACKET_MAX_SIZE);
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    }
}

/**
 * @brief 發送下一筆 UART 封包至 DMA
 *        Transmit next UART packet via DMA
 *
 * @return void
 */
void uart_transmit(void) {
    UartPacket packet;
    if (!transceive_buffer_pop_firstHalf(&transfer_buffer, &packet)) {
        return;
    }
    VecU8 tr_vec_u8 = uart_packet_unpack(&packet);
    HAL_UART_Transmit_DMA(&huart3, tr_vec_u8.data, tr_vec_u8.length);
}
