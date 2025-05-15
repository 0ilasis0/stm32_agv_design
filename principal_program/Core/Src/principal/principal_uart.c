#include "principal/principal_uart.h"
#include <string.h>
#include "principal/packet.h"
#include "usart.h"
#include "principal/motor.h"

// UART 初始化標誌
// UART initialization flag
bool uart_init = 0;

// 接收緩衝區，大小為 PACKET_MAX_SIZE
// Receive buffer of size PACKET_MAX_SIZE
uint8_t uart_buffer_r[PACKET_MAX_SIZE];

// 傳輸和接收循環緩衝區
// Transmission and reception ring buffers
TrReBuffer transfer_buffer;
TrReBuffer receive_buffer;

// 設置 UART，清零緩衝區並啟用 DMA 到空閒中斷接收
// Configure UART: clear buffer and enable DMA reception on IDLE interrupt
void uart_setup(void) {
    memset(uart_buffer_r, 0, sizeof(uart_buffer_r));
    transfer_buffer = tr_re_buffer_new();
    receive_buffer = tr_re_buffer_new();
    // Rx:PB11(r18) Tx:PB9(r5)
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_buffer_r, PACKET_MAX_SIZE);
}

// UART3 中斷處理前置函數
// Pre-handler for UART3 interrupt
void HYCodes_UART3_IRQHandler_Before(void) {
    if (!__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE)) return;
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);

    uint16_t size = PACKET_MAX_SIZE - huart3.hdmarx->Instance->CNDTR;
    uart_RxCNDTR_reset(&huart3);

    HAL_UARTEx_RxEventCallback(&huart3, size);
}

// 重置 DMA 的 CNDTR 寄存器並重新啟用空閒中斷
// Reset DMA CNDTR register and re-enable IDLE interrupt
void uart_RxCNDTR_reset(UART_HandleTypeDef *huart) {
    __HAL_DMA_DISABLE(huart->hdmarx);
    huart->hdmarx->Instance->CNDTR = PACKET_MAX_SIZE;
    __HAL_DMA_ENABLE(huart->hdmarx);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

// UART 傳輸完成回調（可擴展）
// UART TX complete callback (expandable)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        // 可在此處添加傳輸完成處理
        // Transmission complete handling can be added here
    }
}

// UART 空閒接收事件回調函數
// UART IDLE reception event callback
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART3) {
        // 初次初始化階段，不處理數據
        // Initial setup phase: do not process data
        if (!uart_init) {
            uart_init = true;
            return;
        }

        VecU8 re_vec_u8 = vec_u8_new();
        vec_u8_extend_inner(&re_vec_u8, uart_buffer_r, Size);
        memset(uart_buffer_r, 0, PACKET_MAX_SIZE);
        UartPacket re_packet = uart_packet_pack(&re_vec_u8);
        tr_re_buffer_push(&receive_buffer, &re_packet);

        // 構建新的速度數據包並推送到傳輸環
        // Construct new speed packet and push to transfer ring buffer
        VecU8 new_vec = vec_u8_new();
        vec_u8_push(&new_vec, 0x20);
        VecU8 speed = u32_to_u8(motor_right.adc_value);
        vec_u8_extend_inner(&new_vec, speed.data, speed.length);
        UartPacket new_packet = uart_packet_new(&new_vec);
        tr_re_buffer_push(&transfer_buffer, &new_packet);

        // 從傳輸環中彈出數據並發送
        // Pop packet from transfer buffer and transmit
        UartPacket tr_packet = tr_re_buffer_pop(&transfer_buffer);
        VecU8 tr_vec_u8 = uart_packet_unpack(&tr_packet);
        HAL_UART_Transmit_DMA(huart, tr_vec_u8.data, tr_vec_u8.length);

        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_buffer_r, PACKET_MAX_SIZE);
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    }
}
