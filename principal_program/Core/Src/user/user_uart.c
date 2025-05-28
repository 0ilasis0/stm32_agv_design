#include "user/user_uart.h"
#include <string.h>
#include "usart.h"
#include "user/packet_mod.h"

bool uart_init = 0;

TrReFlags transceive_flags = {0};

/**
  * 接收緩衝區，大小為 PACKET_MAX_SIZE
  * 
  * Receive buffer of size PACKET_MAX_SIZE
  */
uint8_t uart_receive_buffer[PACKET_MAX_SIZE];

/**
  * 設置 UART，清零緩衝區並啟用 DMA 到空閒中斷接收
  * 
  * Configure UART by clearing buffer and enabling DMA reception on IDLE interrupt
  */
void uart_setup(void) {
    memset(uart_receive_buffer, 0, sizeof(uart_receive_buffer));
    // Rx:PB11(R18) Tx:PB9(R5)
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_receive_buffer, PACKET_MAX_SIZE);
}

/**
  * UART3 中斷前置處理函式，檢查並清除 IDLE 標誌
  * 
  * Pre-handler for UART3 interrupt: check and clear the IDLE flag
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
  * UART 傳輸完成回調函式，可擴展使用者處理
  * 
  * UART TX complete callback; can be extended for user handling
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        trce_buffer_pop_secondHalf(&transfer_buffer);
    }
}

/**
  * UART 空閒接收事件回調函式，處理接收與傳輸環緩衝
  * 
  * UART IDLE reception event callback: handle receive and transfer ring buffers
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
        UartPacket re_packet = uart_packet_pack(&re_vec_u8);
        trce_buffer_push(&receive_buffer, &re_packet);

        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_receive_buffer, PACKET_MAX_SIZE);
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    }
}

void uart_packet_send(void) {
    if (transfer_buffer.length == 0) return;
    UartPacket tr_packet = trce_buffer_pop_firstHalf(&transfer_buffer);
    VecU8 tr_vec_u8 = uart_packet_unpack(&tr_packet);
    HAL_UART_Transmit_DMA(&huart3, tr_vec_u8.data, tr_vec_u8.length);
}
