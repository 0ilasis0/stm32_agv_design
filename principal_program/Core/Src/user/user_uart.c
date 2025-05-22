#include "user/user_uart.h"
#include <string.h>
#include "user/user_packet.h"
#include "usart.h"
#include "user/motor.h"

/**
  * UART 初始化標誌
  * 
  * UART initialization flag
  */
bool uart_init = 0;

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
UartPacket packet_test = {0};
float f32_test = 2;
uint16_t u16_test = 2;
void uart_setup(void) {
    memset(uart_receive_buffer, 0, sizeof(uart_receive_buffer));
    // transfer_buffer = trRe_buffer_new();
    // receive_buffer = trRe_buffer_new();
    // Rx:PB11(R18) Tx:PB9(R5)
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_receive_buffer, PACKET_MAX_SIZE);

    VecU8 new_vec = vec_u8_new();
    vec_u8_push(&new_vec, &(uint8_t){0x10}, 1);
    vec_u8_push(&new_vec, &(uint8_t){0x01}, 1);
    vec_u8_push(&new_vec, &(uint8_t){0x05}, 1);
    vec_u8_push_u16(&new_vec, u16_test);
    u16_test++;
    vec_u8_push(&new_vec, &(uint8_t){0x01}, 1);
    vec_u8_push(&new_vec, &(uint8_t){0x00}, 1);
    vec_u8_push_float(&new_vec, f32_test);
    f32_test++;
    packet_test = uart_packet_new(&new_vec);
}

/**
  * UART3 中斷前置處理函式，檢查並清除 IDLE 標誌
  * 
  * Pre-handler for UART3 interrupt: check and clear the IDLE flag
  */
void HYCodes_UART3_IRQHandler_Before(void) {
    if (!__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE)) return;
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);

    uint16_t size = PACKET_MAX_SIZE - huart3.hdmarx->Instance->CNDTR;
    uart_RxCNDTR_reset(&huart3);

    HAL_UARTEx_RxEventCallback(&huart3, size);
}

/**
  * 重置 DMA 的 CNDTR 寄存器並重新啟用空閒中斷
  * 
  * Reset DMA CNDTR register and re-enable IDLE interrupt
  */
void uart_RxCNDTR_reset(UART_HandleTypeDef *huart) {
    __HAL_DMA_DISABLE(huart->hdmarx);
    huart->hdmarx->Instance->CNDTR = PACKET_MAX_SIZE;
    __HAL_DMA_ENABLE(huart->hdmarx);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

/**
  * UART 傳輸完成回調函式，可擴展使用者處理
  * 
  * UART TX complete callback; can be extended for user handling
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        // 可在此處添加傳輸完成處理
        // Transmission complete handling can be added here
        trRe_buffer_pop_secondHalf(&transfer_buffer);
    }
}

void rspdw(UartPacket* packet) {
    VecU8 new_vec = vec_u8_new();
    vec_u8_push(&new_vec, &(uint8_t){0x01}, 1);
    vec_u8_push(&new_vec, &(uint8_t){0x00}, 1);
    vec_u8_push_float(&new_vec, f32_test);
    f32_test++;
    uart_packet_add_data(packet, &new_vec);
}

void radcw(UartPacket* packet) {
    VecU8 new_vec = vec_u8_new();
    vec_u8_push(&new_vec, &(uint8_t){0x01}, 1);
    vec_u8_push(&new_vec, &(uint8_t){0x05}, 1);
    vec_u8_push_u16(&new_vec, u16_test);
    u16_test++;
    uart_packet_add_data(packet, &new_vec);
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
        VecU8 re_vec_u8 = vec_u8_new();
        vec_u8_push(&re_vec_u8, uart_receive_buffer, Size);
        memset(uart_receive_buffer, 0, PACKET_MAX_SIZE);
        UartPacket re_packet = uart_packet_pack(&re_vec_u8);
        trRe_buffer_push(&receive_buffer, &re_packet);

        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_receive_buffer, PACKET_MAX_SIZE);
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    }
}

void hytest(void) {
    VecU8 new_vec = vec_u8_new();
    vec_u8_push(&new_vec, &(uint8_t){0x10}, 1);
    UartPacket new_packet = uart_packet_new(&new_vec);
    rspdw(&new_packet);
    radcw(&new_packet);
    trRe_buffer_push(&transfer_buffer, &new_packet);

    if (transfer_buffer.count != 0) {
        UartPacket tr_packet = trRe_buffer_pop_firstHalf(&transfer_buffer);
        VecU8 tr_vec_u8 = uart_packet_unpack(&tr_packet);
        HAL_UART_Transmit_DMA(&huart3, tr_vec_u8.data, tr_vec_u8.length);
    }
}
