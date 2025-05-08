#include "principal/my_uart.h"

bool uart_init = 0;
uint8_t uart_buffer_r[PACKET_MAX_SIZE];

TrReBuffer transfer_buffer;
TrReBuffer receive_buffer;

// UART buffer init
void uartInit(void) {
    memset(uart_buffer_r, 0, sizeof(uart_buffer_r));
    transfer_buffer = tr_re_buffer_new();
    receive_buffer = tr_re_buffer_new();
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_buffer_r, PACKET_MAX_SIZE);
}

// 可更換fn名稱
void HYCodes_UART3_IRQHandler_Before(void) {
    if (!__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE)) return;
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);

    uint16_t size = PACKET_MAX_SIZE - huart3.hdmarx->Instance->CNDTR;
    uart_RxCNDTR_reset(&huart3);

    HAL_UARTEx_RxEventCallback(&huart3, size);
}

// 可更換fn名稱
void uart_RxCNDTR_reset(UART_HandleTypeDef *huart) {
    __HAL_DMA_DISABLE(huart->hdmarx);
    huart->hdmarx->Instance->CNDTR = PACKET_MAX_SIZE;
    __HAL_DMA_ENABLE(huart->hdmarx);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

// 不可更換fn名稱
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {

    }
}

// 不可更換fn名稱
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART3) {
        if (!uart_init) {
            uart_init = true;
            return;
        }
        VecU8 re_vec_u8 = vec_u8_new();
        vec_u8_extend_inner(&re_vec_u8, uart_buffer_r, Size);
        memset(uart_buffer_r, 0, PACKET_MAX_SIZE);
        UartPacket re_packet = uart_packet_pack(&re_vec_u8);
        tr_re_buffer_push(&receive_buffer, &re_packet);

        VecU8 new_vec = float_to_u8(12);// motor_right.present_speed
        UartPacket new_packet = uart_packet_new(&new_vec);
        tr_re_buffer_push(&transfer_buffer, &new_packet);

        UartPacket tr_packet = tr_re_buffer_pop(&transfer_buffer);
        VecU8 tr_vec_u8 = uart_packet_unpack(&tr_packet);
        HAL_UART_Transmit_DMA(huart, tr_vec_u8.data, tr_vec_u8.length);

        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_buffer_r, PACKET_MAX_SIZE);
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    }
}
