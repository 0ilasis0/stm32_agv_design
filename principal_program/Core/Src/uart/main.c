#include "uart/main.h"
#include <string.h>
#include "usart.h"
#include "uart/packet_proc.h"
#include "uart/trcv_buffer.h"

/**
 * @brief 傳輸/接收操作旗標
 *        Transmit/receive operation flags
 *
 * @details 控制資料處理流程 (Control data processing flow)
 */
TransceiveFlags transceive_flags = {0};

VecU8 uart_dma_tr_bytes = {0};
/**
 * @brief UART 接收 DMA 緩衝區
 *
 *        UART receive DMA buffer
 */
VecU8 uart_dma_rv_bytes = {0};

/**
 * @brief 發送下一筆 UART 封包至 DMA
 *        Transmit next UART packet via DMA
 */
static void uart_transmit(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        if (HAL_DMA_GetState(huart->hdmatx) == HAL_DMA_STATE_BUSY) return;
        UartPacket packet = uart_packet_new();
        if (!uart_trcv_buf_pop(&uart_trsm_pkt_buf, &packet)) return;
        uart_pkt_unpack(&packet, &uart_dma_tr_bytes);
        HAL_UART_Transmit_DMA(huart, uart_dma_tr_bytes.data, uart_dma_tr_bytes.len);
    }
}

/**
 * @brief UART 傳輸完成回調：移除已傳輸封包
 *        UART TX complete callback: pop transmitted packet
 *
 * @param huart 指向 UART 處理器結構體的指標 (input UART handle pointer)
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        
    }
}

/**
 * @brief UART 空閒接收事件回調：處理接收資料並推入接收緩衝
 *        UART IDLE reception event callback: process received data and push to receive buffer
 *
 * @param huart 指向 UART 處理器結構體的指標 (input UART handle pointer)
 * @param Size 接收到的資料長度 (input number of received bytes)
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART3) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        if (Size == 0) {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_dma_rv_bytes.data, VECU8_MAX_CAPACITY);
            return;
        }
        uart_dma_rv_bytes.len = Size;
        UartPacket packet = uart_packet_new();
        if (uart_pkt_pack(&packet, &uart_dma_rv_bytes)) {
            uart_trcv_buf_push(&uart_recv_pkt_buf, &packet);
            vec_u8_rm_range(&uart_dma_rv_bytes, 0, VECU8_MAX_CAPACITY);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_dma_rv_bytes.data, VECU8_MAX_CAPACITY);
    }
}

/**
 * @brief 設置 UART，清零接收緩衝並啟用 DMA 接收於 IDLE 中斷
 *        Configure UART: clear receive buffer and enable DMA reception on IDLE interrupt
 */
void uart_setup(void) {
    // Tx:PB9(R5) Rx:PB11(R18)
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_dma_rv_bytes.data, VECU8_MAX_CAPACITY);
}

void uart_main(void) {
    if (transceive_flags.uart_transmit) {
        transceive_flags.uart_transmit = false;
        uart_transmit(&huart3);
    }
    if (transceive_flags.uart_transmit_pkt_proc) {
        transceive_flags.uart_transmit_pkt_proc = false;
        uart_transmit_pkt_proc();
    }
    if (transceive_flags.uart_receive_pkt_proc) {
        transceive_flags.uart_receive_pkt_proc = false;
        uart_receive_pkt_proc(5);
    }
}
