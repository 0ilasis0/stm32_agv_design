#include "uart/main.h"
#include <string.h>
#include "usart.h"
#include "uart/packet_proc.h"

/**
 * @brief UART 初始化完成旗標
 *        UART initialization flag
 *
 * @details 用於跳過第一次 DMA IDLE 中斷 (Used to skip the first DMA IDLE interrupt)
 */
static bool uart_init;

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

void USER_MX_USART3_UART_Init(void) {
    uart_init = 0;
    uart_dma_tr_bytes = vec_u8_new();
    uart_dma_rv_bytes = vec_u8_new();
    uart_trcv_buf_init();
}

static void uart_reset_hdmarx_CNDTR(UART_HandleTypeDef *huart) {
    __HAL_DMA_DISABLE(huart->hdmarx);
    huart->hdmarx->Instance->CNDTR = VECU8_MAX_CAPACITY;
    __HAL_DMA_ENABLE(huart->hdmarx);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

/**
 * @brief UART3 中斷前置處理：檢查並清除 IDLE 標誌，重新配置 DMA
 *        Pre-handler for UART3 interrupt: check and clear IDLE flag, reconfigure DMA
 */
void USER_UART3_IRQHandler_Before(void) {
    UART_HandleTypeDef *huart = &huart3;
    if (!__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) return;
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    uint16_t size = VECU8_MAX_CAPACITY - huart->hdmarx->Instance->CNDTR;
    uart_dma_rv_bytes.len = size;
    uart_reset_hdmarx_CNDTR(huart);
    HAL_UARTEx_RxEventCallback(huart, size);
}

/**
 * @brief 發送下一筆 UART 封包至 DMA
 *        Transmit next UART packet via DMA
 */
static void uart_transmit(UART_HandleTypeDef *huart) {
    if (HAL_DMA_GetState(huart->hdmatx) == HAL_DMA_STATE_BUSY) {
        return;
    }
    UartPacket packet = uart_packet_new();
    if (!uart_trcv_buf_pop_front(&uart_trsm_pkt_buf, &packet)) {
        return;
    }
    uart_pkt_unpack(&packet, &uart_dma_tr_bytes);
    HAL_UART_Transmit_DMA(&huart3, uart_dma_tr_bytes.data, uart_dma_tr_bytes.len);
}

/**
 * @brief UART 傳輸完成回調：移除已傳輸封包
 *        UART TX complete callback: pop transmitted packet
 *
 * @param huart 指向 UART 處理器結構體的指標 (input UART handle pointer)
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        uart_trcv_buf_void_front(&uart_trsm_pkt_buf);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}

static void uart_receive(UART_HandleTypeDef *huart) {
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    UartPacket packet = uart_packet_new();
    if (uart_pkt_pack(&packet, &uart_dma_rv_bytes)) {
        uart_trcv_buf_push(&uart_recv_pkt_buf, &packet);
        vec_u8_rm_range(&uart_dma_rv_bytes, 0, VECU8_MAX_CAPACITY);
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
        if (!uart_init) {
            uart_init = true;
            return;
        }
        uart_receive(&huart3);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_dma_rv_bytes.data, VECU8_MAX_CAPACITY);
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
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
