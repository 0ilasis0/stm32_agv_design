#include "user/user_uart.h"
#include <string.h>
#include "user/user_packet.h"
#include "usart.h"
#include "user/motor.h"
#include "user/mcu_cmd.h"

/**
  * UART 初始化標誌
  * 
  * UART initialization flag
  */
bool uart_init = 0;

TrReFlags tr_re_flags = {0};

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
float f32_test = 1;
uint16_t u16_test = 1;
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
        trRe_buffer_pop_secondHalf(&transfer_buffer);
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
        trRe_buffer_push(&receive_buffer, &re_packet);

        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_receive_buffer, PACKET_MAX_SIZE);
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    }
}

void uart_packet_send(void) {
    if (transfer_buffer.length == 0) return;
    UartPacket tr_packet = trRe_buffer_pop_firstHalf(&transfer_buffer);
    VecU8 tr_vec_u8 = uart_packet_unpack(&tr_packet);
    HAL_UART_Transmit_DMA(&huart3, tr_vec_u8.data, tr_vec_u8.length);
}

void rspdw(VecU8* vec_u8) {
    vec_u8_push(&vec_u8, &(uint8_t){0x01}, 1);
    vec_u8_push(&vec_u8, &(uint8_t){0x00}, 1);
    vec_u8_push_float(&vec_u8, motor_right.present_speed);
    f32_test++;
}
void radcw(VecU8* vec_u8) {
    vec_u8_push(&vec_u8, &(uint8_t){0x01}, 1);
    vec_u8_push(&vec_u8, &(uint8_t){0x05}, 1);
    vec_u8_push_u16(&vec_u8, motor_right.adc_value);
    u16_test++;
}

void tr_packet_proccess(void) {
    tr_re_flags.need_tr_proc = false;
    VecU8 new_vec = {0};
    vec_u8_push(&new_vec, &(uint8_t){0x10}, 1);
    UartPacket new_packet = uart_packet_new(&new_vec);
    bool new_vec_wri_flag = false;
    if (tr_re_flags.right_speed) {
        new_vec_wri_flag = true;
        rspdw(&new_packet);
    }
    if (tr_re_flags.right_adc) {
        new_vec_wri_flag = true;
        radcw(&new_packet);
    }
    if (new_vec_wri_flag) {
        trRe_buffer_push(&transfer_buffer, &new_packet);
    };
}

void re_pkt_proc_data_store(VecU8 *vec_u8);
void re_packet_proccess(uint8_t count) {
    tr_re_flags.need_re_proc = false;
    uint8_t i;
    for (i = 0; i < 5; i++){
        if (receive_buffer.length == 0) return;
        UartPacket re_packet = trRe_buffer_pop_firstHalf(&receive_buffer);
        trRe_buffer_pop_secondHalf(&receive_buffer);
        VecU8 re_vec_u8 = uart_packet_get_vec(&re_packet);
        uint8_t code = re_vec_u8.data[0];
        vec_u8_rm_front(&re_vec_u8, 1);
        switch (code) {
            case CMD_CODE_DATA_TRRE:
                re_pkt_proc_data_store(&re_vec_u8);
                break;
            default:
                break;
        }
    }
}

void re_pkt_proc_data_store(VecU8 *vec_u8) {
    VecU8 new_vec = {0};
    vec_u8_push(&new_vec, &(uint8_t){0x10}, 1);
    bool data_proc_flag;
    bool new_vec_wri_flag = false;
    while (1) {
        data_proc_flag = false;
        if (vec_u8_starts_with_s(vec_u8, CMD_RIGHT_SPEED_STOP)) {
            vec_u8_rm_front(vec_u8, sizeof(CMD_RIGHT_SPEED_STOP));
            data_proc_flag = true;
            tr_re_flags.right_speed = false;
        }
        if (vec_u8_starts_with_s(vec_u8, CMD_RIGHT_SPEED_ONCE)) {
            vec_u8_rm_front(vec_u8, sizeof(CMD_RIGHT_SPEED_ONCE));
            data_proc_flag = true;
            new_vec_wri_flag = true;
            rspdw(&new_vec);
        }
        if (vec_u8_starts_with_s(vec_u8, CMD_RIGHT_SPEED_START)) {
            vec_u8_rm_front(vec_u8, sizeof(CMD_RIGHT_SPEED_START));
            data_proc_flag = true;
            tr_re_flags.right_speed = true;
        }
        if (vec_u8_starts_with_s(vec_u8, CMD_RIGHT_ADC_STOP)) {
            vec_u8_rm_front(vec_u8, sizeof(CMD_RIGHT_ADC_STOP));
            data_proc_flag = true;
            tr_re_flags.right_adc = false;
        }
        if (vec_u8_starts_with_s(vec_u8, CMD_RIGHT_ADC_ONCE)) {
            vec_u8_rm_front(vec_u8, sizeof(CMD_RIGHT_ADC_ONCE));
            data_proc_flag = true;
            new_vec_wri_flag = true;
            radcw(&new_vec);
        }
        if (vec_u8_starts_with_s(vec_u8, CMD_RIGHT_ADC_START)) {
            vec_u8_rm_front(vec_u8, sizeof(CMD_RIGHT_ADC_START));
            data_proc_flag = true;
            tr_re_flags.right_adc = true;
        }
        if (!data_proc_flag) break;
    }
    if (new_vec_wri_flag) {
        UartPacket new_packet = uart_packet_new(&new_vec);
        uart_packet_add_data(&new_packet, &vec_u8);
        trRe_buffer_push(&transfer_buffer, &new_packet);
    }
}
