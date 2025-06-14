#include "uart/packet_proc.h"
#include "main/mcu_const.h"
#include "motor/main.h"
#include "uart/main.h"

float f32_test = 1;
uint16_t u16_test = 1;

/**
 * @brief 將右側馬達當前速度回應至資料向量
 *        Push current right motor speed response into byte vector
 *
 * @param vec_u8 指向要寫入資料的 VecU8 (input/output vector to receive response data)
 * @return void
 */
void rspdw(VecU8* vec_u8) {
    vec_u8_push(vec_u8, CMD_RIGHT_SPEED_STORE, sizeof(CMD_RIGHT_SPEED_STORE));
    // vec_u8_push_f32(vec_u8, motor_right.speed_present);
    vec_u8_push_f32(vec_u8, f32_test);
    f32_test++;
}

/**
 * @brief 將右側馬達 ADC 值回應至資料向量
 *        Push right motor ADC value response into byte vector
 *
 * @param vec_u8 指向要寫入資料的 VecU8 (input/output vector to receive ADC data)
 * @return void
 */
void radcw(VecU8* vec_u8) {
    vec_u8_push(vec_u8, CMD_RIGHT_ADC_STORE, sizeof(CMD_RIGHT_ADC_STORE));
    // vec_u8_push_u16(vec_u8, motor_right.adc_value);
    vec_u8_push_u16(vec_u8, u16_test);
    u16_test++;
}

/**
 * @brief 組合並傳輸封包至傳輸緩衝區
 *        Assemble and transmit packet into transfer buffer
 *
 * @note 根據 transceive_flags 決定回應內容
 *
 * @return void
 */
void uart_transmit_pkt_proc(void) {
    VecU8 vec_u8 = vec_u8_new();
    vec_u8_push_byte(&vec_u8, CMD_CODE_DATA_TRRE);
    bool new_vec_wri_flag = false;
    if (transceive_flags.right_speed) {
        new_vec_wri_flag = true;
        rspdw(&vec_u8);
    }
    if (transceive_flags.right_adc) {
        new_vec_wri_flag = true;
        radcw(&vec_u8);
    }
    rspdw(&vec_u8);
    radcw(&vec_u8);
    new_vec_wri_flag = true;
    if (new_vec_wri_flag) {
        UartPacket packet = uart_packet_new();
        uart_pkt_add_data(&packet, &vec_u8);
        uart_trcv_buf_push(&uart_trsm_pkt_buf, &packet);
    };
}

/**
 * @brief 處理接收命令並存儲/回應資料
 *        Process received commands and store or respond data
 *
 * @param vec_u8 指向去除命令碼後的資料向量 (input vector without command code)
 * @return void
 */
static void uart_re_pkt_proc_data_store(VecU8 *vec_u8) {
    VecU8 new_vec = vec_u8_new();
    vec_u8_push_byte(&new_vec, CMD_CODE_DATA_TRRE);
    bool data_proc_flag;
    bool new_vec_wri_flag = false;
    do {
        data_proc_flag = false;
        if (vec_u8_starts_with(vec_u8, CMD_RIGHT_SPEED_STOP, sizeof(CMD_RIGHT_SPEED_STOP))) {
            vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_SPEED_STOP));
            data_proc_flag = true;
            transceive_flags.right_speed = false;
        }
        else if (vec_u8_starts_with(vec_u8, CMD_RIGHT_SPEED_ONCE, sizeof(CMD_RIGHT_SPEED_ONCE))) {
            vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_SPEED_ONCE));
            data_proc_flag = true;
            new_vec_wri_flag = true;
            rspdw(&new_vec);
        }
        else if (vec_u8_starts_with(vec_u8, CMD_RIGHT_SPEED_START, sizeof(CMD_RIGHT_SPEED_START))) {
            vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_SPEED_START));
            data_proc_flag = true;
            transceive_flags.right_speed = true;
        }
        else if (vec_u8_starts_with(vec_u8, CMD_RIGHT_ADC_STOP, sizeof(CMD_RIGHT_ADC_STOP))) {
            vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_ADC_STOP));
            data_proc_flag = true;
            transceive_flags.right_adc = false;
        }
        else if (vec_u8_starts_with(vec_u8, CMD_RIGHT_ADC_ONCE, sizeof(CMD_RIGHT_ADC_ONCE))) {
            vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_ADC_ONCE));
            data_proc_flag = true;
            new_vec_wri_flag = true;
            radcw(&new_vec);
        }
        else if (vec_u8_starts_with(vec_u8, CMD_RIGHT_ADC_START, sizeof(CMD_RIGHT_ADC_START))) {
            vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_ADC_START));
            data_proc_flag = true;
            transceive_flags.right_adc = true;
        }
    } while (data_proc_flag);
    if (new_vec_wri_flag) {
        UartPacket new_packet = uart_packet_new();
        uart_pkt_add_data(&new_packet, &new_vec);
        uart_trcv_buf_push(&uart_trsm_pkt_buf, &new_packet);
    }
}

/**
 * @brief 從接收緩衝區反覆讀取封包並處理
 *        Pop packets from receive buffer and process them
 *
 * @param count 單次最大處理封包數量 (input maximum number of packets to process per time)
 * @return void
 */
void uart_receive_pkt_proc(uint8_t count) {
    uint8_t i;
    for (i = 0; i < count; i++){
        UartPacket packet = uart_packet_new();
        if (!uart_trcv_buf_pop_front(&uart_recv_pkt_buf, &packet)) {
            return;
        }
        VecU8 vec_u8 = vec_u8_new();
        uart_pkt_get_data(&packet, &vec_u8);
        uint8_t code = vec_u8.data[vec_u8.head];
        vec_u8_rm_range(&vec_u8, 0, 1);
        switch (code) {
            case CMD_CODE_DATA_TRRE:
                uart_re_pkt_proc_data_store(&vec_u8);
                break;
            default:
                break;
        }
    }
}
