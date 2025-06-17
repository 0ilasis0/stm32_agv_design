#include "uart/packet_proc.h"
#include "main/global_state.h"
#include "main/mcu_const.h"
#include "motor/main.h"
#include "uart/main.h"
#include "uart/trcv_buffer.h"

float f32_test = 0;
uint16_t u16_test = 0;

/**
 * @brief 將右側馬達當前速度回應至資料向量
 *        Push current right motor speed response into byte vector
 *
 * @param vec_u8 指向要寫入資料的 VecU8 (input/output vector to receive response data)
 * @return void
 */
static void rspdw(void) {
    f32_test++;
    VecU8 vec_u8 = VEC_U8_NEW();
    vec_u8_push_byte(&vec_u8, CMD_CODE_DATA_TRRE);
    vec_u8_push(&vec_u8, CMD_RIGHT_SPEED_STORE, sizeof(CMD_RIGHT_SPEED_STORE));
    // vec_u8_push_f32(vec_u8, motor_right.speed_present);
    vec_u8_push_f32(&vec_u8, f32_test);
    UartPacket packet = UART_PKT_NEW();
    uart_pkt_add_data(&packet, &vec_u8);
    uart_trcv_buf_push(global_state.uart_tr_pkt_buf_h, &packet);
}

/**
 * @brief 將右側馬達 ADC 值回應至資料向量
 *        Push right motor ADC value response into byte vector
 *
 * @param vec_u8 指向要寫入資料的 VecU8 (input/output vector to receive ADC data)
 * @return void
 */
static void radcw(void) {
    u16_test++;
    VecU8 vec_u8 = VEC_U8_NEW();
    vec_u8_push_byte(&vec_u8, CMD_CODE_DATA_TRRE);
    vec_u8_push(&vec_u8, CMD_RIGHT_ADC_STORE, sizeof(CMD_RIGHT_ADC_STORE));
    // vec_u8_push_u16(vec_u8, motor_right.adc_value);
    vec_u8_push_u16(&vec_u8, u16_test);
    UartPacket packet = UART_PKT_NEW();
    uart_pkt_add_data(&packet, &vec_u8);
    uart_trcv_buf_push(global_state.uart_tr_pkt_buf_h, &packet);
}

/**
 * @brief 組合並傳輸封包至傳輸緩衝區
 *        Assemble and transmit packet into transfer buffer
 *
 * @note 根據 transceive_flags 決定回應內容
 *
 * @return void
 */
void uart_tr_pkt_proc(void) {
    // if (transceive_flags.right_speed) {
    //     rspdw();
    // }
    // if (transceive_flags.right_adc) {
    //     radcw();
    // }
    rspdw();
    radcw();
}

/**
 * @brief 處理接收命令並存儲/回應資料
 *        Process received commands and store or respond data
 *
 * @param vec_u8 指向去除命令碼後的資料向量 (input vector without command code)
 * @return void
 */
static void uart_re_pkt_proc_data_store(VecU8 *vec_u8) {
    bool data_proc_flag;
    do {
        VecU8 new_vec = VEC_U8_NEW();
        vec_u8_push_byte(&new_vec, CMD_CODE_DATA_TRRE);
        data_proc_flag = false;
        if (vec_u8_starts_with(vec_u8, CMD_RIGHT_SPEED_STOP, sizeof(CMD_RIGHT_SPEED_STOP))) {
            vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_SPEED_STOP));
            data_proc_flag = true;
            global_state.transceive_flags_h->right_speed = false;
        }
        else if (vec_u8_starts_with(vec_u8, CMD_RIGHT_SPEED_ONCE, sizeof(CMD_RIGHT_SPEED_ONCE))) {
            vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_SPEED_ONCE));
            data_proc_flag = true;
            rspdw();
        }
        else if (vec_u8_starts_with(vec_u8, CMD_RIGHT_SPEED_START, sizeof(CMD_RIGHT_SPEED_START))) {
            vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_SPEED_START));
            data_proc_flag = true;
            global_state.transceive_flags_h->right_speed = true;
        }
        else if (vec_u8_starts_with(vec_u8, CMD_RIGHT_ADC_STOP, sizeof(CMD_RIGHT_ADC_STOP))) {
            vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_ADC_STOP));
            data_proc_flag = true;
            global_state.transceive_flags_h->right_adc = false;
        }
        else if (vec_u8_starts_with(vec_u8, CMD_RIGHT_ADC_ONCE, sizeof(CMD_RIGHT_ADC_ONCE))) {
            vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_ADC_ONCE));
            data_proc_flag = true;
            radcw();
        }
        else if (vec_u8_starts_with(vec_u8, CMD_RIGHT_ADC_START, sizeof(CMD_RIGHT_ADC_START))) {
            vec_u8_rm_range(vec_u8, 0, sizeof(CMD_RIGHT_ADC_START));
            data_proc_flag = true;
            global_state.transceive_flags_h->right_adc = true;
        }
    } while (data_proc_flag);
}

/**
 * @brief 從接收緩衝區反覆讀取封包並處理
 *        Pop packets from receive buffer and process them
 *
 * @param count 單次最大處理封包數量 (input maximum number of packets to process per time)
 * @return void
 */
void uart_re_pkt_proc(uint8_t count) {
    uint8_t i;
    for (i = 0; i < count; i++){
        UartPacket packet = UART_PKT_NEW();
        if (!uart_trcv_buf_pop(global_state.uart_rv_pkt_buf_h, &packet)) return;
        VecU8 vec_u8 = VEC_U8_NEW();
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
