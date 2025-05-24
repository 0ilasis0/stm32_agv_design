#include "user/packet_proc_mod.h"
#include "user/user_uart.h"
#include "user/packet_mod.h"
#include "user/motor.h"
#include "user/mcu_const.h"

float f32_test = 1;
uint16_t u16_test = 1;

void rspdw(VecU8* vec_u8) {
    vec_u8_push(vec_u8, &(uint8_t){0x01}, 1);
    vec_u8_push(vec_u8, &(uint8_t){0x00}, 1);
    vec_u8_push_float(vec_u8, motor_right.present_speed);
    f32_test++;
}
void radcw(VecU8* vec_u8) {
    vec_u8_push(vec_u8, &(uint8_t){0x01}, 1);
    vec_u8_push(vec_u8, &(uint8_t){0x05}, 1);
    vec_u8_push_u16(vec_u8, motor_right.adc_value);
    u16_test++;
}

void uart_tr_packet_proccess(void) {
    tr_re_flags.need_tr_proc = false;
    VecU8 new_vec = {0};
    vec_u8_push(&new_vec, &(uint8_t){0x10}, 1);
    bool new_vec_wri_flag = false;
    if (tr_re_flags.right_speed) {
        new_vec_wri_flag = true;
        rspdw(&new_vec);
    }
    if (tr_re_flags.right_adc) {
        new_vec_wri_flag = true;
        radcw(&new_vec);
    }
    if (new_vec_wri_flag) {
        UartPacket new_packet = uart_packet_new(&new_vec);
        trRe_buffer_push(&transfer_buffer, &new_packet);
    };
}

void uart_re_pkt_proc_data_store(VecU8 *vec_u8);
void uart_re_packet_proccess(uint8_t count) {
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
                uart_re_pkt_proc_data_store(&re_vec_u8);
                break;
            default:
                break;
        }
    }
}

void uart_re_pkt_proc_data_store(VecU8 *vec_u8) {
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
        uart_packet_add_data(&new_packet, vec_u8);
        trRe_buffer_push(&transfer_buffer, &new_packet);
    }
}
