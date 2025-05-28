#include "user/packet_proc_mod.h"
#include "user/user_uart.h"
#include "user/motor.h"
#include "user/mcu_const.h"

/**
  * 傳輸和接收循環緩衝區
  * 
  * Transmission and reception ring buffers
  */
TransceiveBuffer transfer_buffer = {0};
TransceiveBuffer receive_buffer = {0};

/**
  * 建立傳輸/接收環形緩衝區，初始化頭指標與計數
  * 
  * Create a transmit/receive ring buffer, initialize head and count
  */
TransceiveBuffer transceive_buffer_new(void) {
    TransceiveBuffer transceive_buffer;
    transceive_buffer.head = 0;
    transceive_buffer.length = 0;
    return transceive_buffer;
}

/**
  * 將封包推入環形緩衝區，若已滿則返回false
  * 
  * Push a packet into the ring buffer; return false if buffer is full
  */
bool transceive_buffer_push(TransceiveBuffer *transceive_buffer, const UartPacket *packet) {
    if (transceive_buffer->length >= TR_RE_PKT_BUFFER_CAP) return false;
    uint8_t tail = (transceive_buffer->head + transceive_buffer->length) % TR_RE_PKT_BUFFER_CAP;
    transceive_buffer->packet[tail] = *packet;
    transceive_buffer->length++;
    return true;
}

/**
  * 從環形緩衝區彈出一個封包資料
  * 
  * Pop a packet from the ring buffer
  */
bool transceive_buffer_pop(TransceiveBuffer *buffer, UartPacket *packet) {
    transceive_buffer_pop_firstHalf(buffer, packet);
    return transceive_buffer_pop_secondHalf(buffer);
}

bool transceive_buffer_pop_firstHalf(const TransceiveBuffer *buffer, UartPacket *packet) {
    if (buffer->length == 0) return 0;
    *packet = buffer->packet[buffer->head];
    return 1;
}

bool transceive_buffer_pop_secondHalf(TransceiveBuffer *buffer) {
    if (buffer->length == 0) return 0;
    if (--buffer->length == 0) {
        buffer->head = 0;
    } else {
        buffer->head = (buffer->head + 1) % TR_RE_PKT_BUFFER_CAP;
    }
    return 1;
}

float f32_test = 1;
uint16_t u16_test = 1;
void rspdw(VecU8* vec_u8) {
    vec_u8_push(vec_u8, &(uint8_t){0x01}, 1);
    vec_u8_push(vec_u8, &(uint8_t){0x00}, 1);
    vec_u8_push_float(vec_u8, motor_right.speed_present);
    // vec_u8_push_float(vec_u8, f32_test);
    // f32_test++;
}
void radcw(VecU8* vec_u8) {
    vec_u8_push(vec_u8, &(uint8_t){0x01}, 1);
    vec_u8_push(vec_u8, &(uint8_t){0x05}, 1);
    vec_u8_push_u16(vec_u8, motor_right.adc_value);
    // vec_u8_push_u16(vec_u8, u16_test);
    // u16_test++;
}

void uart_transmit_pkt_proc(bool *flag) {
    if (flag == NULL || !*flag) return;
    *flag = false;
    VecU8 new_vec = {0};
    vec_u8_push(&new_vec, &(uint8_t){0x10}, 1);
    bool new_vec_wri_flag = false;
    if (transceive_flags.right_speed) {
        new_vec_wri_flag = true;
        rspdw(&new_vec);
    }
    if (transceive_flags.right_adc) {
        new_vec_wri_flag = true;
        radcw(&new_vec);
    }
    if (new_vec_wri_flag) {
        UartPacket new_packet = uart_packet_new(&new_vec);
        transceive_buffer_push(&transfer_buffer, &new_packet);
    };
}

void uart_re_pkt_proc_data_store(VecU8 *vec_u8);

void uart_receive_pkt_proc(bool *flag, uint8_t count) {
    if (flag == NULL || !*flag) return;
    *flag = false;
    uint8_t i;
    for (i = 0; i < 5; i++){
        UartPacket packet;
        if (!transceive_buffer_pop(&receive_buffer, &packet)) {
            break;
        }
        VecU8 re_vec_u8 = uart_packet_get_vec(&packet);
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
            transceive_flags.right_speed = false;
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
            transceive_flags.right_speed = true;
        }
        if (vec_u8_starts_with_s(vec_u8, CMD_RIGHT_ADC_STOP)) {
            vec_u8_rm_front(vec_u8, sizeof(CMD_RIGHT_ADC_STOP));
            data_proc_flag = true;
            transceive_flags.right_adc = false;
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
            transceive_flags.right_adc = true;
        }
        if (!data_proc_flag) break;
    }
    if (new_vec_wri_flag) {
        UartPacket new_packet = uart_packet_new(&new_vec);
        uart_packet_add_data(&new_packet, vec_u8);
        transceive_buffer_push(&transfer_buffer, &new_packet);
    }
}

void transmit_buf_set(VecU8* vec_u8) {
    vec_u8_push_u16(vec_u8, motor_left.adc_value);
    vec_u8_push_u8(vec_u8, motor_left.speed_sepoint);
    vec_u8_push_float(vec_u8, motor_left.speed_present);
    vec_u8_push_u8(vec_u8, motor_left.rotate_direction);
    vec_u8_push_u16(vec_u8, motor_right.adc_value);
    vec_u8_push_u8(vec_u8, motor_right.speed_sepoint);
    vec_u8_push_float(vec_u8, motor_right.speed_present);
    vec_u8_push_u8(vec_u8, motor_right.rotate_direction);
}
