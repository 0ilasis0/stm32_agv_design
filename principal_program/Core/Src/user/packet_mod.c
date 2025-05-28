#include "user/packet_mod.h"

/**
  * 生成一個新的UART封包，包含起始碼與結束碼
  * 
  * Create a new UART packet with start and end codes
  */
UartPacket uart_packet_new(VecU8 *data) {
    UartPacket packet;
    packet.start = PACKET_START_CODE;
    packet.data_vec_u8 = (VecU8){0};
    vec_u8_push(&packet.data_vec_u8, data->data, data->length);
    packet.end = PACKET_END_CODE;
    return packet;
}

/**
  * 生成一個錯誤封包，用於表示封包解析失敗
  * 
  * Create an error packet to indicate packet parsing failure
  */
UartPacket uart_packet_new_error(void) {
    UartPacket packet;
    packet.start = 0;
    return packet;
}

/**
  * 檢查封包是否為錯誤封包
  * 
  * Check if the packet is an error packet
  */
bool packet_error(const UartPacket *packet) {
    if (packet->start == 0) {
        return true;
    }
    return false;
}

VecU8 uart_packet_get_vec(const UartPacket *packet) {
    VecU8 vec_u8 = {0};
    vec_u8_push(&vec_u8, packet->data_vec_u8.data, packet->data_vec_u8.length);
    return vec_u8;
}

/**
  * 根據原始資料向量打包成UART封包，並移除起始與結束碼後重新封裝
  * 
  * Pack raw data vector into UART packet, stripping start and end codes before repacking
  */
UartPacket uart_packet_pack(const VecU8 *vec_u8) {
    if (
        (vec_u8->length < 2 || vec_u8->data[0] != PACKET_START_CODE) ||
        (vec_u8->data[vec_u8->length - 1] != PACKET_END_CODE)
    ) {
        return uart_packet_new_error();
    }
    VecU8 data_vec = {0};
    vec_u8_push(&data_vec, vec_u8->data + 1, vec_u8->length - 2);
    return uart_packet_new(&data_vec);
}

void uart_packet_add_data(UartPacket *packet, const VecU8 *vec_u8) {
    vec_u8_push(&packet->data_vec_u8, vec_u8->data, vec_u8->length);
}

/**
  * 解包UART封包，將封包前後碼與資料合併為一個字節向量
  * 
  * Unpack UART packet into a byte vector including start, data, and end codes
  */
VecU8 uart_packet_unpack(const UartPacket *packet) {
    VecU8 vec_u8 = {0};
    vec_u8_push(&vec_u8, &packet->start, 1);
    vec_u8_push(&vec_u8, packet->data_vec_u8.data, packet->data_vec_u8.length);
    vec_u8_push(&vec_u8, &packet->end, 1);
    return vec_u8;
}
