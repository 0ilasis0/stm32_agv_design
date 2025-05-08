#include "principal/packet.h"

UartPacket uart_packet_new(VecU8 *data) {
    UartPacket packet;
    packet.start = PACKET_START_CODE;
    packet.data = vec_u8_new();
    vec_u8_extend_inner(&packet.data, data->data, data->length);
    packet.end = PACKET_END_CODE;
    return packet;
}

UartPacket uart_packet_new_error(void) {
    UartPacket packet;
    packet.start = 0;
    return packet;
}

bool packet_error(const UartPacket *packet) {
    if (packet->start == 0) {
        return 1;
    }
    return 0;
}

UartPacket uart_packet_pack(const VecU8 *vec_u8) {
    if (vec_u8->length < 2 || vec_u8->data[0] != PACKET_START_CODE || vec_u8->data[vec_u8->length - 1] != PACKET_END_CODE) {
        return uart_packet_new_error();
    }
    VecU8 data_vec = vec_u8_new();
    vec_u8_extend_inner(&data_vec, vec_u8->data + 1, vec_u8->length - 2);
    return uart_packet_new(&data_vec);
}

VecU8 uart_packet_unpack(const UartPacket *packet) {
    VecU8 vec_u8 = vec_u8_new();
    vec_u8_push(&vec_u8, packet->start);
    vec_u8_extend_inner(&vec_u8, packet->data.data, packet->data.length);
    vec_u8_push(&vec_u8, packet->end);
    return vec_u8;
}

TrReBuffer tr_re_buffer_new(void) {
    TrReBuffer tr_re_buffer;
    tr_re_buffer.head = 0;
    tr_re_buffer.count = 0;
    return tr_re_buffer;
}

bool tr_re_buffer_push(TrReBuffer *tr_re_buffer, const UartPacket *packet) {
    if (tr_re_buffer->count >= TR_RE_BUFFER_CAP) return false;
    uint8_t tail = (tr_re_buffer->head + tr_re_buffer->count) % TR_RE_BUFFER_CAP;
    tr_re_buffer->packet[tail] = *packet;
    tr_re_buffer->count++;
    return true;
}

UartPacket tr_re_buffer_pop(TrReBuffer *tr_re_buffer) {
    if (tr_re_buffer->count == 0) return uart_packet_new_error();
    uint8_t head = tr_re_buffer->head;
    tr_re_buffer->head = (tr_re_buffer->head + 1) % TR_RE_BUFFER_CAP;
    tr_re_buffer->count--;
    return tr_re_buffer->packet[head];
}
