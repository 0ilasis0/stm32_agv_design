#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>
#include <stdbool.h>
#include "vec.h"

#define PACKET_START_CODE  ((uint8_t) '{')
#define PACKET_END_CODE    ((uint8_t) '}')
#define PACKET_DATA_MAX_SIZE 38
#define PACKET_MAX_SIZE (PACKET_DATA_MAX_SIZE + 2)

typedef struct {
    uint8_t start;
    VecU8   data;
    uint8_t end;
} UartPacket;
UartPacket uart_packet_new(VecU8 *data);
bool packet_error(const UartPacket *packet);
UartPacket uart_packet_pack(const VecU8 *vec_u8);
VecU8 uart_packet_unpack(const UartPacket *packet);

#define TR_RE_BUFFER_CAP 5
typedef struct {
    UartPacket packet[TR_RE_BUFFER_CAP];
    uint8_t head;
    uint8_t count;
} TrReBuffer;
TrReBuffer tr_re_buffer_new(void);
bool tr_re_buffer_push(TrReBuffer *tr_re_buffer, const UartPacket *packet);
UartPacket tr_re_buffer_pop(TrReBuffer *tr_re_buffer);

#endif
