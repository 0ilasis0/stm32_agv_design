#ifndef PACKET_PROC_MOD_H
#define PACKET_PROC_MOD_H

#include <stdint.h>
#include <stdbool.h>
#include "user/packet_mod.h"

#define TR_RE_PKT_BUFFER_CAP 5

typedef struct {
    UartPacket  packet[TR_RE_PKT_BUFFER_CAP];
    uint8_t     head;
    uint8_t     length;
} TransceiveBuffer;
extern TransceiveBuffer transfer_buffer;
extern TransceiveBuffer receive_buffer;
TransceiveBuffer transceive_buffer_new(void);
bool transceive_buffer_push(TransceiveBuffer *transceive_buffer, const UartPacket *packet);
bool transceive_buffer_pop(TransceiveBuffer *buffer, UartPacket *packet);
bool transceive_buffer_pop_firstHalf(const TransceiveBuffer *buffer, UartPacket *packet);
bool transceive_buffer_pop_secondHalf(TransceiveBuffer *transceive_buffer);

void uart_transmit_pkt_proc(bool *flag);
void uart_receive_pkt_proc(bool *flag, uint8_t count);

#endif
