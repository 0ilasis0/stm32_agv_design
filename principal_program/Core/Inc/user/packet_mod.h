#ifndef USER_PACKET_H
#define USER_PACKET_H

#include <stdint.h>
#include <stdbool.h>
#include "user/vec_mod.h"

#define PACKET_START_CODE  ((uint8_t) '{')
#define PACKET_END_CODE    ((uint8_t) '}')
// 50
#define PACKET_MAX_SIZE VECU8_MAX_CAPACITY
#define PACKET_DATA_MAX_SIZE (VECU8_MAX_CAPACITY - 2)

typedef struct {
    uint8_t     start;
    VecU8       data_vec_u8;
    uint8_t     end;
} UartPacket;
UartPacket uart_packet_new(VecU8 *data);
bool packet_error(const UartPacket *packet);
VecU8 uart_packet_get_vec(const UartPacket *packet);
UartPacket uart_packet_pack(const VecU8 *vec_u8);
void uart_packet_add_data(UartPacket *packet, const VecU8 *vec_u8);
VecU8 uart_packet_unpack(const UartPacket *packet);

#endif
