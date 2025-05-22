#include "user/user_vec.h"
#include <string.h>

VecU8 vec_u8_new(void) {
    VecU8 vec = {
        .length = 0
    };
    return vec;
}

void vec_u8_push(VecU8 *vec_u8, const void *src, size_t src_len) {
    if (vec_u8->length + src_len > VECU8_MAX_CAPACITY) {
        return;
    }
    memcpy(vec_u8->data + vec_u8->length, src, src_len);
    vec_u8->length += src_len;
    return;
}

uint32_t swap32(uint32_t value) {
    return ((value & 0x000000FFU) << 24)
         | ((value & 0x0000FF00U) <<  8)
         | ((value & 0x00FF0000U) >>  8)
         | ((value & 0xFF000000U) >> 24);
}
void vec_u8_push_float(VecU8 *vec_u8, float value) {
    uint32_t u32;
    memcpy(&u32, &value, sizeof(u32));
    u32 = swap32(u32);
    vec_u8_push(vec_u8, &u32, sizeof(u32));
}

uint16_t swap16(uint16_t value) {
    return ((value & 0x00FFU) << 8)
         | ((value & 0xFF00U) >> 8);
}
void vec_u8_push_u16(VecU8 *vec_u8, uint16_t value) {
    uint16_t u16 = swap16(value);
    vec_u8_push(vec_u8, &u16, sizeof(u16));
}
