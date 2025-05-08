#include "principal/vec.h"

VecU8 vec_u8_new(void) {
    VecU8 vec_u8;
    vec_u8.length  = 0;
    return vec_u8;
}

bool vec_u8_push(VecU8 *vec_u8, uint8_t byte) {
    if (vec_u8->length >= VECU8_MAX_CAPACITY) return false;
    vec_u8->data[vec_u8->length++] = byte;
    return true;
}

bool vec_u8_extend_inner(VecU8 *vec_u8, const uint8_t *src, size_t src_len) {
    if (vec_u8->length + src_len > VECU8_MAX_CAPACITY) return false;
    memcpy(vec_u8->data + vec_u8->length, src, src_len);
    vec_u8->length += src_len;
    return true;
}

VecU8 u32_to_u8(uint32_t value) {
    VecU8 vec_u8 = vec_u8_new();
    uint8_t src[] = {
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >>  8) & 0xFF,
        (value >>  0) & 0xFF,
    };
    vec_u8_extend(&vec_u8, src);
    return vec_u8;
}

VecU8 float_to_u8(float value) {
    uint32_t u32_buf;
    memcpy(&u32_buf, &value, sizeof(u32_buf));
    return u32_to_u8(u32_buf);
}

VecU8 u16_to_u8(uint16_t value) {
    VecU8 vec_u8 = vec_u8_new();
    uint8_t src[] = {
        (value >>  8) & 0xFF,
        (value >>  0) & 0xFF,
    };
    vec_u8_extend(&vec_u8, src);
    return vec_u8;
}
