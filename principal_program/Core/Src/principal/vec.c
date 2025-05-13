#include "principal/vec.h"
#include <string.h>

// 建立新的 VecU8 結構並初始化長度為 0
// Create a new VecU8 structure and initialize length to 0
VecU8 vec_u8_new(void) {
    VecU8 vec_u8;
    vec_u8.length  = 0;
    return vec_u8;
}

// 將 byte 推入 VecU8，若已達容量上限則返回 false
// Push a byte into VecU8, return false if max capacity reached
bool vec_u8_push(VecU8 *vec_u8, uint8_t byte) {
    if (vec_u8->length >= VECU8_MAX_CAPACITY) return false;
    vec_u8->data[vec_u8->length++] = byte;
    return true;
}

// 在 VecU8 後續添加 src 陣列資料，若超過容量則返回 false
// Extend VecU8 with src array data, return false if exceeding capacity
bool vec_u8_extend_inner(VecU8 *vec_u8, const uint8_t *src, uint8_t src_len) {
    if (vec_u8->length + src_len > VECU8_MAX_CAPACITY) return false;
    memcpy(vec_u8->data + vec_u8->length, src, src_len);
    vec_u8->length += src_len;
    return true;
}

// 將 32 位整數轉換為大端序 uint8_t 向量
// Convert a 32-bit integer to a big-endian uint8_t vector
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

// 將 float 值按 IEEE 754 原位轉為 uint8_t 向量
// Convert a float value to a uint8_t vector by copying its IEEE 754 representation
VecU8 float_to_u8(float value) {
    uint32_t u32_buf;
    memcpy(&u32_buf, &value, sizeof(u32_buf));
    return u32_to_u8(u32_buf);
}

// 將 16 位整數轉換為大端序 uint8_t 向量
// Convert a 16-bit integer to a big-endian uint8_t vector
VecU8 u16_to_u8(uint16_t value) {
    VecU8 vec_u8 = vec_u8_new();
    uint8_t src[] = {
        (value >>  8) & 0xFF,
        (value >>  0) & 0xFF,
    };
    vec_u8_extend(&vec_u8, src);
    return vec_u8;
}
