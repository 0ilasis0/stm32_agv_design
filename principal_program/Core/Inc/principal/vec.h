#ifndef VEC_H
#define VEC_H

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define VECU8_MAX_CAPACITY 38

typedef struct {
    uint8_t data[VECU8_MAX_CAPACITY];
    size_t  length;
} VecU8;

VecU8 vec_u8_new(void);
bool vec_u8_push(VecU8 *vec_u8, uint8_t byte);
bool vec_u8_extend_inner(VecU8 *vec_u8, const uint8_t *src, size_t src_len);
#define vec_u8_extend(vec_u8, src) \
    vec_u8_extend_inner(vec_u8, src, sizeof(src))
VecU8 u32_to_u8(uint32_t value);
VecU8 float_to_u8(float value);
VecU8 u16_to_u8(uint16_t value);

#endif
