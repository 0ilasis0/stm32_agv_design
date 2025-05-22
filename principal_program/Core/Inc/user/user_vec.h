#ifndef USER_VEC_U8_H
#define USER_VEC_U8_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define VECU8_MAX_CAPACITY  255

typedef struct {
    uint8_t data[VECU8_MAX_CAPACITY];
    size_t  length;
} VecU8;

VecU8 vec_u8_new(void);
void vec_u8_push(VecU8 *vec_u8, const void *src, size_t src_len);
void vec_u8_push_float(VecU8 *vec_u8, float value);
void vec_u8_push_u16(VecU8 *vec_u8, uint16_t value);

#endif
