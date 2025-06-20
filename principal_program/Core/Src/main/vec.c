#include "main/vec.h"
#include <stdlib.h>
#include <string.h>
#include "main/config.h"

FnState vec_u8_new(Vec_U8 *self, uint16_t capacity)
{
    if (capacity == 0 || capacity > VECU8_MAX_CAPACITY) return FNS_FAIL;
    self->data = malloc(capacity * sizeof(*self->data));
    if (!self->data) return FNS_ERR_OOM;
    self->capacity = capacity;
    self->head     = 0;
    self->len      = 0;
    return FNS_OK;
}

FnState vec_u8_free(Vec_U8 *self)
{
    if (self->data) {
        free(self->data);
        self->data = NULL;
    }
    self->capacity = 0;
    self->head     = 0;
    self->len      = 0;
    return FNS_OK;
}

FnState vec_u8_realign(Vec_U8 *self)
{
    if (self->len == 0 || self->head == 0) return FNS_OK;
    uint16_t first_part = self->capacity - self->head;
    if (first_part >= self->len)
    {
        memmove(self->data, self->data + self->head, self->len);
    }
    else
    {
        uint16_t second_part = self->len - first_part;
        uint8_t tmp[second_part];
        memcpy(tmp, self->data, second_part);
        memmove(self->data, self->data + self->head, first_part);
        memmove(self->data + first_part, tmp, second_part);
    }
    self->head = 0;
    return FNS_OK;
}

FnState vec_u8_get_byte(const Vec_U8 *self, uint8_t *u8, uint16_t id)
{
    if (self->len == 0) return FNS_BUF_EMPTY;
    if (id >= self->len) return FNS_FAIL;
    uint16_t idx = (self->head + id) % self->capacity;
    *u8 = self->data[idx];
    return FNS_OK;
}

FnState vec_u8_starts_with(const Vec_U8 *self, const uint8_t *pre, uint16_t pre_len)
{
    if (self->len < pre_len) return FNS_NO_MATCH;
    if (
        (self->head + pre_len <= self->capacity) &&
        (memcmp(self->data + self->head, pre, pre_len) == 0)
    ) return FNS_OK;
    uint16_t first_part  = self->capacity - self->head;
    uint16_t remaining = pre_len - first_part;
    if (memcmp(self->data + self->head, pre, first_part) != 0) return FNS_NO_MATCH;
    if (memcmp(self->data, pre + first_part, remaining) != 0) return FNS_NO_MATCH;
    return FNS_OK;
}

FnState vec_u8_push(Vec_U8 *self, const void *src, uint16_t src_len)
{
    if (self->len + src_len > self->capacity) return FNS_BUF_OVERFLOW;
    uint16_t tail = self->head + self->len;
    if (
        (tail >= self->capacity) ||
        (tail + src_len >= self->capacity)
    )
    {
        vec_u8_realign(self);
        tail = self->len;
    }
    memcpy(self->data + tail, src, src_len);
    self->len += src_len;
    return FNS_OK;
}

inline FnState vec_u8_push_byte(Vec_U8 *self, uint8_t value)
{
    return vec_u8_push(self, &value, 1);
}

/**
 * @brief 交換 16-bit 整數的大小端
 *
 * @param value 要交換大小端的 16-bit 值
 * 
 * @return uint16_t 交換後的 16-bit 值
 */
static inline uint16_t swap16(const uint16_t value)
{
    return  ((value & 0x00FFU) << 8) |
            ((value & 0xFF00U) >> 8);
}

FnState vec_u8_push_u16(Vec_U8 *self, uint16_t value)
{
    uint16_t u16 = swap16(value);
    return vec_u8_push(self, &u16, sizeof(u16));
}

/**
 * @brief 交換 32-bit 整數的大小端
 *
 * @param value 要交換大小端的 32-bit 值
 * 
 * @return uint32_t 交換後的 32-bit 值
 */
static inline uint32_t swap32(uint32_t value)
{
    return  ((value & 0x000000FFU) << 24) |
            ((value & 0x0000FF00U) <<  8) | 
            ((value & 0x00FF0000U) >>  8) | 
            ((value & 0xFF000000U) >> 24);
}

FnState vec_u8_push_f32(Vec_U8 *self, float value)
{
    uint32_t u32;
    uint8_t u32_len = sizeof(u32);
    memcpy(&u32, &value, u32_len);
    u32 = swap32(u32);
    return vec_u8_push(self, &u32, u32_len);
}

inline FnState vec_u8_rm_all(Vec_U8 *self)
{
    self->head = 0;
    self->len  = 0;
    return FNS_OK;
}

FnState vec_u8_rm_range(Vec_U8 *self, uint16_t offset, uint16_t size)
{
    if (offset >= self->len) return FNS_FAIL;
    if (size == 0) return FNS_OK;
    if (size >= self->len) return vec_u8_rm_all(self);
    if (offset == 0)
    {
        self->head = (self->head + size) % self->capacity;
        self->len -= size;
        return FNS_OK;
    }
    if (offset + size >= self->len)
    {
        self->len = offset;
        return FNS_OK;
    }
    vec_u8_realign(self);
    memmove(self->data + offset, self->data + (offset + size), self->len - (offset + size));
    self->len -= size;
    return FNS_OK;
}
