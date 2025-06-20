#pragma once
// ----------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include "main/fn_state.h"
// ----------------------------------------------------------------------------------------------------

typedef struct Vec_U8
{
    uint8_t*    data;
    uint16_t    capacity;
    uint16_t    head;
    uint16_t    len;
} Vec_U8;

/**
 * @brief 初始化 Vec_U8，動態配置緩衝區
 * 
 * @param self 指向 Vec_U8 實例的指標
 * @param capacity 要配置的容量（bytes），須大於 0
 * 
 * @return FNS_OK    成功
 * @return FNS_ERR_OOM 配置失敗（記憶體不足）
 * @return FNS_FAIL  輸入參數錯誤（如 capacity==0）
 */
FnState vec_u8_new(Vec_U8 *self, uint16_t capacity);
/**
 * @brief 釋放 Vec_U8 佔用的資源，並重置欄位
 * 
 * @param self 指向 Vec_U8 實例的指標
 * 
 * @return FNS_OK
 */
FnState vec_u8_free(Vec_U8 *self);
/**
 * @brief 把 Vec_U8 裡的資料「搬到索引 0 開始」(head = 0)，並保留原本的儲存順序
 *
 * @param self 指向 Vec_U8 實例的指標
 * 
 * @return FNS_OK 重新對齊成功
 */
FnState vec_u8_realign(Vec_U8 *self);
/**
 * @brief 從 VecU8 中，讀取相對於 head 的第 id 個位元組
 *
 * @param self 指向 Vec_U8 實例的指標
 * @param u8 用來存放讀出位元組的位址參考
 * @param id 欲讀取的偏移量（相對 head 的索引，範圍須在 0 ~ len-1 之間）
 *
 * @return FNS_OK 已被填入對應值  
 * @return FNS_BUF_EMPTY 緩衝區為空
 * @return FNS_FAIL id 超出範圍
 */
FnState vec_u8_get_byte(const Vec_U8 *self, uint8_t *u8, uint16_t id);
/**
 * @brief 檢查 Vec_U8 起始位置是否以指定序列開頭
 * 
 * @param self 指向 Vec_U8 實例的指標
 * @param pre 指向要比對的序列
 * @param pre_len 序列長度
 * 
 * @return FNS_OK 開頭吻合
 * @return false 否則 (false otherwise)
 */
FnState vec_u8_starts_with(const Vec_U8 *self, const uint8_t *pre, uint16_t pre_len);
/**
 * @brief 將 src 指向的位元組組合並推入 Vec_U8 末端
 *
 * @param self 指向 Vec_U8 實例的指標
 * @param src 指向要推入的資料緩衝區
 * @param src_len 要推入的資料長度
 * 
 * @return FNS_OK 成功推入
 * @return FNS_BUF_OVERFLOW 推入失敗（超過容量）
 */
FnState vec_u8_push(Vec_U8 *self, const void *src, uint16_t src_len);
/**
 * @brief 將一 byte 推入 Vec_U8
 *
 * @param self 指向 Vec_U8 實例的指標
 * @param value 要推入的原始值
 * 
 * @return FNS_OK 成功推入
 * @return FNS_BUF_OVERFLOW 推入失敗（超過容量）
 */
FnState vec_u8_push_byte(Vec_U8 *self, uint8_t value);
/**
 * @brief 將原始值轉換為 IEEE-754 大端序並推入 Vec_U8
 * 
 * @param self 指向 Vec_U8 實例的指標
 * @param value 要推入的原始值
 * 
 * @return FNS_OK 成功推入
 * @return FNS_BUF_OVERFLOW 推入失敗（超過容量）
 */
FnState vec_u8_push_u16(Vec_U8 *self, uint16_t value);
/**
 * @brief 將原始值轉換為 IEEE-754 大端序並推入 Vec_U8
 * 
 * @param self 指向 Vec_U8 實例的指標
 * @param value 要推入的原始值
 * 
 * @return FNS_OK 成功推入
 * @return FNS_BUF_OVERFLOW 推入失敗（超過容量）
 */
FnState vec_u8_push_f32(Vec_U8 *self, float value);
/**
 * @brief 從 Vec_U8 中移除全部資料
 * 
 * @param self   指向 Vec_U8 實例的指標
 * 
 * @return FNS_OK 成功移除
 */
FnState vec_u8_rm_all(Vec_U8 *self);
/**
 * @brief 從 Vec_U8 中移除指定範圍的資料
 * 
 * @param self   指向 Vec_U8 實例的指標
 * @param offset 要移除區段在目前資料（以 head 為起點）的起始位移
 * @param size   要移除的 byte 長度
 * 
 * @return FNS_OK 成功移除
 * @return FNS_FAIL offset 超過目前資料長度或 realign 失敗
 */
FnState vec_u8_rm_range(Vec_U8 *self, uint16_t offset, uint16_t size);
