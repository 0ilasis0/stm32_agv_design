#pragma once

#include "main/config.h"
#include "main/fn_state.h"
#include "map/simple.h"

#define print_Direction(fmt, value)                                            \
    do {                                                                       \
        printf((fmt), Direction_to_string((value)));                           \
    } while (0)

#ifdef __GNUC__
#else
#error "RESULT_UNWRAP_HANDLE 需要 GCC/Clang 的 statement-expression 支持"
#endif
#define CHECK_MAP_EXISTS()                                                     \
    do {                                                                       \
        if (!g_map) {                                                          \
            return RESULT_ERROR(RES_ERR_MEMORY_ERROR);                                  \
        }                                                                      \
    } while (0)

    // 將重複的讀鍵邏輯抽成宏，減少 if/else
#define READ_KEY(var) \
    do { \
        printf("輸入 key："); \
        if (scanf("%d", &(var)) != 1) { \
            fprintf(stderr, "鍵輸入錯誤。\n"); \
            clear_input_buffer(); \
            break; \
        } \
    } while (0)
#define INITIAL_CAPACITY 16
#define LOAD_FACTOR 0.75

typedef struct Entry {
    uint32_t key;
    SimpleDirect value;
    struct Entry* next;
} Entry;

typedef struct HashMap {
    Entry** buckets;
    size_t capacity;
    size_t size;
} HashMap;

Result HashMap_new(size_t initial_capacity);
Result HashMap_destroy(HashMap* g_map);
Result HashMap_insert(HashMap* g_map, uint32_t key, SimpleDirect value);
Result HashMap_get(HashMap* g_map, uint32_t key);
Result HashMap_remove(HashMap* g_map, uint32_t key);
Result HashMap_contains(HashMap* g_map, uint32_t key);
Result HashMap_size(HashMap* g_map);
Result HashMap_isFull(HashMap* g_map);

