#pragma once

#include "main/config.h"
#include "main/fn_state.h"

#define print_Direction(fmt, value)                                            \
    do {                                                                       \
        printf((fmt), Direction_to_string((value)));                           \
    } while (0)

#ifdef __GNUC__
#else
#error "UNWRAP_RESULT 需要 GCC/Clang 的 statement-expression 支持"
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

typedef enum Direction { NORTH, SOUTH, EAST, WEST } Direction;

typedef struct Entry {
    int key;
    Direction value;
    struct Entry* next;
} Entry;

typedef struct SuccessResult {
    void* obj;
} SuccessResult;

typedef enum ErrorType {
    
    RES_ERR_EMPTY,
    RES_ERR_FULL,
    RES_ERR_MEMORY_ERROR,
    RES_ERR_NOT_FOUND,
    RES_ERR_INVALID,
    RES_ERR_REMOVE_FAIL,

    /*new_insert_function*/
    RES_ERR_BUSY,
    RES_ERR_TIMEOUT,
    //RES_ERR_OVERFLOW,  跟RES_ERR_FULL, 重疊到
    RES_ERR_NOT_MOVE,
    RES_ERR_FAIL,

} ErrorType;
/*
typedef enum ErrorType {
    False -> RES_ERR_INVALID = -1,
    RES_ERR_FAIL,
    MemoryError -> RES_ERR_MEMORY_ERROR,
    RES_ERR_BUSY,
    RES_ERR_TIMEOUT,
    Empty -> RES_ERR_EMPTY,
    Full -> RES_ERR_FULL,
    RES_ERR_OVERFLOW,
    NotFound -> RES_ERR_NOT_FOUND,
    RES_ERR_NOT_MOVE,
    RemoveFail -> RES_ERR_REMOVE_FAIL,
} ErrorType;
*/
typedef struct Result {
    bool is_ok;
    union {
        SuccessResult success;
        ErrorType error;
    } result;
} Result;

typedef struct HashMap {
    Entry** buckets;
    size_t capacity;
    size_t size;
} HashMap;
Result HashMap_new(size_t initial_capacity);
Result HashMap_destroy(HashMap* g_map);
Result HashMap_insert(HashMap* g_map, int key, Direction value);
Result HashMap_get(HashMap* g_map, int key);
Result HashMap_remove(HashMap* g_map, int key);
Result HashMap_contains(HashMap* g_map, int key);
Result HashMap_size(HashMap* g_map);
Result HashMap_isFull(HashMap* g_map);

const char* ErrorType_to_string(ErrorType error);
const char* Direction_to_string(Direction* dir);
