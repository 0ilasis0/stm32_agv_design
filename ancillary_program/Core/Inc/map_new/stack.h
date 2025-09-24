#pragma once

#include <stddef.h>
#include <stdbool.h>



typedef struct Stack Stack;

// 元件複製與釋放函式（可為 NULL）
// copy_fn: 當需要把使用者傳入的元素複製到內部儲存時呼叫（深拷貝）
// free_fn: 當釋放元素內部資源（在 clear / destroy 時）呼叫
typedef void (*stack_copy_fn)(void* dest, const void* src);
typedef void (*stack_free_fn)(void* elem);

// 建立一個 stack
// elem_size: 每個元素（記憶體區塊）的大小（bytes）
// initial_capacity: 初始容量（若為 0 則使用預設）
// copy_fn, free_fn: 可為 NULL
Stack* stack_create(size_t elem_size, size_t initial_capacity, stack_copy_fn copy_fn, stack_free_fn free_fn);

// 釋放一個 stack（會呼叫 free_fn 釋放元素內部資源）
void stack_destroy(Stack* s);

// push/pop/peek
// 回傳 0 表示成功，-1 表示失敗（例如記憶體不足或空棧）
int stack_push(Stack* s, const void* elem);
int stack_pop(Stack* s, void* out_elem); // 若 out_elem 為 NULL 則表示僅移除（不回傳）
int stack_peek(const Stack* s, void* out_elem); // 複製頂元素到 out_elem

// 資訊函式
size_t stack_size(const Stack* s);
bool stack_is_empty(const Stack* s);

// 清空（會呼叫 free_fn 釋放元素內部資源），但不釋放 stack 結構本身
void stack_clear(Stack* s);
