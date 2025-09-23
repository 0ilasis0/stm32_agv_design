#include "map_new/stack.h"

#include <stdlib.h>
#include <string.h>

#define STACK_DEFAULT_CAPACITY 8

struct Stack {
    void*            data;        // 連續儲存區
    size_t           elem_size;   // 每個元素大小
    size_t           capacity;    // 總容量
    size_t           top;         // 元素數（也表示下個可放置索引）
    stack_copy_fn    copy_fn;
    stack_free_fn    free_fn;
};

static int stack_grow(Stack* s, size_t min_capacity) {
    size_t new_cap = s->capacity ? s->capacity : STACK_DEFAULT_CAPACITY;
    while (new_cap < min_capacity) new_cap *= 2;
    void* new_ptr = realloc(s->data, new_cap * s->elem_size);
    if (!new_ptr) return -1;
    s->data = new_ptr;
    s->capacity = new_cap;
    return 0;
}

Stack* stack_create(size_t elem_size, size_t initial_capacity, stack_copy_fn copy_fn, stack_free_fn free_fn) {
    if (elem_size == 0) return NULL;
    Stack* s = (Stack*)malloc(sizeof(Stack));
    if (!s) return NULL;
    s->elem_size = elem_size;
    s->capacity = (initial_capacity == 0) ? STACK_DEFAULT_CAPACITY : initial_capacity;
    s->top = 0;
    s->copy_fn = copy_fn;
    s->free_fn = free_fn;
    s->data = malloc(s->capacity * s->elem_size);
    if (!s->data) {
        free(s);
        return NULL;
    }
    return s;
}

void stack_destroy(Stack* s) {
    if (!s) return;
    if (s->free_fn) {
        // 呼叫 free_fn 釋放每個元素的內部資源
        for (size_t i = 0; i < s->top; ++i) {
            void* elem_ptr = (char*)s->data + i * s->elem_size;
            s->free_fn(elem_ptr);
        }
    }
    free(s->data);
    free(s);
}

int stack_push(Stack* s, const void* elem) {
    if (!s || !elem) return -1;
    if (s->top >= s->capacity) {
        if (stack_grow(s, s->capacity * 2) != 0) return -1;
    }
    void* dest = (char*)s->data + s->top * s->elem_size;
    if (s->copy_fn) {
        s->copy_fn(dest, elem);
    } else {
        memcpy(dest, elem, s->elem_size);
    }
    s->top += 1;
    return 0;
}

int stack_pop(Stack* s, void* out_elem) {
    if (!s || s->top == 0) return -1;
    void* src = (char*)s->data + (s->top - 1) * s->elem_size;
    if (out_elem) {
        memcpy(out_elem, src, s->elem_size);
    }
    // 不在 pop 時釋放內部資源，以便呼叫者能取得元素
    s->top -= 1;
    return 0;
}

int stack_peek(const Stack* s, void* out_elem) {
    if (!s || s->top == 0 || !out_elem) return -1;
    void* src = (char*)s->data + (s->top - 1) * s->elem_size;
    memcpy(out_elem, src, s->elem_size);
    return 0;
}

size_t stack_size(const Stack* s) {
    if (!s) return 0;
    return s->top;
}

bool stack_is_empty(const Stack* s) {
    return (s == NULL) || (s->top == 0);
}

void stack_clear(Stack* s) {
    if (!s) return;
    if (s->free_fn) {
        for (size_t i = 0; i < s->top; ++i) {
            void* elem_ptr = (char*)s->data + i * s->elem_size;
            s->free_fn(elem_ptr);
        }
    }
    s->top = 0;
}

/*
說明：
- 本實作為泛型（generic）堆疊，使用者以 elem_size 指定元素大小，因此可儲存任何固定長度的資料結構或原始型別。
- 若元素內含指標或需要深拷貝/釋放，請在建立 stack 時傳入 copy_fn 與 free_fn：
    - copy_fn(dest, src)：把 src 的內容深拷貝到 dest（dest 已經有足夠空間），在 push 時會被呼叫。
    - free_fn(elem)：釋放 elem 內部資源（例如釋放內部指標），在 clear / destroy 時會被呼叫。
- pop 不會自動呼叫 free_fn（以免釋放後使用者無法取得資料）。若希望在 pop 時順便釋放，可在呼叫者端處理或另外包一個函式。
- 回傳值：0 表示成功，-1 表示失敗（如記憶體不足或參數錯誤）。
*/
