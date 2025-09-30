#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "map_new/variable.h" // 假設 MapIdF 在這裡定義



#define MAX_SIZE 20

typedef struct Node {
    MapIdF val;
    struct Node* next;
} Node;

typedef struct Queue {
    Node* front;
    Node* rear;
} Queue;

// 建立新 queue
Queue* queue_create(void);

// 釋放 queue（同時清空節點）
void queue_destroy(Queue* q);

// push
void enqueue(Queue* q, MapIdF* val);

// pop，將值存入 *val，成功回傳 true，空隊列回傳 false
bool dequeue(Queue* q, MapIdF* val);

// 清空 queue
void clear_queue(Queue* q);

// 將 queue 所有資料存入陣列 arr，回傳元素個數
uint32_t get_all_data(Queue* q, MapIdF* arr);
