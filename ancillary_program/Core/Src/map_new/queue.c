#include "map_new/queue.h"



// 建立新 queue
Queue* queue_create(){
    Queue* q = (Queue*)malloc(sizeof(Queue));
    q->front = NULL;
    q->rear = NULL;
    return q;
}

// 釋放 queue
void queue_destroy(Queue* q){
    clear_queue(q);
    free(q);
}

// push
void enqueue(Queue* q, uint32_t *val){
    Node* newNode = (Node*)malloc(sizeof(Node));
    newNode->val = *val;
    newNode->next = NULL;
    if(q->rear) q->rear->next = newNode;
    q->rear = newNode;
    if(!q->front) q->front = newNode;
}

// pop，將值存入 *val，成功回傳 1，空隊列回傳 0
bool dequeue(Queue* q, uint32_t* val){
    if(!q->front) return 0; // 空隊列

    Node* temp = q->front;
    *val = temp->val;       // 將值存入傳入變數
    q->front = q->front->next;
    if(!q->front) q->rear = NULL;
    free(temp);
    return 1; // 成功
}


// clear
void clear_queue(Queue* q){
    while(q->front){
        Node* temp = q->front;
        q->front = q->front->next;
        free(temp);
    }
    q->rear = NULL;
}

// get_all_data
uint32_t get_all_data(Queue* q, uint32_t* arr){
    int count = 0;
    Node* cur = q->front;
    while(cur && count < MAX_SIZE){
        arr[count++] = cur->val;
        cur = cur->next;
    }
    return count;
}