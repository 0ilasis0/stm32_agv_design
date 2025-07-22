#include "HashMap.h"

static size_t hash_int(int key, size_t capacity) {
    unsigned int k = (unsigned int)key;
    k ^= k >> 16;
    k *= 0x7feb352d;
    k ^= k >> 15;
    return (size_t)(k % capacity);
}
// static bool hashmap_resize(HashMap* g_map, size_t new_capacity) {
//     Entry** new_buckets = calloc(new_capacity, sizeof(Entry*));
//     if (!new_buckets)
//         return false;
//     for (size_t i = 0; i < g_map->capacity; i++) {
//         Entry* e = g_map->buckets[i];
//         while (e) {
//             Entry* next = e->next;
//             size_t idx = hash_int(e->key, new_capacity);
//             e->next = new_buckets[idx];
//             new_buckets[idx] = e;
//             e = next;
//         }
//     }
//     free(g_map->buckets);
//     g_map->buckets = new_buckets;
//     g_map->capacity = new_capacity;
//     return true;
// }
Result HashMap_new(size_t initial_capacity) {
    HashMap* g_map = malloc(sizeof(HashMap));
    CHECK_MAP_EXISTS();
    g_map->capacity =
        initial_capacity > 0 ? initial_capacity : INITIAL_CAPACITY;  //capacity預設初始大小
    g_map->size = 0;   // 你已經使用的空間數
    g_map->buckets = calloc(g_map->capacity, sizeof(Entry*));
    if (!g_map->buckets) {
        free(g_map);
        return RESULT_ERROR(RES_ERR_MEMORY_ERROR);
    }
    return RESULT_OK(g_map);
}
Result HashMap_destroy(HashMap* g_map) {
    CHECK_MAP_EXISTS();
    for (size_t i = 0; i < g_map->capacity; i++) {
        Entry* e = g_map->buckets[i];
        while (e) {
            Entry* next = e->next;
            free(e);
            e = next;
        }
    }
    free(g_map->buckets);
    free(g_map);
    g_map = NULL;
    return RESULT_OK(NULL);
}
Result HashMap_insert(HashMap* g_map, int key, Direction value) {
    CHECK_MAP_EXISTS();
    // 如果需要自動擴容
    // if ((double)(g_map->size + 1) / g_map->capacity > LOAD_FACTOR) {
    //     if (!hashmap_resize(g_map->capacity * 2))
    //         return RESULT_ERROR(RES_ERR_MEMORY_ERROR);
    // }
    if (HashMap_isFull(g_map).is_ok)
        return RESULT_ERROR(RES_ERR_FULL);
    size_t idx = hash_int(key, g_map->capacity);
    Entry* e = g_map->buckets[idx];
    while (e) {
        if (e->key == key) {
            e->value = value;  //尋找重複向
            return RESULT_OK(NULL);
        }
        e = e->next;
    }
    Entry* new_entry = malloc(sizeof(Entry));
    if (!new_entry)
        return RESULT_ERROR(RES_ERR_MEMORY_ERROR);
    new_entry->key = key;
    new_entry->value = value;
    new_entry->next = g_map->buckets[idx];
    g_map->buckets[idx] = new_entry;
    g_map->size++;
    return RESULT_OK(NULL);
}
Result HashMap_get(HashMap* g_map, int key) {
    CHECK_MAP_EXISTS();
    size_t idx = hash_int(key, g_map->capacity);
    Entry* e = g_map->buckets[idx];
    while (e) {
        if (e->key == key) {
            return RESULT_OK(&(e->value));
        }
        e = e->next;
    }
    return RESULT_ERROR(RES_ERR_NOT_FOUND);
}
Result HashMap_remove(HashMap* g_map, int key) {
    CHECK_MAP_EXISTS();
    size_t idx = hash_int(key, g_map->capacity);
    Entry* e = g_map->buckets[idx];
    Entry* prev = NULL;
    while (e) {
        if (e->key == key) {
            if (prev)
                prev->next = e->next;
            else
                g_map->buckets[idx] = e->next;
            free(e);
            g_map->size--;
            return RESULT_OK(NULL);
        }
        prev = e;
        e = e->next;
    }
    return RESULT_ERROR(RES_ERR_REMOVE_FAIL);
}
Result HashMap_contains(HashMap* g_map, int key) {   //不用CHECK_RESULT，我只在乎有沒有
    Result obj = HashMap_get(g_map, key);
    if (obj.is_ok) {
        return RESULT_BOOL(true);
    } else {
        return RESULT_BOOL(false);
    }
}
Result HashMap_size(HashMap* g_map) {
    CHECK_MAP_EXISTS();
    return RESULT_OK(&(g_map->size));
}
Result HashMap_isFull(HashMap* g_map) {
    // Full -> True
    // Not Full -> False
    CHECK_MAP_EXISTS();
    return RESULT_BOOL(g_map->size >= g_map->capacity);
}


const char* ErrorType_to_string(ErrorType error) {
    switch (error) {
    case RES_ERR_EMPTY:
        return "RES_ERR_EMPTY";
    case RES_ERR_FULL:
        return "RES_ERR_FULL";
    case RES_ERR_MEMORY_ERROR:
        return "RES_ERR_MEMORY_ERROR";
    case RES_ERR_NOT_FOUND:
        return "RES_ERR_NOT_FOUND";
    case RES_ERR_INVALID:
        return "";
    case RES_ERR_REMOVE_FAIL:
        return "RES_ERR_REMOVE_FAIL";
    
    case RES_ERR_BUSY:
        return "RES_ERR_BUSY";
    
    case RES_ERR_TIMEOUT:
        return "RES_ERR_TIMEOUT";
    
    case RES_ERR_NOT_MOVE:
        return "RES_ERR_NOT_MOVE";
    
    case RES_ERR_FAIL:
        return "RES_ERR_FAIL";

    default:
        return "Unknown Error";
    }
}

const char* Direction_to_string(Direction* dir) {
    switch (*dir) {
    case NORTH:
        return "North";
    case SOUTH:
        return "South";
    case EAST:
        return "East";
    case WEST:
        return "West";
    default:
        return "Unknown Direction";
    }
}
