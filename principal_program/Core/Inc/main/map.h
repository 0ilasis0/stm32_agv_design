#pragma once

#include <stdint.h>

#define INF 99999
#define max_node 10
#define no_data -1

typedef enum {
    agv_straight,                                   // 循跡狀態mode
    agv_rotate,                                     // 原地旋轉mode
    agv_end,                                        // 直行mode
    agv_next
} AGV_STATUS;

typedef enum {
    clockwise,
    counter_clockwise,
    either
} ROTATE_STATUS;

typedef struct {
    int id;
    int distance;
} CONNECTION;

typedef struct {
    int local_id;
    CONNECTION connect[8];
} LOCATION;

typedef struct {
    int8_t          start_direction;
    int8_t          start_address_id;
    int8_t          current_count;
    int8_t          direction[max_node];
    uint16_t        address_id[max_node];
    AGV_STATUS      status[max_node];
} MAP_DATA;

extern MAP_DATA map_data;
extern LOCATION locations_t[max_node];

void map_setup(void);
void init_map(void);
MAP_DATA init_map_data_direction_and_address (MAP_DATA *map_new, int8_t init_address_id, int8_t init_direction);
MAP_DATA init_map_data (
    int8_t init_start_direction,
    int8_t init_start_address_id,
    int8_t init_current_count,
    int8_t init_direction[max_node],
    uint16_t init_address_id[max_node],
    AGV_STATUS init_status[max_node]);
void floyd_warshall(void);
void build_current_map_data(int from, int to);
int get_index_by_id(int id);
AGV_STATUS decide_vehicle_status(uint8_t count);
