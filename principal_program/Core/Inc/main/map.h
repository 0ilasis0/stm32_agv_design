#pragma once

#include "main/config.h"
#include "vehicle/vehicle2.h"

#define INF 99999
#define max_node 10
#define no_data -1

typedef enum {
    agv_straight,                                   // 循跡狀態mode
    agv_rotate,                                     // 原地旋轉mode
    agv_end,                                        // 直行mode
    agv_next,
    agv_idle,                                       //初始化或待命
} AGV_STATUS;

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

typedef struct {
    uint16_t        address_id;
    int8_t          direction;
    VehicleMotion   vehicle_currnet_mode;
} AgvState;

extern MAP_DATA map_data;
extern AgvState agv_state;
extern LOCATION locations_t[max_node];

void map_setup(void);
static AgvState init_agv_state_data (void);
void init_map_data_direction_and_address (MAP_DATA *map_new, int8_t init_address_id, int8_t init_direction);
void build_current_map_data(int from, int to);
int get_index_by_id(int id);
