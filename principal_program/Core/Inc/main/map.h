#pragma once

#include "main/config.h"
#include "vehicle/basic.h"


#define INF 99999
#define MAX_NODE 10
#define NO_DATA -1

typedef int16_t MapIdF;
typedef int8_t  MapDirF;

typedef enum {
    agv_straight,                                   // 循跡狀態mode
    agv_rotate,                                     // 原地旋轉mode
    agv_end,                                        // 直行mode
    agv_next,
    agv_idle,                                       //初始化或待命
} AgvStatus;

typedef struct {
    int id;
    int distance;
} Connection;

typedef struct{
    int local_id;
    Connection connect[8];
} Location;

typedef struct{
    int8_t          current_count;
    VehicleDirect   currnet_mode[MAX_NODE];
    MapDirF         direction[MAX_NODE];
    MapDirF         real_rotate_count[MAX_NODE];
    MapIdF          address_id[MAX_NODE];
    AgvStatus       status[MAX_NODE];
} MapData;

typedef struct  {
    MapIdF        address_id;
    MapDirF       direction;
    VehicleDirect currnet_mode;
    MapDirF       real_rotate_count;
    AgvStatus     status;
} MapDataStart;

extern Location locations_t[MAX_NODE];
extern MapData map_data;
extern MapDataStart map_data_start;

void map_setup(void);
void map_bulid(MapIdF from, MapIdF to);
void init_map_data_direction_and_address (
    MapDataStart *map_new,
    MapIdF init_address_id,
    int8_t init_direction);
void map_adjust_startup_heading (void);
