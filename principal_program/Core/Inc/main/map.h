#pragma once

#include "main/config.h"
#include "vehicle/basic.h"


#define INF 99999
#define MAX_NODE 10
#define NO_DATA -1

typedef int16_t MapIdF;
typedef int8_t  MapDirF;

typedef struct {
    MapIdF id;
    int distance;
} Connection;

typedef struct{
    MapIdF local_id;
    Connection connect[8];
} Location;

typedef struct{
    int8_t          current_count;
    MapIdF          address_id[MAX_NODE];
    MapDirF         direction[MAX_NODE];
    VehicleDirect   vehicle_direction[MAX_NODE];
    MapDirF         real_rotate_count[MAX_NODE];
    VehicleMode     mode[MAX_NODE];
} MapData;

typedef struct {
    MapIdF        address_id;
    MapDirF       direction;
    VehicleDirect vehicle_direction;
    MapDirF       real_rotate_count;
    VehicleMode   mode;
} MapDataCurrent;

extern Location locations_t[MAX_NODE];
extern MapData map_data;
extern MapDataCurrent map_data_start;

void map_setup(void);
void map_bulid(MapIdF from, MapIdF to);
void map_data_renew_direction_and_address (
    MapDataCurrent *map_new,
    MapIdF init_address_id,
    int8_t init_direction);
