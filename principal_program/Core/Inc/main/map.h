#pragma once

#include "vehicle/basic.h"
#include "main/config.h"



#define INF 99999
#define MAX_NODE 10
#define NO_DATA -1

typedef int16_t   MapIdF;
typedef int8_t    MapDirF;
typedef uint16_t  MapDisF;

typedef struct {
    MapIdF id;
    MapDisF distance;
} Connection;

typedef struct{
    MapIdF local_id;
    Connection connect[8];
} Location;

typedef struct {
    MapIdF        address_id;
    MapDirF       direction;
    VehicleDirect vehicle_direction;
    MapDirF       real_rotate_count;
    VehicleMode   mode;
} MapData;

typedef struct{
    int8_t  current_count;
    MapData map_data[MAX_NODE];
} MapDataAll;

extern Location locations_t[MAX_NODE];
extern MapDataAll map_data_all;
extern MapData map_data_start;

void map_setup(void);
void map_bulid(MapIdF from, MapIdF to);
void map_data_renew_direction_and_address (
    MapData *map_new,
    MapIdF init_address_id,
    int8_t init_direction
);
