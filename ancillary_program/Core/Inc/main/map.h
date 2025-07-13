#pragma once

#include "main/config.h"
#include "main/vehicle.h"

#define INF 99999
#define MAX_NODE 10
#define NO_DATA 0xFF

typedef uint16_t   MapIdF;
typedef uint8_t    MapDirF;
typedef uint8_t    MapCountF;
typedef uint16_t   MapDisF;
typedef bool       MapErrorF;

typedef struct{
    MapErrorF lose_navigation;
    MapErrorF map_data_trans_error;
} MapError;

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
    VehicleMotion vehicle_direction;
    MapCountF  need_rotate_count;
    VehicleMode   mode;
    Percentage    speed_setpoint;
} MapData;

typedef struct{
    MapCountF  current_count;
    MapData map_data[MAX_NODE];
} MapDataAll;



extern Location locations_t[MAX_NODE];
extern MapDataAll map_data_all;
extern MapData map_data_start;

void map_bulid(MapIdF from, MapIdF to);
