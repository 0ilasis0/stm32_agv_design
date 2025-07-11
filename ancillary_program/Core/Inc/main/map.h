#pragma once

#include "connectivity/vehicle.h"
#include "main/config.h"
#include "main/vehicle.h"



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
    Percentage    speed_setpoint;
} MapData;

typedef struct{
    int8_t  current_count;
    MapData map_data[MAX_NODE];
} MapDataAll;


extern Location locations_t[MAX_NODE];
extern MapDataAll map_data_all;
extern MapData map_data_start;

void map_bulid(MapIdF from, MapIdF to);
