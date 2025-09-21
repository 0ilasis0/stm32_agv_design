#pragma once

#include "main/fn_state.h"
#include "main/config.h"
#include "main/vehicle.h"



#define INF 99999
#define MAX_NODE 10
#define NO_DATA 0xFF
#define TIME_INIT 3000



typedef uint32_t   MapIdF;
typedef uint8_t    MapDirF;
typedef uint8_t    MapCountF;
typedef uint16_t   MapDisF;

typedef struct Connection{
    MapIdF  id;
    MapDisF distance;
} Connection;

typedef struct Location{
    MapIdF     local_id;
    Connection connect[8];
} Location;

typedef struct MapData{
    MapIdF        address_id;
    MapDirF       direction;
    VehicleMotion vehicle_motion;
    MapCountF     need_rotate_count;
    VehicleMode   mode;
    Percentage    speed_setpoint;
} MapData;

typedef struct MapDataAll{
    MapCountF  current_count;
    MapData    map_data[MAX_NODE];
} MapDataAll;

typedef struct MapError{
    Result lose_navigation;
    Result no_path;
    Result input_start_id_err;
} MapError;



extern MapError map_error;

extern uint32_t time_start;
extern uint8_t final_node_count;
extern uint8_t textabc[3];

extern int graph[MAX_NODE][MAX_NODE];
extern int path[MAX_NODE][MAX_NODE];

extern bool map_enable;
extern bool map_toggle;

extern MapDataAll map_data_all;
extern MapData agv_state;

extern MapData map_data_init;
extern MapData map_data_start;



extern Location locations_t[MAX_NODE];

extern const Location locations_t_inner[MAX_NODE];
// extern const Location locations_t_inner[MAX_NODE];
// extern const Location locations_t_inner[MAX_NODE];
