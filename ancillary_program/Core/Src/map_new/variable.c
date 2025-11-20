#include "map_new/variable.h"

uint8_t final_node_count = 0;

bool    map_enable = false;

int graph[MAX_NODE][MAX_NODE];
int path[MAX_NODE][MAX_NODE];

MapError map_error = {0};

MapDataAll map_data_all;
MapDataNode agv_state;

MapDataNode map_data_init;
MapDataNode map_data_start;


MapDataNode map_data_init = {
    .address_id         = NO_DATA,
    .direction          = NO_DATA,
    .need_rotate_count  = NO_DATA,
    .vehicle_motion     = VEHICLE_MOTION_STOP,
    .mode               = VEHICLE_MODE_FREE,
    .speed_setpoint     = 0,
    .end_flag           = DISABLE,
};

MapDataNode map_data_start = {
    .address_id         = NO_DATA,
    .direction          = NO_DATA,
    .need_rotate_count  = NO_DATA,
    .vehicle_motion     = VEHICLE_MOTION_STOP,
    .mode               = VEHICLE_MODE_ROTATE,
    .speed_setpoint     = MAP_SETPOINT_ROTATE,
    .end_flag           = DISABLE,
};

Debug dbg = {
    .log = {},
    .map_main_count = 0,
};


//0 1505360132 -> 0
//1 858788143  -> 51 48 17 47
//2 592978984  -> 33 88 86 40
//3 3623155971 -> 45 249 244 3
Location locations_t[MAX_NODE];

const Location locations_t_inner[MAX_NODE] = {
    {1505360132,   { {858788143,10},     {0,0},      {0,0},    {0,0},      {0,0},      {0,0},      {0,0},      {0,0}       } },
    {858788143,    { {0,0},     {0,0},      {0,0},    {0,0},      {1505360132,10},      {0,0},      {0,0},      {0,0}       } },
};

// const Location locations_t_inner[MAX_NODE] = {
//     {1505360132,   { {858788143,80},     {592978984,100},      {0,0},    {0,0},      {0,0},      {0,0},      {0,0},      {0,0}       } },
//     {858788143,    { {3623155971,20},     {0,0},      {592978984,30},    {0,0},      {1505360132,80},      {0,0},      {0,0},      {0,0}       } },
//     {592978984,    { {0,0},     {0,0},      {0,0},    {0,0},      {0,0},      {1505360132,100},      {858788143,30},      {3623155971,20}       } },
//     {3623155971,   { {0,0},     {0,0},      {0,0},    {592978984,20},      {858788143,20},      {0,0},      {0,0},      {0,0}       } }
// };

// const Location locations_t_inner[MAX_NODE] = {
//     {1,    { {0,0},     {0,0},      {2,85},    {4,30},      {3,50},      {0,0},      {0,0},      {0,0}       } },
//     {2,    { {0,0},     {0,0},      {0,0},    {0,0},      {0,0},      {4,90},      {1,85},      {0,0}       } },
//     {3,    { {1,50},     {4,10},      {0,0},    {0,0},      {0,0},      {0,0},      {0,0},      {0,0}       } },
//     {4,    { {0,0},      {2,90},       {0,0},    {0,0},      {0,0},      {3,10},      {0,0},      {1,30}       } },
// };

// const Location locations_t_inner[MAX_NODE] = {
//     {5,     { {0,0},     {0,0},      {78,20},    {0,0},      {0,0},      {0,0},      {0,0},      {0,0}       } },
//     {78,    { {0,0},     {0,0},      {11,35},    {15,30},    {0,0},      {0,0},      {5,20},      {0,0}       } },
//     {11,    { {0,0},     {0,0},      {131,80},   {0,0},      {12,5},     {15,40},    {78,35},    {0,0}       } },
//     {12,    { {11,5},    {131,20},   {0,0},      {0,0},      {0,0},      {0,0},      {15,45},    {0,0}       } },
//     {131,   { {0,0},     {0,0},      {0,0},      {14,10},    {0,0},      {12,20},    {11,80},    {0,0}       } },
//     {14,    { {0,0},     {0,0},      {0,0},      {0,0},      {0,0},      {0,0},      {0,0},      {131,10}    } },
//     {15,    { {0,0},     {11,40},    {12,45},    {0,0},      {0,0},      {0,0},      {0,0},      {78,30}     } }
// };
