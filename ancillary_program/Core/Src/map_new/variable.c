#include "map_new/variable.h"



uint8_t textabc[3] = {};
uint8_t final_node_count = 0;
uint32_t time_start;

bool    map_enable = false;
bool    map_toggle = false;

int graph[MAX_NODE][MAX_NODE];
int path[MAX_NODE][MAX_NODE];

MapError map_error = {0};

MapDataAll map_data_all;
MapData agv_state;

MapData map_data_init;
MapData map_data_start;


MapData map_data_init = {
    .address_id         = NO_DATA,
    .direction          = NO_DATA,
    .need_rotate_count  = NO_DATA,
    .vehicle_motion     = VEHICLE_MOTION_STOP,
    .mode               = VEHICLE_MODE_FREE,
    .speed_setpoint     = 0,
};

MapData map_data_start = {
    .address_id         = NO_DATA,
    .direction          = NO_DATA,
    .need_rotate_count  = NO_DATA,
    .vehicle_motion  = VEHICLE_MOTION_STOP,
    .mode               = VEHICLE_MODE_ROTATE,
    .speed_setpoint     = MAP_SETPOINT_ROTATE,
};



// 858788143  -> 51 48 17 47
// 592978984  -> 33 88 86 40
// 3623155971 -> 45 249 244 3
// 1505360132 -> 0
Location locations_t[MAX_NODE];

const Location locations_t_inner[MAX_NODE] = {
    {1505360132,   { {858788143,5},     {0,0},      {0,0},    {0,0},      {0,0},      {0,0},      {0,0},      {0,0}       } },
    {858788143,    { {3623155971,85},     {592978984,10},      {0,0},    {0,0},      {1505360132,5},      {0,0},      {0,0},      {0,0}       } },
    {592978984,    { {0,0},     {0,0},      {0,0},    {0,0},      {0,0},      {858788143,10},      {0,0},      {3623155971,20}       } },
    {3623155971,   { {0,0},     {0,0},      {0,0},    {592978984,20},      {858788143,85},      {0,0},      {0,0},      {0,0}       } }
};

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
