#include "main/map.h"
#include "vehicle/navigation.h"

static int graph[MAX_NODE][MAX_NODE];
static int path[MAX_NODE][MAX_NODE];
static uint8_t final_node_count = 0;

MapDataAll map_data_all;

MapData map_data_start = {
    .address_id         = NO_DATA,
    .direction          = NO_DATA,
    .real_rotate_count  = NO_DATA,
    .vehicle_direction  = VEHICLE_DIRECT_STOP,
    .mode               = VEHICLE_MODE_ROTATE,
};


Location locations_t[MAX_NODE] = {
    {5 , {{0,0}, {0,0}, {78,20}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}}},
    {78, {{0,0}, {0,0}, {11,35}, {15,30}, {0,0}, {0,0}, {0,0}, {0,0}}},
    {11, {{0,0}, {0,0}, {131,80}, {0,0}, {12,5}, {15,40}, {78,35}, {0,0}}},
    {12, {{11,5}, {131,20}, {0,0}, {0,0}, {0,0}, {0,0}, {15,45}, {0,0}}},
    {131, {{0,0}, {0,0}, {0,0}, {14,10}, {0,0}, {12,20}, {11,80}, {0,0}}},
    {14, {{0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {131,10}}},
    {15, {{0,0}, {11,40}, {12,45}, {0,0}, {0,0}, {0,0}, {0,0}, {78,30}}}
};



// 尋找節點 ID 對應的陣列索引
static MapIdF get_index_by_id(MapIdF id)
 {
    for (int i = 0; i < MAX_NODE; i++) {
        if (locations_t[i].local_id == id) return i;
    }
    return -1;
}

// 初始化 graph 距離矩陣與 path 路徑矩陣
static void init_map(void)
 {
    for (int i = 0; i < MAX_NODE; i++) {
        for (int j = 0; j < MAX_NODE; j++) {
            // 自己到自己距離為0，其他為無限大
            if (i == j) {
                graph[i][j] = 0;
            } else {
                graph[i][j] = INF;
            }
            path[i][j] = NO_DATA;                   // 初始化路徑為無路徑 (-1)
        }
    }

    // 依據 locations_t 中的連線設定距離與路徑
    for (int i = 0; i < MAX_NODE; i++) {
        for (int d = 0; d < 8; d++) {
            MapIdF id_to = locations_t[i].connect[d].id;
            int distance = locations_t[i].connect[d].distance;
            if (distance > 0) {
                MapIdF to_index = get_index_by_id(id_to);
                if (to_index != -1) {
                    graph[i][to_index] = distance;
                    graph[to_index][i] = distance;
                    path[i][to_index] = to_index;
                    path[to_index][i] = i;
                }
            }
        }
    }
}

static MapDataAll init_map_data (void)
{
    MapDataAll map_new;

    map_new.current_count = 0;
    for (uint8_t i = 0; i < MAX_NODE; i++) {
        map_new.map_data[i].real_rotate_count  = NO_DATA;
        map_new.map_data[i].direction          = NO_DATA;
        map_new.map_data[i].address_id         = NO_DATA;
        map_new.map_data[i].vehicle_direction  = NO_DATA;
        map_new.map_data[i].mode               = VEHICLE_MODE_IDLE;
    }

    return map_new;
}

// Floyd-Warshall 演算法計算所有節點對間最短路徑
static void floyd_warshall(void)
 {
    for (int k = 0; k < MAX_NODE; k++) {
        for (int i = 0; i < MAX_NODE; i++) {
            for (int j = 0; j < MAX_NODE; j++) {
                if (graph[i][k] + graph[k][j] < graph[i][j]) {
                    graph[i][j] = graph[i][k] + graph[k][j];
                    path[i][j] = path[i][k];
                }
            }
        }
    }
}

/*
 * 決定agv當前狀態
 */
static VehicleMode decide_vehicle_status(uint8_t count)
 {
    if (count == 0 && map_data_all.map_data[count].direction == NO_DATA) return VEHICLE_MODE_END;
    if (count == 0) return VEHICLE_DIRECT_FORWARD;

    if (map_data_all.map_data[count].direction == map_data_all.map_data[count - 1].direction)
    {
        return VEHICLE_MODE_TRACK;
    }
    else if (map_data_all.map_data[count].direction == NO_DATA)
    {
        return VEHICLE_MODE_END;
    }
    else
    {
        return VEHICLE_MODE_ROTATE;
    }
}

/**
  * @brief 判斷旋轉方向（順時針／逆時針）
  */
static VehicleDirect get_rotate_direction(MapDirF start_dir, MapDirF end_dir)
{
    if (end_dir == NO_DATA) return VEHICLE_DIRECT_STOP;

    MapDirF diff = (end_dir - start_dir + 8) % 8;

    if (diff == 0)
    {
        return VEHICLE_DIRECT_FORWARD;
    }
    else if (diff >= 4)
    {
        return VEHICLE_DIRECT_C_CLOCKWISE;
    }
    else
    {
        return VEHICLE_DIRECT_CLOCKWISE;
    }

}

/**
  * @brief 根據旋轉方向，計算在旋轉過程中會通過幾條磁條
  */
static int8_t decide_pass_magnetic_stripe_calculate(
    MapDirF rotate_direction_mode,
    uint16_t current_id_input,
    uint8_t from_dir,
    uint8_t to_dir
)
{
    uint8_t count = 0;

    // 取得目前節點（node）在 locations_t 中的索引值
    MapIdF current_id = get_index_by_id(current_id_input);

    if (rotate_direction_mode == VEHICLE_DIRECT_CLOCKWISE)
    {
        for (int i = (from_dir + 1) % 8; i != (to_dir + 1) % 8; i = (i + 1) % 8)
        {
            if (locations_t[current_id].connect[i].distance != 0)
            {
                count++;
            }
        }
    }
    else if (rotate_direction_mode == VEHICLE_DIRECT_C_CLOCKWISE)
    {
        for (int i = (from_dir - 1 + 8) % 8; i != (to_dir - 1 + 8) % 8; i = (i - 1 + 8) % 8)
        {
            if (locations_t[current_id].connect[i].distance != 0)
            {
                count++;
            }
        }
    }
    else
    {
        return NO_DATA;
    }

    //若原方向上也有磁條，表示會需加一次
    if (locations_t[current_id].connect[from_dir].distance != 0)
    {
        count++;
    }

    return count;
}

static void build_current_map_data(int from, int to)
 {
    uint8_t count = 0;

    // 根據 path 矩陣追蹤從 from 到 to 的節點路徑
    while (from != to && count < MAX_NODE) {
        int next_node = path[from][to];
        map_data_all.map_data[count].address_id = locations_t[from].local_id;

        int direction_index = NO_DATA;

        // 找出當前節點連接到下一節點的方向
        for (int i = 0; i < 8; i++) {
            if (locations_t[from].connect[i].id == locations_t[next_node].local_id) {
                direction_index = i;
                break;
            }
        }

        map_data_all.map_data[count].direction = direction_index;
        from = next_node;
        count++;
    }

    map_data_all.map_data[count].address_id = locations_t[to].local_id;
    map_data_all.map_data[count].direction = NO_DATA;

    // 紀錄路徑節點數（不含終點）
    final_node_count = count;
}

/**
  * @brief 偵測是否有初始方向數據，如果存在，則執行原地旋轉修正以對準起始航向
  */
static void map_adjust_start (void)
{
    if (map_data_start.address_id == NO_DATA) return;

    map_data_start.vehicle_direction = get_rotate_direction(
        map_data_start.direction,
        map_data_all.map_data[0].direction
        );

    map_data_start.real_rotate_count = decide_pass_magnetic_stripe_calculate(
        map_data_start.vehicle_direction,
        map_data_all.map_data[0].address_id,
        map_data_start.direction,
        map_data_all.map_data[0].direction
        );

    if (map_data_start.direction != map_data_all.map_data[0].direction)
    {
        map_data_all.map_data[0] = map_data_start;
    }
}

void map_setup(void)
 {
    init_map();

    map_data_all = init_map_data();

    floyd_warshall();
}

void map_data_renew_direction_and_address (MapData *map_new, MapIdF init_address_id, MapDirF init_direction)
{
    map_new->direction = init_direction;
    map_new->address_id = init_address_id;
}

void map_bulid(MapIdF from, MapIdF to)
{
    from = get_index_by_id(from);
    to   = get_index_by_id(to);

    build_current_map_data(from, to);

    for (int i = 0; i <= final_node_count; i++)
    {
        map_data_all.map_data[i].mode = decide_vehicle_status(i);

        if (i > 0)
        {
            map_data_all.map_data[i].vehicle_direction = get_rotate_direction(
                map_data_all.map_data[i - 1].direction,
                map_data_all.map_data[i].direction
            );

            map_data_all.map_data[i].real_rotate_count = decide_pass_magnetic_stripe_calculate(
                map_data_all.map_data[i].vehicle_direction,
                map_data_all.map_data[i].address_id,
                map_data_all.map_data[i - 1].direction,
                map_data_all.map_data[i].direction
            );
        }
    }

    map_adjust_start();

    agv_state = map_data_all.map_data[0];
}
