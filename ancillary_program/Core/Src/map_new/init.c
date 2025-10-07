#include "map_new/init.h"
#include "map_new/base.h"



// 初始化 graph 距離矩陣與 path 路徑矩陣
static void init_map (void)
 {
    for (uint8_t i = 0; i < MAX_NODE; i++) {
        for (uint8_t j = 0; j < MAX_NODE; j++) {
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
    for (uint8_t i = 0; i < MAX_NODE; i++) {
        for (uint8_t d = 0; d < 8; d++) {
            MapIdF id_to = locations_t[i].connect[d].id;
            int distance = locations_t[i].connect[d].distance;
            if (distance > 0) {
                MapIdF to_index = get_index_by_id(id_to);
                if (to_index != NO_DATA) {
                    graph[i][to_index] = distance;
                    graph[to_index][i] = distance;
                    path[i][to_index] = to_index;
                    path[to_index][i] = i;
                }
            }
        }
    }
}

MapDataAll init_map_data (void)
{
    MapDataAll map_new;

    map_new.current_count = 0;
    for (uint8_t i = 0; i < MAX_NODE; i++) {
        map_new.map_node[i] = map_data_init;
    }

    map_new.map_node[0].vehicle_motion = VEHICLE_MOTION_FORWARD;

    return map_new;
}

void map_set (void)
 {
    init_map();
    floyd_warshall();
}
