#include "main/map.h"
#include <stdio.h>
#include <stdint.h>

int graph[max_node][max_node];
int path[max_node][max_node];
uint8_t final_node_count = 0;

LOCATION locations_t[max_node] = {
    {78, {{0,0}, {0,0}, {11,35}, {15,30}, {0,0}, {0,0}, {0,0}, {0,0}}},
    {11, {{0,0}, {0,0}, {131,80}, {0,0}, {12,5}, {15,40}, {78,35}, {0,0}}},
    {12, {{11,5}, {131,20}, {0,0}, {0,0}, {0,0}, {0,0}, {15,45}, {0,0}}},
    {131, {{0,0}, {0,0}, {0,0}, {14,10}, {0,0}, {12,20}, {11,80}, {0,0}}},
    {14, {{0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {131,10}}},
    {15, {{0,0}, {11,40}, {12,45}, {0,0}, {0,0}, {0,0}, {0,0}, {78,30}}}
};

MAP_DATA map_data;
/*
MAP_DATA map_data = {
    0,                                                            //current_count
    {1, 7, 3, 3, no_data},                                        //direction
    {0, 1, 2, 3, 4}                                               //address_id
    {agv_straight, agv_rotate, agv_rotate, agv_straight, agv_end}                                                        // status
};
*/



void map_setup(void) {

    int text_from = get_index_by_id(78);
    int text_to = get_index_by_id(14);

    map_init();
    floyd_warshall();
    map_data_new(no_data);
    build_current_map_data(text_from, text_to);

    for (int i = 0; i <= final_node_count; i++) {
        map_data.status[i] = decide_vehicle_status(i);
    }

}

// 初始化 graph 距離矩陣與 path 路徑矩陣
void map_init(void) {
    for (int i = 0; i < max_node; i++) {
        for (int j = 0; j < max_node; j++) {
            graph[i][j] = (i == j) ? 0 : INF;  // 自己到自己距離為0，其他為無限大
            path[i][j] = -1;                   // 初始化路徑為無路徑 (-1)
        }
    }

    // 依據 locations_t 中的連線設定距離與路徑
    for (int i = 0; i < max_node; i++) {
        for (int d = 0; d < 8; d++) {
            int id_to = locations_t[i].connect[d].id;
            int distance = locations_t[i].connect[d].distance;
            if (distance > 0) {
                int to_index = get_index_by_id(id_to);
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

MAP_DATA map_data_new (int8_t start_direction) {
    MAP_DATA map_new;
    map_new.start_direction = start_direction;

    return map_new;
}

// Floyd-Warshall 演算法計算所有節點對間最短路徑
void floyd_warshall(void) {
    for (int k = 0; k < max_node; k++) {
        for (int i = 0; i < max_node; i++) {
            for (int j = 0; j < max_node; j++) {
                if (graph[i][k] + graph[k][j] < graph[i][j]) {
                    graph[i][j] = graph[i][k] + graph[k][j];
                    path[i][j] = path[i][k];
                }
            }
        }
    }
}

void build_current_map_data(int from, int to) {
    int count = 0;

    // 根據 path 矩陣追蹤從 from 到 to 的節點路徑
    while (from != to && count < max_node) {
        int next_node = path[from][to];
        map_data.address_id[count] = locations_t[from].local_id;

        int direction_index = no_data;

        // 找出當前節點連接到下一節點的方向
        for (int i = 0; i < 8; i++) {
            if (locations_t[from].connect[i].id == locations_t[next_node].local_id) {
                direction_index = i;
                break;
            }
        }

        map_data.direction[count] = direction_index;
        from = next_node;
        count++;
    }

    map_data.address_id[count] = locations_t[to].local_id;
    map_data.direction[count] = no_data;

    // 紀錄路徑節點數（不含終點）
    final_node_count = count;
}

// 尋找節點 ID 對應的陣列索引
int get_index_by_id(int id) {
    for (int i = 0; i < max_node; i++) {
        if (locations_t[i].local_id == id) return i;
    }
    return -1;
}



/*
 * 決定agv當前狀態
 */
AGV_STATUS decide_vehicle_status(uint8_t count) {
    if (count == 0 && map_data.direction[count] == no_data) return agv_end;
    if (count == 0) return agv_straight;

    if (map_data.direction[count] == map_data.direction[count - 1]) {
        return agv_straight;

    } else if (map_data.direction[count] == no_data){
        return agv_end;

    } else {
        return agv_rotate;

    }
}
