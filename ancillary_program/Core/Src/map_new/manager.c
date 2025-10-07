#include "map_new/manager.h"
#include "map_new/base.h"
#include "map_new/adjust_start.h"
#include "map_new/init.h"



static void decide_map_id_and_direction(MapIdF from, MapIdF to)
 {
    uint8_t count = 0;

    // 根據 path 矩陣追蹤從 from 到 to 的節點路徑
    while (from != to && count < MAX_NODE) {
        int next_node = path[from][to];
        map_data_all.map_node[count].address_id = locations_t[from].local_id;

        MapIdF direction_index = NO_DATA;

        // 找出當前節點連接到下一節點的方向
        for (MapIdF i = 0; i < 8; i++) {
            if (locations_t[from].connect[i].id == locations_t[next_node].local_id) {
                direction_index = i;
                break;
            }
        }

        map_data_all.map_node[count].direction = direction_index;
        from = next_node;
        count++;
    }

    map_data_all.map_node[count].address_id = locations_t[to].local_id;
    map_data_all.map_node[count].direction = map_data_all.map_node[count - 1].direction;
    map_data_all.map_node[count].end_flag = ENABLE;

    // 紀錄路徑節點數（不含終點）
    final_node_count = count;
}

bool map_bulid(MapIdF from, MapIdF to)
{
    from = get_index_by_id(from);
    to   = get_index_by_id(to);

    // 確認起點合法、圖上有路可走
    if (from == NO_DATA || to == NO_DATA|| graph[from][to] == INF) return false;

    map_data_all = init_map_data();

    decide_map_id_and_direction(from, to);

    for (size_t i = 0; i <= final_node_count; i++)
    {
        map_data_all.map_node[i].mode = decide_map_mode_and_speed(i, NO_DATA);

        if(i >= final_node_count - 1) continue;

        map_data_all.map_node[i + 1].vehicle_motion = decide_map_vehicle_motion(
            map_data_all.map_node[i].direction,
            map_data_all.map_node[i + 1].direction
        );

        map_data_all.map_node[i + 1].need_rotate_count = decide_need_rotate_count(
            map_data_all.map_node[i + 1].vehicle_motion,
            map_data_all.map_node[i + 1].address_id,
            map_data_all.map_node[i].direction,
            map_data_all.map_node[i + 1].direction
        );
    }

    map_adjust_start();

    // 決定map 0的mode
    map_data_all.map_node[0].mode = decide_map_mode_and_speed(0, map_data_start.direction);

    agv_state = map_data_all.map_node[0];

    return true;
}

void delete_locations_t_data(MapIdF id, MapDirF dir)
{
    id = get_index_by_id(id);
    if (id == NO_DATA)
    {
        return;
    }

    locations_t[id].connect[dir].distance = 0;
    locations_t[id].connect[dir].id       = 0;
}
