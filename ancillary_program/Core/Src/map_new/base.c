#include "map_new/base.h"
#include "map_new/base.h"
#include "connectivity/fdcan/main.h"
#include "connectivity/fdcan/pkt_write.h"

void map_trans (const MapData* trans_map)
{
    FdcanPkt *pkt = RESULT_UNWRAP_HANDLE(fdcan_pkt_pool_alloc());
    RESULT_CHECK_HANDLE(pkt_vehi_set_motion(pkt, trans_map->vehicle_motion));
    RESULT_CHECK_HANDLE(fdcan_pkt_buf_push(&fdcan_trsm_pkt_buf, pkt));
    pkt = RESULT_UNWRAP_HANDLE(fdcan_pkt_pool_alloc());
    RESULT_CHECK_HANDLE(pkt_vehi_set_mode(pkt, trans_map->mode, trans_map->need_rotate_count));
    RESULT_CHECK_HANDLE(fdcan_pkt_buf_push(&fdcan_trsm_pkt_buf, pkt));
    pkt = RESULT_UNWRAP_HANDLE(fdcan_pkt_pool_alloc());
    RESULT_CHECK_HANDLE(pkt_vehi_set_speed(pkt, trans_map->speed_setpoint));
    RESULT_CHECK_HANDLE(fdcan_pkt_buf_push(&fdcan_trsm_pkt_buf, pkt));
}

// Floyd-Warshall 演算法計算所有節點對間最短路徑
void floyd_warshall (void)
 {
    for (uint8_t k = 0; k < MAX_NODE; k++) {
        for (uint8_t i = 0; i < MAX_NODE; i++) {
            for (uint8_t j = 0; j < MAX_NODE; j++) {
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
VehicleMode decide_map_mode_and_speed(uint8_t count)
 {
    if (count == 0 && map_data_all.map_data[count].direction == NO_DATA)
    {
        map_data_all.map_data[count].speed_setpoint = MAP_SETPOINT_STOP;
        return VEHICLE_MODE_END;
    }
    if (count == 0)
    {
        map_data_all.map_data[count].speed_setpoint = MAP_SETPOINT_TRACK;
        return VEHICLE_MODE_TRACK;
    }

    if (map_data_all.map_data[count].direction == map_data_all.map_data[count - 1].direction)
    {
        map_data_all.map_data[count].speed_setpoint = MAP_SETPOINT_TRACK;
        return VEHICLE_MODE_TRACK;
    }
    else if (map_data_all.map_data[count].direction == NO_DATA)
    {
        map_data_all.map_data[count].speed_setpoint = MAP_SETPOINT_STOP;
        return VEHICLE_MODE_END;
    }
    else
    {
        map_data_all.map_data[count].speed_setpoint = MAP_SETPOINT_ROTATE;
        return VEHICLE_MODE_ROTATE;
    }
}

/**
  * @brief 根據旋轉方向，計算在旋轉過程中會通過幾條磁條
  */
MapCountF decide_need_rotate_count(
    MapDirF rotate_direction_mode,
    MapIdF current_id_input,
    MapDirF from_dir,
    MapDirF to_dir
) {
    if (current_id_input == map_data_all.map_data[final_node_count].address_id) return 0;

    uint8_t count = 0;

    // 取得目前節點（node）在 locations_t 中的索引值
    MapIdF current_id = get_index_by_id(current_id_input);

    if (rotate_direction_mode == VEHICLE_MOTION_CLOCKWISE)
    {
        for (int8_t i = (from_dir + 1) % 8; i != (to_dir + 1) % 8; i = (i + 1) % 8)
        {
            if (locations_t[current_id].connect[i].distance != 0)
            {
                count++;
            }
        }
    }
    else if (rotate_direction_mode == VEHICLE_MOTION_C_CLOCKWISE)
    {
        for (int8_t i = (from_dir - 1 + 8) % 8; i != (to_dir - 1 + 8) % 8; i = (i - 1 + 8) % 8)
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
    // if (locations_t_inner[current_id].connect[from_dir].distance != 0)
    // {
    //     count++;
    // }

    return count;
}

/**
  * @brief 判斷旋轉方向（順時針／逆時針）
  */
VehicleMotion decide_map_vehicle_motion(MapDirF start_dir, MapDirF end_dir)
{
    MapDirF diff = (end_dir - start_dir + 8) % 8;

    if (diff == 0)
    {
        return VEHICLE_MOTION_FORWARD;
    }
    else if (diff >= 4)
    {
        return VEHICLE_MOTION_C_CLOCKWISE;
    }
    else
    {
        return VEHICLE_MOTION_CLOCKWISE;
    }

}

void map_data_renew_direction_and_address (
    MapData *map_new,
    MapIdF address_id,
    MapDirF direction
) {
    *map_new = map_data_init;

    map_new->direction = direction;
    map_new->address_id = address_id;
}

// 尋找節點 ID 對應的陣列索引
MapIdF get_index_by_id (MapIdF id)
{
    for (uint8_t i = 0; i < MAX_NODE; i++) {
        if (locations_t[i].local_id == id) return i;
    }
    return  NO_DATA;
}

MapDirF opposite_direction (MapDirF dir)
{
    return (dir + 4) % 8;
}
