#include "map/main.h"
#include "connectivity/fdcan/main.h"
#include "rfid/main.h"

static int graph[MAX_NODE][MAX_NODE];
static int path[MAX_NODE][MAX_NODE];

static bool    map_enable = false;
static bool    map_toggle = false;
static uint8_t final_node_count = 0;

static MapDataAll map_data_all;
static MapData agv_state;

static MapData map_data_init = {
    .address_id         = NO_DATA,
    .direction          = NO_DATA,
    .need_rotate_count  = NO_DATA,
    .vehicle_motion     = VEHICLE_MOTION_STOP,
    .mode               = VEHICLE_MODE_FREE,
    .speed_setpoint     = 0,
};

static MapData map_data_start = {
    .address_id         = NO_DATA,
    .direction          = NO_DATA,
    .need_rotate_count  = NO_DATA,
    .vehicle_motion  = VEHICLE_MOTION_STOP,
    .mode               = VEHICLE_MODE_ROTATE,
    .speed_setpoint     = MAP_SETPOINT_ROTATE,
};

static MapError map_error = {
    .lose_navigation        = FNS_OK,
    .no_path                = FNS_OK,
};

static Location locations_t[MAX_NODE];
static const Location locations_t_inner[MAX_NODE] = {
    {5,     { {0,0},     {0,0},      {78,20},    {0,0},      {0,0},      {0,0},      {0,0},      {0,0}       } },
    {78,    { {0,0},     {0,0},      {11,35},    {15,30},    {0,0},      {0,0},      {5,20},      {0,0}       } },
    {11,    { {0,0},     {0,0},      {131,80},   {0,0},      {12,5},     {15,40},    {78,35},    {0,0}       } },
    {12,    { {11,5},    {131,20},   {0,0},      {0,0},      {0,0},      {0,0},      {15,45},    {0,0}       } },
    {131,   { {0,0},     {0,0},      {0,0},      {14,10},    {0,0},      {12,20},    {11,80},    {0,0}       } },
    {14,    { {0,0},     {0,0},      {0,0},      {0,0},      {0,0},      {0,0},      {0,0},      {131,10}    } },
    {15,    { {0,0},     {11,40},    {12,45},    {0,0},      {0,0},      {0,0},      {0,0},      {78,30}     } }
};

static void map_trans (const MapData* trans_map)
{
    // text
    // return;
    // text

    VecByte vec_byte;
    if (ERROR_CHECK_FNS_RAW(vec_byte_new(&vec_byte, FDCAN_VEC_BYTE_CAP)))
    {
    }

    if (
           ERROR_CHECK_FNS_RAW(pkt_vehi_set_motion(&vec_byte, trans_map->vehicle_motion))
        || ERROR_CHECK_FNS_RAW(fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_VEHI_ID))
    ) {
    }
    if (
           ERROR_CHECK_FNS_RAW(pkt_vehi_set_mode(&vec_byte, trans_map->mode, 0))
        || ERROR_CHECK_FNS_RAW(fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_VEHI_ID))
    ) {
    }
    if (
           ERROR_CHECK_FNS_RAW(pkt_vehi_set_mode(&vec_byte, trans_map->mode, trans_map->need_rotate_count))
        || ERROR_CHECK_FNS_RAW(fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_VEHI_ID))
    ) {
    }
    if (
           ERROR_CHECK_FNS_RAW(pkt_vehi_set_speed(&vec_byte, trans_map->speed_setpoint))
        || ERROR_CHECK_FNS_RAW(fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_VEHI_ID))
    ) {
    }

    vec_byte_free(&vec_byte);
}

static MapDirF opposite_direction (MapDirF dir)
{
    return (dir + 4) % 8;
}

// 尋找節點 ID 對應的陣列索引
static MapIdF get_index_by_id (MapIdF id)
 {
    for (uint8_t i = 0; i < MAX_NODE; i++) {
        if (locations_t[i].local_id == id) return i;
    }
    return -1;
}

// Floyd-Warshall 演算法計算所有節點對間最短路徑
static void floyd_warshall (void)
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
        map_new.map_data[i] = map_data_init;
    }

    map_new.map_data[0].vehicle_motion = VEHICLE_MOTION_FORWARD;

    return map_new;
}

static void map_set (void)
 {
    init_map();
    floyd_warshall();
}

/*
 * 決定agv當前狀態
 */
static VehicleMode decide_map_mode_and_speed(uint8_t count)
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
  * @brief 判斷旋轉方向（順時針／逆時針）
  */
static VehicleMotion decide_map_vehicle_motion(MapDirF start_dir, MapDirF end_dir)
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

/**
  * @brief 根據旋轉方向，計算在旋轉過程中會通過幾條磁條
  */
static MapCountF decide_need_rotate_count(
    MapDirF rotate_direction_mode,
    MapIdF current_id_input,
    MapDirF from_dir,
    MapDirF to_dir
)
{
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
    if (locations_t_inner[current_id].connect[from_dir].distance != 0)
    {
        count++;
    }

    return count;
}

static void decide_map_id_and_direction(int from, int to)
 {
    uint8_t count = 0;

    // 根據 path 矩陣追蹤從 from 到 to 的節點路徑
    while (from != to && count < MAX_NODE) {
        int next_node = path[from][to];
        map_data_all.map_data[count].address_id = locations_t[from].local_id;

        MapIdF direction_index = NO_DATA;

        // 找出當前節點連接到下一節點的方向
        for (MapIdF i = 0; i < 8; i++) {
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

static void delete_locations_t_data(MapIdF id, MapDirF dir)
{
    id = get_index_by_id(id);

    locations_t[id].connect[dir].distance = 0;
    locations_t[id].connect[dir].id       = 0;
}

/**
  * @brief 偵測是否有初始方向數據，如果存在，則執行原地旋轉修正以對準起始航向
  */
static void map_adjust_start (void)
{
    if (map_data_start.address_id == NO_DATA) return;

    map_data_start.vehicle_motion = decide_map_vehicle_motion(
        map_data_start.direction,
        map_data_all.map_data[0].direction
        );

    map_data_start.need_rotate_count = decide_need_rotate_count(
        map_data_start.vehicle_motion,
        map_data_all.map_data[0].address_id,
        map_data_start.direction,
        map_data_all.map_data[0].direction
        );

    if (map_data_start.direction != map_data_all.map_data[0].direction)
    {
        map_data_all.map_data[0] = map_data_start;
    }
}

static void map_data_renew_direction_and_address (
    MapData *map_new,
    MapIdF address_id,
    MapDirF direction
    ) {
    *map_new = map_data_init;

    map_new->direction = direction;
    map_new->address_id = address_id;
}

static void map_bulid(MapIdF from, MapIdF to)
{
    from = get_index_by_id(from);
    to   = get_index_by_id(to);

    // 確認起點合法、圖上有路可走
    if (from == -1 || to == -1 || graph[from][to] == INF) {
        ERROR_STOP_MAP_RETURN(map_error.no_path, FNS_FAIL);
    }

    map_data_all = init_map_data();

    decide_map_id_and_direction(from, to);

    for (uint8_t i = 0; i <= final_node_count; i++)
    {
        map_data_all.map_data[i].mode = decide_map_mode_and_speed(i);

        if(i >= final_node_count - 1) continue;

        map_data_all.map_data[i + 1].vehicle_motion = decide_map_vehicle_motion(
            map_data_all.map_data[i].direction,
            map_data_all.map_data[i + 1].direction
        );

        map_data_all.map_data[i + 1].need_rotate_count = decide_need_rotate_count(
            map_data_all.map_data[i + 1].vehicle_motion,
            map_data_all.map_data[i + 1].address_id,
            map_data_all.map_data[i].direction,
            map_data_all.map_data[i + 1].direction
        );
    }

    map_adjust_start();

    agv_state = map_data_all.map_data[0];
}

void map_windows (MapIdF from, MapIdF to)
{
    map_enable = true;
    map_bulid(from, to);
    map_trans(&agv_state);
}

int yy = 1;
int tick_ttt = 0;
void StartMapTask(void *argument)
{
    // osThreadExit();

    memcpy(locations_t, locations_t_inner, sizeof(locations_t));
    map_set();

    // text
    // map_data_renew_direction_and_address(&map_data_start, 5, 5);
    map_windows(5, 14);
    // text

    for(;;)
    {
        // text
        tick_ttt++;
        // text

        // map flag
        if (spi2_rfid.state == CARD_STATE_EXIST && !map_toggle) map_toggle = true;
        if (spi2_rfid.state == CARD_STATE_NONE && map_toggle)   map_toggle = false;

        if(map_enable)
        // if(map_enable && map_toggle)
        {
            // 讀到RFID執行給資料到另一個stm32
            if (spi2_rfid.uid32 == map_data_all.map_data[map_data_all.current_count + 1].address_id || tick_ttt % 100 == 0)
            // if (spi2_rfid.uid32 == map_data_all.map_data[map_data_all.current_count + 1].address_id)
            {
                map_data_all.current_count ++;
                agv_state = map_data_all.map_data[map_data_all.current_count];

                if (agv_state.mode == VEHICLE_MODE_END)
                {
                    map_data_renew_direction_and_address(
                        &map_data_start,
                        map_data_all.map_data[final_node_count].address_id,
                        map_data_all.map_data[final_node_count - 1].direction
                        );
                    map_enable = false;

                    // text
                    map_windows(14, 5);
                    // text
                }

                map_trans(&agv_state);
            }
            // 如果循跡應該往前結果讀到原本的rfid而非下一個rfid，代表遇上障礙，進行地圖重製
            else if (spi2_rfid.uid32 == map_data_all.map_data[map_data_all.current_count].address_id || (tick_ttt % 330 == 0 && yy))
            // else if (spi2_rfid.uid32 == map_data_all.map_data[map_data_all.current_count].address_id)
            {
                // text
                yy = 0;
                // text

                // 先傳送停止動作，等地圖計算完畢
                agv_state = map_data_init;
                map_trans(&agv_state);

                MapData map_data_temp = map_data_all.map_data[map_data_all.current_count];
                MapIdF  target_id     = map_data_all.map_data[final_node_count].address_id;

                delete_locations_t_data(
                    map_data_all.map_data[map_data_all.current_count].address_id,
                    map_data_all.map_data[map_data_all.current_count].direction
                    );
                delete_locations_t_data(
                    map_data_all.map_data[map_data_all.current_count + 1].address_id,
                    opposite_direction(map_data_all.map_data[map_data_all.current_count].direction)
                    );

                map_set();
                map_data_renew_direction_and_address(
                    &map_data_start,
                    map_data_temp.address_id,
                    map_data_temp.direction
                    );
                map_bulid(map_data_temp.address_id, target_id);
                map_trans(&agv_state);
            }
            // 只知道現在位置不知道方向，所以停止動作，目前測試所以槓掉
            // else
            // {
            //     ERROR_STOP_MAP_RETURN(map_error.lose_navigation, FNS_FAIL);
            // }
        }

        osDelay(10);
    }
}
