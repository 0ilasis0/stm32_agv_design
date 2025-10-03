#include "map_new/work_space.h"
#include "map_new/base.h"
#include "map_new/manager.h"



static MapIdF current_map_id;

MapIdF init_map_id = 0;

Queue* map_queue;


void work_space_set (void)
{
    map_queue = queue_create();
}

void run_map (MapIdF from, MapIdF to)
{
    // 若已設定起點且與本次輸入不符，視為錯誤
    if (map_data_start.address_id == NO_DATA)
    {
        map_error.input_start_id_err = RESULT_ERROR(RES_ERR_NOT_FOUND);
        dbg.log[5]++;
    }

    // 如果地圖建構失敗，傳回初始值
    if (!(map_bulid(from, to)))
    {
        enforce_stop();
        dbg.log[1]++;
        return;
    }

    map_trans(&agv_state);
}



MapIdF queue_debug[MAX_SIZE];
void main_work_space (MapIdF current_id, MapIdF next_id)
{
    // 如果是重製地圖則跳過 dequeue
    if (current_id != NO_DATA && next_id != NO_DATA)
    {
        run_map(current_id, next_id);
        dbg.log[4]++;
        return;
    }

    // 檢查當前 map_queue 狀況
    get_all_data(map_queue, queue_debug);
    // 先檢查 map_stack 是否為 NULL
    if (map_queue == NULL) {
        enforce_stop();
        dbg.log[2]++;
        return;
    }

    // 檢測是否還有工作
    MapIdF next_map_id;
    if(!dequeue(map_queue, &next_map_id))
    {
        enforce_stop();
        dbg.log[3]++;
        return;
    }

    if (init_map_id != NO_DATA)
    {
        run_map(init_map_id, next_map_id);
        init_map_id = NO_DATA;
    }
    else
    {
        run_map(current_map_id, next_map_id);
    }

    current_map_id = next_map_id;
}
