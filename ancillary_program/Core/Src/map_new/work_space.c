#include "map_new/work_space.h"
#include "map_new/base.h"
#include "map_new/manager.h"
#include "map_new/stack.h"



static MapIdF init_id = 0;
static MapIdF current_map_id;
static Stack* map_stack;



void work_space_set (void)
{
    map_stack = stack_create(sizeof(MapIdF), 0, NULL, NULL);

    stack_push(map_stack, &locations_t_inner[0].local_id);
    stack_push(map_stack, &locations_t_inner[2].local_id);
    stack_push(map_stack, &locations_t_inner[1].local_id);

    if (stack_pop(map_stack, &init_id) != 0) init_id = 0;
}

void run_map (MapIdF from, MapIdF to)
{
    // 若已設定起點且與本次輸入不符，視為錯誤
    if (map_data_start.address_id != from && map_data_start.address_id != NO_DATA)
    {
        map_error.input_start_id_err = RESULT_ERROR(RES_ERR_NOT_FOUND);
    }

    map_enable = true;
    map_bulid(from, to);

    // 如果地圖建構失敗，傳回初始值
    if (map_enable == false)
    {
        enforce_stop();
        return;
    }

    map_trans(&agv_state);

    time_start = HAL_GetTick();
}


void work_space_main (void)
{
    MapIdF next_map_id;

    // 先檢查 map_stack 是否為 NULL
    if (map_stack == NULL || stack_is_empty(map_stack)) {
        enforce_stop();
        return;
    }
    // 檢測是否還有工作
    if( stack_pop(map_stack, &next_map_id) != 0)
    {
        enforce_stop();
        return;
    }

    if (init_id != NO_DATA)
    {
        run_map(init_id, next_map_id);
        init_id = NO_DATA;
    }
    else
    {
        run_map(current_map_id, next_map_id);
    }

    current_map_id = next_map_id;
}
