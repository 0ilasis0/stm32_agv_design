#include "map_new/window.h"
#include "map_new/stack.h"
#include "map_new/base.h"
#include "map_new/work_space.h"



void map_window_init_work (MapIdF init_id, MapDirF init_dir)
{
    stack_push(map_stack, &init_id);
    map_data_renew_direction_and_address(&map_data_start, init_id, init_dir);

    if (stack_pop(map_stack, &init_map_id) != 0) init_map_id = 0;
}

void map_window_add_work (MapIdF add_id)
{
    stack_push(map_stack, &add_id);
}

void map_window_clear_work (void)
{
    stack_clear(map_stack);
}

void map_window_start_work (void)
{
    map_enable = true;
    work_space_main();
}
