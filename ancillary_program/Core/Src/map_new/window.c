#include "map_new/window.h"
#include "map_new/base.h"
#include "map_new/work_space.h"



void map_window_init_work (MapIdF init_id, MapDirF init_dir)
{
    enqueue(map_queue, &init_id);
    map_data_renew_direction_and_address(&map_data_start, init_id, init_dir);

    if (!dequeue(map_queue, &init_map_id)) init_map_id = 0;
}

void map_window_add_work (MapIdF add_id)
{
    enqueue(map_queue, &add_id);
}

void map_window_clear_work (void)
{
    clear_queue(map_queue);
}

void map_window_start_work (void)
{
    map_enable = true;
    main_work_space(NO_DATA, NO_DATA);
}
