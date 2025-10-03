#pragma once

#include "map_new/variable.h"
#include "map_new/queue.h"


extern MapIdF init_map_id;
extern Queue* map_queue;


void run_map (MapIdF, MapIdF);
void work_space_set (void);
void main_work_space (MapIdF current_id, MapIdF next_id);
