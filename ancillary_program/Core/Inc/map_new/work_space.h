#pragma once

#include "map_new/variable.h"
#include "map_new/stack.h"



extern Stack* map_stack;
extern MapIdF init_map_id;



void run_map (MapIdF, MapIdF);
void work_space_set (void);
void main_work_space (void);
