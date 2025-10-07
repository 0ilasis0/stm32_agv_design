#include "map_new/adjust_start.h"
#include "map_new/base.h"



/**
  * @brief 偵測是否有初始方向數據，如果存在，則執行原地旋轉修正以對準起始航向
  */
void map_adjust_start (void)
{
    if (map_data_start.direction == NO_DATA) return;

    map_data_start.vehicle_motion = decide_map_vehicle_motion(
        map_data_start.direction,
        map_data_all.map_node[0].direction
        );

    map_data_start.need_rotate_count = decide_need_rotate_count(
        map_data_start.vehicle_motion,
        map_data_all.map_node[0].address_id,
        map_data_start.direction,
        map_data_all.map_node[0].direction
        );

    if (map_data_start.direction != map_data_all.map_node[0].direction)
    {
        map_data_all.map_node[0] = map_data_start;
    }
}
