#include "map_new/main.h"
#include "map_new/stack.h"
#include "map_new/base.h"
#include "map_new/manager.h"
#include "map_new/init.h"
#include "map_new/work_space.h"
#include "rfid/main.h"



static uint32_t def_time;



int tick_time = 0;
void StartDefaultTask(void *argument)
{
    // osThreadExit();
    // return;

    memcpy(locations_t, locations_t_inner, sizeof(locations_t));
    map_set();
    work_space_set();

    // text
    // map_data_renew_direction_and_address(&map_data_start, 5, 5);
    work_space_main();
    // text

    for(;;)
    {

        // text
        tick_time++;
        def_time = HAL_GetTick() - time_start;
        text_a[0] = map_data_all.current_count;
        // text

        // map flag
        if (new_card == 1 && (def_time >= TIME_INIT) && !map_toggle)
        {
            time_start = HAL_GetTick();
            map_toggle = true;
        }
        if (spi2_rfid.state == CARD_STATE_NONE && map_toggle) map_toggle = false;

        if(map_enable && map_toggle)
        {
            if (spi2_rfid.uid32 == map_data_all.map_data[map_data_all.current_count + 1].address_id)
            {
                map_data_all.current_count ++;
                agv_state = map_data_all.map_data[map_data_all.current_count];

                text_a[1] = map_data_all.current_count;

                if (agv_state.mode == VEHICLE_MODE_END)
                {
                    map_data_renew_direction_and_address(
                        &map_data_start,
                        map_data_all.map_data[final_node_count].address_id,
                        map_data_all.map_data[final_node_count - 1].direction
                        );
                    map_enable = false;

                    // text
                    work_space_main();
                    // text
                }

                map_trans(&agv_state);
            }
            // 如果循跡應該往前結果讀到原本的rfid而非下一個rfid，代表遇上障礙，進行地圖重製
            // else if (spi2_rfid.uid32 == map_data_all.map_data[map_data_all.current_count].address_id)
            // {

            //     // 先傳送停止動作，等地圖計算完畢
            //     agv_state = map_data_init;
            //     map_trans(&agv_state);

            //     MapData map_data_temp = map_data_all.map_data[map_data_all.current_count];
            //     MapIdF  target_id     = map_data_all.map_data[final_node_count].address_id;

            //     delete_locations_t_data(
            //         map_data_all.map_data[map_data_all.current_count].address_id,
            //         map_data_all.map_data[map_data_all.current_count].direction
            //         );
            //     delete_locations_t_data(
            //         map_data_all.map_data[map_data_all.current_count + 1].address_id,
            //         opposite_direction(map_data_all.map_data[map_data_all.current_count].direction)
            //         );

            //     map_set();
            //     map_data_renew_direction_and_address(
            //         &map_data_start,
            //         map_data_temp.address_id,
            //         map_data_temp.direction
            //         );
            //     map_bulid(map_data_temp.address_id, target_id);
            //     map_trans(&agv_state);
            // }
            // 只知道現在位置不知道方向，所以停止動作，目前測試所以槓掉
            // else
            // {
            //     ERROR_STOP_MAP_RETURN(map_error.lose_navigation, FNS_FAIL);
            // }
        }

        osDelay(10);
    }
}

