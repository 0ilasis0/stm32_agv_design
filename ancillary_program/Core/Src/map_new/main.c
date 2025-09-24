#include "rfid/main.h"
#include "map_new/base.h"
#include "map_new/init.h"
#include "map_new/manager.h"
#include "map_new/work_space.h"
#include "map_new/window.h"



static uint32_t tick_time = 0;
static uint32_t now;



void StartDefaultTask(void *argument)
{
    // osThreadExit();
    // return;

    memcpy(locations_t, locations_t_inner, sizeof(locations_t));
    work_space_set();
    map_set();

    uint32_t last_edge_time = HAL_GetTick();

    // text
    map_window_init_work(locations_t_inner[0].local_id, 0);

    map_window_add_work(locations_t_inner[1].local_id);
    map_window_add_work(locations_t_inner[2].local_id);
    map_window_add_work(locations_t_inner[1].local_id);

    map_window_start_work();
    // text

    for(;;)
    {
        tick_time++;
        now = HAL_GetTick();

        // map flag
        if (new_card == 1 && (now - last_edge_time) >= TIME_INIT && !map_toggle)
        {
            last_edge_time = now;
            map_toggle = true;
        }
        // if (spi2_rfid.state == CARD_STATE_NONE && map_toggle && map_enable) map_toggle = false;

        if(map_enable && map_toggle)
        {
            if (spi2_rfid.uid32 == map_data_all.map_data[map_data_all.current_count + 1].address_id)
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

                    main_work_space();
                }

                map_trans(&agv_state);
            }
            // 如果循跡應該往前結果讀到原本的rfid而非下一個rfid，代表遇上障礙，進行地圖重製
            // else if (spi2_rfid.uid32 == map_data_all.map_data[map_data_all.current_count].address_id)
            // {

            //     // 先傳送停止動作，等地圖計算完畢
            //     main_work_space();

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

                // if (!(map_bulid(map_data_temp.address_id, target_id);))
                // {
                //     enforce_stop();
                    // continue;
                // }

            //     map_trans(&agv_state);
            // }
            // 只知道現在位置不知道方向，所以停止動作，目前測試所以槓掉
            // else
            // {
                // enforce_stop();
                // continue;
            // }
        }

        // map flag
        if (spi2_rfid.state == CARD_STATE_NONE && map_toggle) map_toggle = false;

        osDelay(10);
    }
}

