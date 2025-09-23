#include "map_new/main.h"
#include "map_new/base.h"
#include "map_new/manager.h"
#include "map_new/init.h"
#include "rfid/main.h"



void map_windows (MapIdF from, MapIdF to)
{
    // 若已設定起點且與本次輸入不符，視為錯誤
    if (map_data_start.address_id != from && map_data_start.address_id != NO_DATA)
    {
        map_error.input_start_id_err = RESULT_ERROR(RES_ERR_NOT_FOUND);
    }

    map_enable = true;
    map_bulid(from, to);
    // 如果地圖建構失敗
    if (map_enable == false)
    {
        return;
    }
    map_trans(&agv_state);
    map_data_start = map_data_all.map_data[0];
    time_start = HAL_GetTick();
}

int yy = 1;
int tick_time = 0;
uint32_t def_time;
void StartDefaultTask(void *argument)
{
    // osThreadExit();
    // return;

    memcpy(locations_t, locations_t_inner, sizeof(locations_t));
    map_set();

    // text
    // map_data_renew_direction_and_address(&map_data_start, 5, 5);
    map_windows(locations_t_inner[0].local_id, locations_t_inner[3].local_id);

    // text

    for(;;)
    {

        // text
        tick_time++;
        def_time = HAL_GetTick() - time_start;
        // text

        // map flag
        if (new_card == 1 && (def_time >= TIME_INIT) && !map_toggle)
        {
            time_start = HAL_GetTick();
            map_toggle = true;
        } 
        if (spi2_rfid.state == CARD_STATE_NONE && map_toggle) map_toggle = false;

        // if(map_enable)
        if(map_enable && map_toggle)
        {
            // 讀到RFID執行給資料到另一個stm32
            // if (spi2_rfid.uid32 == map_data_all.map_data[map_data_all.current_count + 1].address_id || tick_time % 100 == 0)

            // text
            textabc[0] = spi2_rfid.uid32;
            textabc[1] = map_data_all.map_data[map_data_all.current_count + 1].address_id;
            // text

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
                    map_enable = false;

                    // text
                    // map_windows(14, 5);
                    // text
                }

                map_trans(&agv_state);
            }
            // 如果循跡應該往前結果讀到原本的rfid而非下一個rfid，代表遇上障礙，進行地圖重製
            // else if (spi2_rfid.uid32 == map_data_all.map_data[map_data_all.current_count].address_id || (tick_time % 330 == 0 && yy))
            else if (spi2_rfid.uid32 == map_data_all.map_data[map_data_all.current_count].address_id)
            {
                // text
                // yy = 0;
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

