#include "rfid/main.h"
#include "main/config.h"
#include "map_new/base.h"
#include "map_new/init.h"
#include "map_new/manager.h"
#include "map_new/work_space.h"
#include "map_new/window.h"
#include "adc.h"


static uint32_t tick_time = 0;
static uint32_t now;
static bool map_toggle = false;
uint32_t text_id[7] = {};
uint16_t adc_raw = 0;
size_t defalt_running = 0;
void StartDefaultTask(void *argument)
{
    // osThreadExit();
    // return;

    memcpy(locations_t, locations_t_inner, sizeof(locations_t));
    work_space_set();
    map_set();

    // text
    map_window_init_work(locations_t[1].local_id, 4);
    // map_window_add_work(locations_t[3].local_id);
    map_window_add_work(locations_t[0].local_id);
    map_window_add_work(locations_t[1].local_id);
    map_window_add_work(locations_t[0].local_id);
    map_window_add_work(locations_t[1].local_id);
    map_window_add_work(locations_t[0].local_id);
    map_window_add_work(locations_t[1].local_id);
    // map_window_add_work(locations_t[2].local_id);

    // map_window_init_work(locations_t[3].local_id, 0);
    // map_window_add_work(locations_t[1].local_id);

    map_window_start_work();

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADC_Start(&hadc1);
    adc_raw = 1;

    // text

    for(;;)
    {
        tick_time++;
        now = HAL_GetTick();
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            defalt_running++;
            adc_raw = HAL_ADC_GetValue(&hadc1);
            // 每完成一筆，就會進來一次
        }

        // map toggle
        if (new_card == 1 && !map_toggle)
        {
            new_card = 0;
            map_toggle = true;
        }

        if(map_enable && map_toggle)
        {
            dbg.map_main_count++;

            if (spi2_rfid.uid32 == map_data_all.map_node[map_data_all.current_count + 1].address_id)
            {
                map_data_all.current_count ++;
                agv_state = map_data_all.map_node[map_data_all.current_count];

                if (agv_state.mode == VEHICLE_MODE_END)
                {
                    map_data_renew_direction_and_address(
                        &map_data_start,
                        map_data_all.map_node[final_node_count].address_id,
                        map_data_all.map_node[final_node_count].direction
                        );

                    main_work_space(NO_DATA, NO_DATA);
                }

                map_trans(&agv_state);
            }
            // 如果循跡應該往前結果讀到原本的rfid而非下一個rfid，代表遇上障礙，進行地圖重製
            else if (spi2_rfid.uid32 == map_data_all.map_node[map_data_all.current_count].address_id)
            {
                dbg.log[7]++;

                MapIdF  map_data_temp_id    = map_data_all.map_node[map_data_all.current_count].address_id;
                MapDirF map_data_temp_dir   = map_data_all.map_node[map_data_all.current_count + 1].direction;
                MapIdF  target_id           = map_data_all.map_node[final_node_count].address_id;

                text_id[0] = map_data_all.map_node[map_data_all.current_count].address_id;
                text_id[1] = map_data_all.map_node[map_data_all.current_count + 1].direction;
                text_id[2] = map_data_all.map_node[map_data_all.current_count + 1].address_id;
                text_id[3] = opposite_direction(map_data_all.map_node[map_data_all.current_count + 1].direction);
                text_id[4] = map_data_all.current_count;
                text_id[5] = map_data_all.map_node[0].address_id;
                text_id[6] = map_data_all.map_node[1].address_id;

                delete_locations_t_data(
                    map_data_all.map_node[map_data_all.current_count].address_id,
                    map_data_all.map_node[map_data_all.current_count + 1].direction
                    );
                delete_locations_t_data(
                    map_data_all.map_node[map_data_all.current_count + 1].address_id,
                    opposite_direction(map_data_all.map_node[map_data_all.current_count + 1].direction)
                    );

                map_set();
                map_data_renew_direction_and_address(
                    &map_data_start,
                    map_data_temp_id,
                    map_data_temp_dir
                    );

                main_work_space(map_data_temp_id, target_id);

                map_trans(&agv_state);
            }
            // 只知道現在位置不知道方向，所以停止動作，目前測試所以槓掉
            // else
            // {
            //     enforce_stop();
            //     continue;
            // }
        }

        // map flag
        if (map_toggle) map_toggle = false;

        osDelay(10);
    }
}

