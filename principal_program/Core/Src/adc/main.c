#include "adc/main.h"
#include "adc.h"
#include "motor/main.h"
#include "main/config.h"

static uint16_t ADC_Values[ADC_COUNT * ADC_NEED_LEN] = {0}; // adc儲存位置

AdcHall adchall_track_left = {
    .const_h = {
        // PB11(R18)
        .id = 2,
    },
    .gate = 1860,
    .min = 4095,
};

AdcHall adchall_track_right = {
    .const_h = {
        // PB1(R24)
        .id = 1,
    },
    .gate = 1850,
    .min = 4095,
};

AdcHall adchall_direction = {
    .const_h = {
        // PB12(R16)
        .id = 0,
    },
    .gate = 1780,
    .min = 4095,
};

AdcHall adchall_node = {
    .const_h = {
        // PB0(L34)
        .id = 3,
    },
    .gate = 1850,
    .min = 4095,
};

// 2100 - 1100
static uint16_t adc_cnt[ADC_ACC_CNT_MAX - ADC_ACC_CNT_MIN] = {0};

int count = 0;
static void max_min (AdcHall* adc)
{
    count ++ ;
    if (count % 300 == 0)
    {
        adc->max = 0;
        adc->min = 4095;
    }

    if(adc->value > adc->max) adc->max = adc->value;
    else if(adc->value < adc->min && adc->value > 1000) adc->min = adc->value;
}

static void hall_update(AdcHall* adc)
{
    memset(adc_cnt, 0, sizeof(adc_cnt));
    uint16_t i, value;
    for (i = 0; i < ADC_NEED_LEN; i++)
    {
        value = ADC_Values[i * ADC_COUNT + adc->const_h.id];
        if(value < ADC_ACC_CNT_MIN || value >= ADC_ACC_CNT_MAX) continue;
        adc_cnt[value - ADC_ACC_CNT_MIN]++;
    }
    uint16_t acc = 0;
    for (i = 0; i < ADC_ACC_CNT_MAX - ADC_ACC_CNT_MIN; i++)
    {
        acc += adc_cnt[i];
        if (acc > ADC_NEED_LEN / 2)
        {
            adc->value = i + ADC_ACC_CNT_MIN;
            break;
        }
    }
    // ! IMPORTANT
    if (adc->value <= adc->gate) adc->state = ADC_HALL_STATE_ON_MAG;
    else adc->state = ADC_HALL_STATE_NONE;
    max_min(adc);
}

void StartAdcTask(void *argument)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, ADC_COUNT * ADC_NEED_LEN);

    for(;;)
    {
        if (
            !runtime_switch.adc
            || HAL_GetTick() < 500
        ) {
            osDelay(50);
            continue;
        }
        hall_update(&adchall_track_left);
        hall_update(&adchall_track_right);
        hall_update(&adchall_node);
        hall_update(&adchall_direction);
        osDelay(50);
    }
}
