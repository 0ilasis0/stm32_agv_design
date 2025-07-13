#include "adc/main.h"
#include "adc.h"
#include "motor/main.h"
#include "main/config.h"

static uint16_t ADC_Values[ADC_COUNT * ADC_NEED_LEN] = {0}; // adc儲存位置

AdcHall adchall_track_left = {
    .const_h = {
        // PB11(R18)
        .id = 2,
        .magnetic_value = 2360, // 1910
    },
    .min = 4095,
};

AdcHall adchall_track_right = {
    .const_h = {
        // PB1(R24)
        .id = 1,
        .magnetic_value = 2350, // 1880
    },
    .min = 4095,
};

AdcHall adchall_direction = {
    .const_h = {
        // PB12(R16)
        .id = 0,
        .magnetic_value = 2250, // 1835
    },
    .min = 4095,
};

AdcHall adchall_node = {
    .const_h = {
        // PB0(L34)
        .id = 3,
        .magnetic_value = 1850,
    },
    .min = 4095,
};

// 2100 - 1100
static uint16_t adc_cnt[2560] = {0};

static void max_min (AdcHall* adc)
{
    if(adc->value > adc->max) adc->max = adc->value;
    else if(adc->value < adc->min && adc->value > 1000) adc->min = adc->value;
}

static void hall_update(AdcHall* adc)
{
    memset(adc_cnt, 0, sizeof(adc_cnt));
    uint16_t i, k;
    for (i = 0; i < ADC_NEED_LEN; i++)
    {
        k = ADC_Values[i * ADC_COUNT + adc->const_h.id];
        if(k > 2559) continue;
        adc_cnt[k]++;
    }
    const uint16_t target = (ADC_NEED_LEN-1)/2;
    uint16_t acc = 0;
    for (i = 500; i < 2500; i++)
    {
        acc += adc_cnt[i];
        if (acc > target)
        {
            adc->value = i;
            break;
        }
    }
    max_min(adc);
}

void StartAdcTask(void *argument)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, ADC_COUNT * ADC_NEED_LEN);

    for(;;)
    {
        if (
            !runtime_switch.adc
            || HAL_GetTick() < 3000
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
