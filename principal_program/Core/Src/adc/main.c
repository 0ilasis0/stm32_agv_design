#include "adc/main.h"
#include "adc.h"
#include "motor/main.h"
#include "main/config.h"

static uint16_t ADC_Values[ADC_COUNT * ADC_NEED_LEN] = {0}; // adc儲存位置

AdcHall adchall_track_left = {
    .const_h = {
        // PB12(R16)
        .id = 0,
        .magnetic_value = 1860,
    },
    .min = 4095,
};

AdcHall adchall_track_right = {
    .const_h = {
        // PB11(R18)
        .id = 2,
        .magnetic_value = 1830,
    },
    .min = 4095,
};

AdcHall adchall_direction = {
    .const_h = {
        // PB1(R24)
        .id = 1,
        .magnetic_value = 1800,
    },
    .min = 4095,
};

AdcHall adchall_node = {
    .const_h = {
        // PB0(L34)
        .id = 3,
        .magnetic_value = 1800,
    },
    .min = 4095,
};

static uint16_t adc_cnt[4096] = {0};

static void max_min (AdcHall* adc)
{
    if(adc->value > adc->max) adc->max = adc->value;
    else if(adc->value < adc->min && adc->value > 1000) adc->min = adc->value;
}

static void hall_update(AdcHall* adc)
{
    memset(adc_cnt, 0, sizeof(adc_cnt));
    uint16_t i;
    for (i = 0; i < ADC_NEED_LEN; i++)
    {
        adc_cnt[ADC_Values[i * ADC_COUNT + adc->const_h.id]]++;
    }
    const uint8_t target = (ADC_NEED_LEN-1)/2;
    uint16_t acc = 0;
    for (i = 0; i < 4096; i++)
    {
        acc += adc_cnt[i];
        if (acc > target)
        {
            adc->value = i;
            break;
        }
    }
    if (HAL_GetTick() < 3000) return;
    max_min(adc);
}

void StartAdcTask(void *argument)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, ADC_COUNT * ADC_NEED_LEN);

    for(;;)
    {
        if (!runtime_switch.adc)
        {
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
