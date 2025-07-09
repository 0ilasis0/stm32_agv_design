#include "main/adc.h"
#include "adc.h"
#include "motor/main.h"
#include "main/config.h"

// PB12(R16)     PB1(R24)     PB11(R18)      PB0(L34)

#define SWAP(a,b)       \
do {                    \
    uint16_t temp = a;  \
    a = b;              \
    b = temp;           \
} while(0)

static uint16_t ADC_Values[ADC_COUNT * ADC_NEED_LEN] = {0};                                 // adc儲存位置

AdcHall adchall_track_left = {
    .const_h = {
        .id = 0,
        .magnetic_value = 1860,
    },
    .min = 4095,
};

AdcHall adchall_track_right = {
    .const_h = {
        .id = 1,
        .magnetic_value = 1830,
    },
    .min = 4095,
};

AdcHall adchall_node = {
    .const_h = {
        .id = 2,
        .magnetic_value = 1800,
    },
    .min = 4095,
};

AdcHall adchall_direction = {
    .const_h = {
        .id = 3,
        .magnetic_value = 1800,
    },
    .min = 4095,
};

static void max_min (AdcHall* adc)
{
    if(adc->value > adc->max) adc->max = adc->value;
    else if(adc->value < adc->min && adc->value > 1000) adc->min = adc->value;
}

void adc_setup (void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, ADC_COUNT * ADC_NEED_LEN);
}

// static void classify(void)
// {
//     uint16_t i;
//     for(i = 0; i < ADC_NEED_LEN; i++)
//     {
//         adchall_track_left.data[i]   = ADC_Values[i * ADC_COUNT + adchall_track_left.const_h];
//         adchall_track_right.data[i] = ADC_Values[i * ADC_COUNT + adchall_track_right.const_h];
//         adchall_node.data[i]         = ADC_Values[i * ADC_COUNT + adchall_node.const_h];
//         adchall_direction.data[i]    = ADC_Values[i * ADC_COUNT + adchall_direction.const_h];
//     }
// }

uint16_t adc_cnt[4096] = {0};
static void get_middle_value(AdcHall* adc)
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

// renew adc senser
void adc_renew (void)
{
    if (!runtime_switch.adc) return;
    get_middle_value(&adchall_track_left);
    get_middle_value(&adchall_track_right);
    get_middle_value(&adchall_node);
    get_middle_value(&adchall_direction);
}


