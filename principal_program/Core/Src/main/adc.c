#include "main/adc.h"
#include "adc.h"
#include "motor/main.h"
#include "main/config.h"

AdcHall adc_hall;

static uint16_t ADC_Values[ADC_COUNT * ADC_NEED_LEN] = {0};                                 // adc儲存位置

// PB12(R16)     PB1(R24)     PB11(R18)      PB0(L34)
static AdcHall adc_hall_init (void)
{
    AdcHall adc_hall_new;
    adc_hall_new.sensor_track_right     = 0;
    adc_hall_new.sensor_track_left      = 0;
    adc_hall_new.sensor_node            = 0;
    adc_hall_new.sensor_direction       = 0;
    adc_hall_new.magnetic_stripe_value  = ADC_MAGNETIC_STRIPE_VALUE;
    adc_hall_new.strong_magnet_value    = ADC_STRONG_MAGNET_VALUE;
    return adc_hall_new;
}

/* +setup -----------------------------------------------------------*/
void adc_setup (void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, ADC_COUNT * ADC_NEED_LEN);
    adc_hall = adc_hall_init();
}

uint16_t text_max[4] = {0};
uint16_t text_min[4] = {2000, 2000, 2000, 2000};

void text_cal (uint16_t temp, uint16_t *max, uint16_t *min)
{
    if(temp > *max) *max = temp;
    if(temp < *min && temp > 1000) *min = temp;
}

#define SWAP(a,b)       \
do {                    \
    uint16_t temp = a;  \
    a = b;              \
    b = temp;           \
} while(0)

static void quick_sort(uint16_t* arr, int left, int right) {
    if (left >= right) return;
    uint16_t pivot = arr[right];
    int i = left - 1;
    for (int j = left; j < right; j++) {
        if (arr[j] <= pivot) {
            i++;
            SWAP(arr[i], arr[j]);
        }
    }
    SWAP(arr[i + 1], arr[right]);
    quick_sort(arr, left, i);
    quick_sort(arr, i + 2, right);
}

static void sort(uint16_t* values, size_t count) {
    if (count > 1) {
        quick_sort(values, 0, count - 1);
    }
}

// renew adc senser
void adc_renew (void)
{
    if (!runtime_switch.adc) return;

    // uint32_t sum[4] = {0};
    uint16_t adc_1[ADC_NEED_LEN], adc_2[ADC_NEED_LEN], adc_3[ADC_NEED_LEN], adc_4[ADC_NEED_LEN];

    for(int i = 0; i < ADC_NEED_LEN; i++)
    {
        adc_1[i] = ADC_Values[i*ADC_COUNT];
        adc_2[i] = ADC_Values[i*ADC_COUNT+1];
        adc_3[i] = ADC_Values[i*ADC_COUNT+2];
        adc_4[i] = ADC_Values[i*ADC_COUNT+3];
    }

    sort(adc_1, ADC_NEED_LEN);
    sort(adc_2, ADC_NEED_LEN);
    sort(adc_3, ADC_NEED_LEN);
    sort(adc_4, ADC_NEED_LEN);

    adc_hall.sensor_track_right = adc_1[ADC_NEED_LEN/2];
    adc_hall.sensor_track_left = adc_2[ADC_NEED_LEN/2];
    adc_hall.sensor_node = adc_3[ADC_NEED_LEN/2];
    adc_hall.sensor_direction = adc_4[ADC_NEED_LEN/2];

    if (HAL_GetTick() < 5000) return;
    text_cal(adc_hall.sensor_track_right,   &text_max[0], &text_min[0]);
    text_cal(adc_hall.sensor_track_left,    &text_max[1], &text_min[1]);
    text_cal(adc_hall.sensor_node,          &text_max[2], &text_min[2]);
    text_cal(adc_hall.sensor_direction,     &text_max[3], &text_min[3]);
}
