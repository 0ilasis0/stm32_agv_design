#include "main/adc.h"
#include "adc.h"
#include "motor/main.h"
#include "main/config.h"

AdcHall adc_hall;

static uint16_t ADC_Values[ADC_CAP] = {0};                                 // adc儲存位置

// PB12(R16)     PB1(R24)     PB11(R18)      PB0(L34)
/* +setup -----------------------------------------------------------*/
void adc_setup (void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, ADC_CAP);
    adc_hall = adc_hall_init();
}

AdcHall adc_hall_init (void)
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

uint16_t text_max = 0;
uint16_t text_min = 0;

void text_cal (uint16_t temp)
{
    if(temp > text_max) text_max = temp;
    if(temp < text_min) text_min = temp;
}

// renew adc senser
void adc_renew (void)
{
    if (!sys_run_switch.enable_adc) return;

    uint32_t sum[4] = {0};
    for(int i = 0; i < ADC_CAP; i += 4)
    {
        sum[0] += ADC_Values[i];
        sum[1] += ADC_Values[i+1];
        sum[2] += ADC_Values[i+2];
        sum[3] += ADC_Values[i+3];
    }

    adc_hall.sensor_track_right = sum[0] / 5;
    adc_hall.sensor_track_left = sum[1] / 5;
    adc_hall.sensor_node = sum[2] / 5;
    adc_hall.sensor_direction = sum[3] / 5;

    text_cal(adc_hall.sensor_track_right);
    text_cal(adc_hall.sensor_track_left);
    text_cal(adc_hall.sensor_node);
    text_cal(adc_hall.sensor_direction);

}
