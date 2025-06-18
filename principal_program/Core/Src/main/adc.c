#include "main/adc.h"
#include <stdint.h>
#include "adc.h"
#include "motor/main.h"

static uint16_t ADC_Values[10] = {0};                                 // adc儲存位置

// PB12 R16    PB1 R24
/* +setup -----------------------------------------------------------*/
void adc_setup(void) {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, 10);
}

// renew adc senser
void adc_renew(void) {
    int i;
    uint32_t sum_r = 0, sum_l = 0;
    for(i = 0; i < 10; i += 2) {
        sum_r += ADC_Values[i];
        sum_l += ADC_Values[i+1];
    }

    motor_set_adc_val(&motor_right, sum_r / 5);
    motor_set_adc_val(&motor_left, sum_l / 5);
}
