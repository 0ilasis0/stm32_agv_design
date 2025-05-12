#include "principal/principal_adc.h"
#include <stdint.h>
#include "principal/motor.h"
#include "adc.h"

uint16_t ADC_Values[10] = {0};                                 // adc儲存位置

/* +setup -----------------------------------------------------------*/
void hall_detection_adc_setup(void) {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, 10);
}

// renew adc senser
void adc_renew(void) {
    uint32_t sum_r = 0, sum_l = 0;
    for(int i = 0; i < 10; i+=2) {
        sum_r += ADC_Values[i];
        sum_l += ADC_Values[i+1];
    }
    motor_right.adc_value = sum_r / 5;
    motor_left.adc_value  = sum_l / 5;
}
