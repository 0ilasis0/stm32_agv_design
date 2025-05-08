#include "principal/principal_adc.h"

uint16_t ADC_Values[10] = {0};                                 // adc儲存位置



/* +setup -----------------------------------------------------------*/
void hall_detection_adc_setup(void) {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, 10);
}



void adc_renew(void) {                                          // renew adc senser
    motor_right.adc_value = 0;
    motor_left.adc_value = 0;
    for(int i = 0; i < 10; i+=2) {
        motor_right.adc_value += ADC_Values[i];
        motor_left.adc_value += ADC_Values[i+1];
    }
    motor_right.adc_value /= 2;
    motor_left.adc_value /= 2;
}
