#include "main/adc.h"
#include "adc.h"
#include "motor/main.h"
#include "main/config.h"

// 判斷磁條強度大小
uint32_t hall_magnetic_stripe_value = ADC_HALL_MAGNITUTE_EDGE;
// 判斷強力磁鐵強度大小
uint32_t hall_strong_magnet_value = ADC_HALL_MAGNITUTE_EDGE;

// 霍爾實際值
uint32_t hall_sensor_node = ADC_HALL_MAGNITUTE_EDGE + 1;
uint32_t hall_sensor_direction = 0;


static uint16_t ADC_Values[ADC_CAP] = {0};                                 // adc儲存位置

// PB12(R16)     PB1(R24)     PB11(R18)      PB0(L34I)
/* +setup -----------------------------------------------------------*/
void adc_setup(void) {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, ADC_CAP);
}

// renew adc senser
void adc_renew(void) {
    if (!sys_run_switch.enable_adc) return;

    uint32_t sum[4] = {0};
    for(int i = 0; i < ADC_CAP; i += 4) {
        sum[0] += ADC_Values[i];
        sum[1] += ADC_Values[i+1];
        sum[2] += ADC_Values[i+2];
        sum[3] += ADC_Values[i+3];
    }

    motor_set_adc_val(&motor_right, sum[0] / 5);
    motor_set_adc_val(&motor_left, sum[1] / 5);
    hall_sensor_node = sum[2] / 5;
    hall_sensor_direction = sum[3] / 5;
}
