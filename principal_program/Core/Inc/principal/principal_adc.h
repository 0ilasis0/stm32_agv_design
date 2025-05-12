#ifndef PRINCIPAL_ADC_H
#define PRINCIPAL_ADC_H

#include "stm32g4xx_hal_adc.h"

extern ADC_HandleTypeDef hadc1;

void adc_renew(void);
void hall_detection_adc_setup(void);

#endif
