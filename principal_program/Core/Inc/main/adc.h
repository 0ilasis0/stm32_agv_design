#pragma once

#include <stdint.h>

typedef struct
{
    uint32_t sensor_track_right;
    uint32_t sensor_track_left;
    uint32_t sensor_node;
    uint32_t sensor_direction;
    uint32_t magnetic_stripe_value;         // 判斷磁條強度大小
    uint32_t strong_magnet_value;           // 判斷強力磁鐵強度大小
} AdcHall;

extern AdcHall adc_hall;

void adc_setup(void);
AdcHall adc_hall_init (void);
void adc_renew(void);
