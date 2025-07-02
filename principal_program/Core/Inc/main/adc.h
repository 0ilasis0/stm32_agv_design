#pragma once

#include <stdint.h>

typedef struct
{
    uint16_t sensor_track_right;
    uint16_t sensor_track_left;
    uint16_t sensor_node;
    uint16_t sensor_direction;
    uint16_t magnetic_stripe_value;         // 判斷磁條強度大小
    uint16_t strong_magnet_value;           // 判斷強力磁鐵強度大小
} AdcHall;

extern AdcHall adc_hall;

void adc_setup(void);
AdcHall adc_hall_init (void);
void adc_renew(void);
