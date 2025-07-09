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

typedef struct {
    float x_est;  // 狀態估計值
    float P;      // 誤差協方差
    float Q;      // 過程噪聲
    float R;      // 量測噪聲
} KalmanFilter;

extern AdcHall adc_hall;

void adc_setup(void);
void adc_renew(void);
