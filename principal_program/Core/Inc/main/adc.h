#pragma once

#include <stdint.h>

extern uint32_t hall_magnetic_stripe_value;
extern uint32_t hall_strong_magnet_value;
extern uint32_t hall_sensor_direction;
extern uint32_t hall_sensor_node;

void adc_renew(void);
void adc_setup(void);
