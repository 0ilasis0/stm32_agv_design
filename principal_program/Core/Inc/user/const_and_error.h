#ifndef USER_CONST_H
#define USER_CONST_H

#include <stdbool.h>
#include <stdint.h>

#define error_timeout_time_limit 10 * 1500

extern bool PI_enable;

typedef struct {
    bool ensure_motor_stop;
    bool test_no_load_speed;
    bool over_hall_fall_back_time_based;
    bool over_hall_fall_back;
    bool rotate_in_place;
    bool rotate_in_place_hall;
} ERROR_TIMEOUT;

extern ERROR_TIMEOUT error_timeout;

void timeout_error (uint32_t error_start, bool *error_parameter);

#endif
