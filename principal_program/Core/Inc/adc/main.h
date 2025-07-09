#pragma once

#include "main/config.h"

typedef struct AdcHallConst
{
    uint8_t id;
    uint16_t magnetic_value;
} AdcHallConst;

typedef struct AdcHall
{
    const AdcHallConst const_h;
    uint16_t value;
    uint16_t max;
    uint16_t min;
} AdcHall;

extern AdcHall adchall_track_left;
extern AdcHall adchall_track_right;
extern AdcHall adchall_node;
extern AdcHall adchall_direction;
