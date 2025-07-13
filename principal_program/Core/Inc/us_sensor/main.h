#pragma once

#include "main/config.h"
#include "main/fn_state.h"

typedef struct USSConst
{
    GPIO_TypeDef* trig_GPIOx;
    uint16_t trig_GPIO_Pin_x;
    GPIO_TypeDef* echo_GPIOx;
    uint16_t echo_GPIO_Pin_x;
} USSConst;

typedef uint8_t USSState;
#define USSS_STOP    0
#define USSS_RUNNING 1
#define USSS_TRIGGER 2
#define USSS_WAITING 3

typedef struct USSensor
{
    const USSConst const_h;
    uint8_t state;
    uint16_t time;
    float distance;
} USSensor;

extern USSensor us_sensor_head;

FnState us_sensor_enable(USSensor* us_sensor);
void us_sensor_start(void);
void us_sensor_tri_off(void);
void us_sensor_overflow(void);
FnState us_sensor_stop(USSensor* us_sensor);
void us_sensor_main(void);
