#pragma once

#include "main/config.h"
#include "main/fn_state.h"

typedef struct USSConst
{
    GPIO_TypeDef* trig_GPIOx;
    uint16_t trig_GPIO_Pin_x;
    GPIO_TypeDef* echo_GPIOx;
    uint16_t echo_GPIO_Pin_x;
    float warning;
    float danger;
} USSConst;

typedef enum USSState
{
    USS_STATE_STOP,
    USS_STATE_RUNNING,
    USS_STATE_TRIGGER,
    USS_STATE_WAITING,
} USSState;

typedef enum USSStatus
{
    USS_STATUS_SAVE,
    USS_STATUS_WARNING,
    USS_STATUS_DANGER,
} USSStatus;

typedef struct USSensor
{
    const USSConst const_h;
    USSState state;
    uint32_t time;
    float distance;
    USSStatus status;
} USSensor;

extern USSensor us_sensor_head;

FnState us_sensor_enable(USSensor* us_sensor);
void us_sensor_start(void);
void us_sensor_tri_off(void);
void us_sensor_overflow(void);
FnState us_sensor_stop(USSensor* us_sensor);
