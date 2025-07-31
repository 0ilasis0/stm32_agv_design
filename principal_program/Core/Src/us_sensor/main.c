#include "us_sensor/main.h"
#include "vehicle/basic.h"
#include "tim.h"

USSensor us_sensor_head = {
    .const_h = {
        .trig_GPIOx = GPIOC,
        .trig_GPIO_Pin_x = GPIO_PIN_5,
        .echo_GPIOx = GPIOC,
        .echo_GPIO_Pin_x = GPIO_PIN_6,
        .warning = 400.0f,
        .danger = 200.0f,
    },
    .state = USS_STATE_STOP,
};

Result us_sensor_enable(USSensor* self)
{
    if (self->state != USS_STATE_STOP) return RESULT_ERROR(RES_ERR_BUSY);
    self->state = USS_STATE_WAITING;
    return RESULT_OK(self);
}

static Result uss_start_inner(USSensor* self)
{
    if (self->state != USS_STATE_WAITING) return RESULT_ERROR(RES_ERR_INVALID);
    self->state = USS_STATE_TRIGGER;
    HAL_GPIO_WritePin(self->const_h.trig_GPIOx, self->const_h.trig_GPIO_Pin_x, GPIO_PIN_SET);
    return RESULT_OK(self);
}

static Result uss_tri_off_inner(USSensor* self)
{
    if (self->state != USS_STATE_TRIGGER) return RESULT_ERROR(RES_ERR_INVALID);
    self->state = USS_STATE_RUNNING;
    HAL_GPIO_WritePin(self->const_h.trig_GPIOx, self->const_h.trig_GPIO_Pin_x, GPIO_PIN_RESET);
    return RESULT_OK(self);
}

static Result uss_overflow_inner(USSensor* self)
{
    if (self->state != USS_STATE_RUNNING) return RESULT_ERROR(RES_ERR_INVALID);
    self->state = USS_STATE_STOP;
    HAL_GPIO_WritePin(self->const_h.trig_GPIOx, self->const_h.trig_GPIO_Pin_x, GPIO_PIN_RESET);
    self->time = UINT16_MAX;
    self->distance = FLT_MAX;
    return RESULT_OK(self);
}

void us_sensor_start(void)
{
    uss_start_inner(&us_sensor_head);
}

void us_sensor_tri_off(void)
{
    uss_tri_off_inner(&us_sensor_head);
}

void us_sensor_overflow(void)
{
    uss_overflow_inner(&us_sensor_head);
}

Result us_sensor_stop(USSensor* self)
{
    if (self->state != USS_STATE_RUNNING) return RESULT_ERROR(RES_ERR_INVALID);
    self->state = USS_STATE_STOP;
    const USSConst *const_h = &self->const_h;
    HAL_GPIO_WritePin(const_h->trig_GPIOx, const_h->trig_GPIO_Pin_x, GPIO_PIN_RESET);
    self->time = __HAL_TIM_GET_COUNTER(US_SENSOR_HTIM);
    self->distance = (float)self->time * 0.343f / 2.0f;  // uint:mm
    if      (self->distance <= const_h->danger ) self->status = USS_STATUS_DANGER;
    else if (self->distance <= const_h->warning) self->status = USS_STATUS_WARNING;
    else self->status = USS_STATUS_SAVE;
    return RESULT_OK(self);
}
