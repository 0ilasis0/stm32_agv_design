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

Result us_sensor_enable(USSensor* us_sensor)
{
    if (us_sensor->state != USS_STATE_STOP) return RESULT_ERROR(RES_ERR_BUSY);
    us_sensor->state = USS_STATE_WAITING;
    return RESULT_OK(NULL);
}

static Result uss_start_inner(USSensor* us_sensor)
{
    if (us_sensor->state != USS_STATE_WAITING) return RESULT_ERROR(RES_ERR_INVALID);
    us_sensor->state = USS_STATE_TRIGGER;
    HAL_GPIO_WritePin(us_sensor->const_h.trig_GPIOx, us_sensor->const_h.trig_GPIO_Pin_x, GPIO_PIN_SET);
    return RESULT_OK(NULL);
}

static Result uss_tri_off_inner(USSensor* us_sensor)
{
    if (us_sensor->state != USS_STATE_TRIGGER) return RESULT_ERROR(RES_ERR_INVALID);
    us_sensor->state = USS_STATE_RUNNING;
    HAL_GPIO_WritePin(us_sensor->const_h.trig_GPIOx, us_sensor->const_h.trig_GPIO_Pin_x, GPIO_PIN_RESET);
    return RESULT_OK(NULL);
}

static Result uss_overflow_inner(USSensor* us_sensor)
{
    if (us_sensor->state != USS_STATE_RUNNING) return RESULT_ERROR(RES_ERR_INVALID);
    us_sensor->state = USS_STATE_STOP;
    HAL_GPIO_WritePin(us_sensor->const_h.trig_GPIOx, us_sensor->const_h.trig_GPIO_Pin_x, GPIO_PIN_RESET);
    us_sensor->time = UINT16_MAX;
    us_sensor->distance = FLT_MAX;
    return RESULT_OK(NULL);
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

Result us_sensor_stop(USSensor* us_sensor)
{
    if (us_sensor->state != USS_STATE_RUNNING) return RESULT_ERROR(RES_ERR_INVALID);
    us_sensor->state = USS_STATE_STOP;
    const USSConst *const_h = &us_sensor->const_h;
    HAL_GPIO_WritePin(const_h->trig_GPIOx, const_h->trig_GPIO_Pin_x, GPIO_PIN_RESET);
    us_sensor->time = __HAL_TIM_GET_COUNTER(US_SENSOR_HTIM);
    us_sensor->distance = (float)us_sensor->time * 0.343f / 2.0f;  // uint:mm
    if      (us_sensor->distance <= const_h->danger ) us_sensor->status = USS_STATUS_DANGER;
    else if (us_sensor->distance <= const_h->warning) us_sensor->status = USS_STATUS_WARNING;
    else us_sensor->status = USS_STATUS_SAVE;
    return RESULT_OK(NULL);
}
