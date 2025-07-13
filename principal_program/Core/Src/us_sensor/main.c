#include "us_sensor/main.h"
#include "vehicle/basic.h"
#include "tim.h"

USSensor us_sensor_head = {
    .const_h = {
        .trig_GPIOx = GPIOC,
        .trig_GPIO_Pin_x = GPIO_PIN_5,
        .echo_GPIOx = GPIOC,
        .echo_GPIO_Pin_x = GPIO_PIN_6,
    },
    .state = USSS_STOP,
};

FnState us_sensor_enable(USSensor* us_sensor)
{
    if (us_sensor->state != USSS_STOP) return FNS_BUSY;
    us_sensor->state = USSS_WAITING;
    return FNS_OK;
}

static FnState uss_start_inner(USSensor* us_sensor)
{
    if (us_sensor->state != USSS_WAITING) return FNS_INVALID;
    us_sensor->state = USSS_TRIGGER;
    HAL_GPIO_WritePin(us_sensor->const_h.trig_GPIOx, us_sensor->const_h.trig_GPIO_Pin_x, GPIO_PIN_SET);
    return FNS_OK;
}

static FnState uss_tri_off_inner(USSensor* us_sensor)
{
    if (us_sensor->state != USSS_TRIGGER) return FNS_INVALID;
    us_sensor->state = USSS_RUNNING;
    HAL_GPIO_WritePin(us_sensor->const_h.trig_GPIOx, us_sensor->const_h.trig_GPIO_Pin_x, GPIO_PIN_RESET);
    return FNS_OK;
}

static FnState uss_overflow_inner(USSensor* us_sensor)
{
    if (us_sensor->state != USSS_RUNNING) return FNS_INVALID;
    us_sensor->state = USSS_STOP;
    HAL_GPIO_WritePin(us_sensor->const_h.trig_GPIOx, us_sensor->const_h.trig_GPIO_Pin_x, GPIO_PIN_RESET);
    us_sensor->time = 0xFFFF;
    us_sensor->distance = -1;
    return FNS_OK;
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

FnState us_sensor_stop(USSensor* us_sensor)
{
    if (us_sensor->state != USSS_RUNNING) return FNS_INVALID;
    us_sensor->state = USSS_STOP;
    HAL_GPIO_WritePin(us_sensor->const_h.trig_GPIOx, us_sensor->const_h.trig_GPIO_Pin_x, GPIO_PIN_RESET);
    us_sensor->time = __HAL_TIM_GET_COUNTER(US_SENSOR_HTIM);
    us_sensor->distance = (float)us_sensor->time * 0.0343f / 2.0f;  // uint:cm
    return FNS_OK;
}

void us_sensor_main(void)
{
    if (us_sensor_head.distance <= 20)
    {
        vehicle_set_motion(VEHICLE_DIRECT_STOP);
        vehicle_set_mode(VEHICLE_MODE_FREE);

        // vehicle_set_motion(VEHICLE_DIRECT_BACKWARD);
        // vehicle_set_mode(VEHICLE_MODE_F_TRACK);
    }
}
