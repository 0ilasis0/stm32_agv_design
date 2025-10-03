#include "us_sensor/main.h"
#include "vehicle/basic.h"
#include "tim.h"
#include "connectivity/fdcan/main.h"
#include "connectivity/fdcan/pkt_write.h"

USSensor us_sensor_head = {
    .const_h = {
        .trig_GPIOx = GPIOC,
        .trig_GPIO_Pin_x = GPIO_PIN_5,
        .echo_GPIOx = GPIOC,
        .echo_GPIO_Pin_x = GPIO_PIN_6,
        .warning = 800.0f,
        .danger = 550.0f,
    },
    .state = USS_STATE_STOP,
};

uint32_t hyrun[5] = {0};

Result us_sensor_start(void)
{
    USSensor *self = &us_sensor_head;
    if (self->state != USS_STATE_WAITING) return RESULT_ERROR(RES_ERR_INVALID);
    hyrun[0]++;
    self->state = USS_STATE_RUNNING;
    HAL_GPIO_WritePin(self->const_h.trig_GPIOx, self->const_h.trig_GPIO_Pin_x, GPIO_PIN_SET);
    return RESULT_OK(self);
}

Result us_sensor_enable(void)
{
    USSensor *self = &us_sensor_head;
    if (self->state != USS_STATE_STOP) return RESULT_ERROR(RES_ERR_BUSY);
    HAL_TIM_Base_Start_IT(US_SENSOR_HTIM);
    HAL_TIM_PWM_Start_IT(US_SENSOR_HTIM, US_SENSOR_TIM_CH);
    self->state = USS_STATE_WAITING;
    us_sensor_start();
    return RESULT_OK(self);
}

Result us_sensor_tri_off(void)
{
    USSensor *self = &us_sensor_head;
    if (self->state != USS_STATE_RUNNING) return RESULT_ERROR(RES_ERR_INVALID);
    hyrun[1]++;
    HAL_GPIO_WritePin(self->const_h.trig_GPIOx, self->const_h.trig_GPIO_Pin_x, GPIO_PIN_RESET);
    return RESULT_OK(self);
}

Result us_sensor_overflow(void)
{
    USSensor *self = &us_sensor_head;
    if (self->state == USS_STATE_RUNNING)
    {
        hyrun[3]++;
        self->state = USS_STATE_WAITING;
        HAL_GPIO_WritePin(self->const_h.trig_GPIOx, self->const_h.trig_GPIO_Pin_x, GPIO_PIN_RESET);
        self->time = UINT32_MAX;
        self->distance = UINT32_MAX;
        us_sensor_start();
        return RESULT_OK(self);
    }
    else if (self->state == USS_STATE_STOP)
    {
        hyrun[4]++;
        self->state = USS_STATE_WAITING;
        HAL_GPIO_WritePin(self->const_h.trig_GPIOx, self->const_h.trig_GPIO_Pin_x, GPIO_PIN_RESET);
        us_sensor_start();
        return RESULT_OK(self);
    }
    else return RESULT_ERROR(RES_ERR_INVALID);
}

static bool uss_danger_flag = false;
Result us_sensor_echo(void)
{
    USSensor *self = &us_sensor_head;
    if (self->state != USS_STATE_RUNNING) return RESULT_ERROR(RES_ERR_INVALID);
    hyrun[2]++;
    self->state = USS_STATE_STOP;
    const USSConst *const_h = &self->const_h;
    HAL_GPIO_WritePin(const_h->trig_GPIOx, const_h->trig_GPIO_Pin_x, GPIO_PIN_RESET);
    self->time = __HAL_TIM_GET_COUNTER(US_SENSOR_HTIM);
    self->distance = (uint32_t)((float)self->time * 0.343f / 2.0f);  // uint:mm
    if (self->distance <= const_h->danger)
    {
        self->status = USS_STATUS_DANGER;
        if (!uss_danger_flag)
        {
            uss_danger_flag = true;
            FdcanPkt* pkt;
            pkt = RESULT_UNWRAP_HANDLE(fdcan_pkt_pool_alloc());
            fdcan_rfid_reset(pkt);
            fdcan_pkt_buf_push(&fdcan_trsm_pkt_buf, pkt);
        }
    }
    else if (self->distance <= const_h->warning)
    {
        self->status = USS_STATUS_WARNING;
        uss_danger_flag = false;
    }
    else
    {
        self->status = USS_STATUS_SAVE;
        uss_danger_flag = false;
    }
    return RESULT_OK(self);
}
