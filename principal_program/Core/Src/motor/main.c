#include "motor/main.h"
#include "tim.h"
#include "us_sensor/main.h"


#define HIGH_PASS   1
#define NONE_PASS   0
#define LOW_PASS   -1
// Commutation right_SEQUENCE for 120 degree control
static const int8_t SEQUENCE[6][3] = {
  { HIGH_PASS, LOW_PASS,  NONE_PASS },
  { HIGH_PASS, NONE_PASS, LOW_PASS  },
  { NONE_PASS, HIGH_PASS, LOW_PASS  },
  { LOW_PASS,  HIGH_PASS, NONE_PASS },
  { LOW_PASS,  NONE_PASS, HIGH_PASS },
  { NONE_PASS, LOW_PASS,  HIGH_PASS }
};

/**
 * STEP -> HALL
 * static const uint8_t  cw[6] = {4, 3, 5, 1, 2, 0};
 * static const uint8_t ccw[6] = {4, 0, 2, 1, 5, 3};
 *
 * HALL -> STEP
 */
static const uint8_t hall_index[] = {0xFF, 5, 3, 4, 1, 0, 2, 0xFF};

MotorParameter motor_left = {
    .const_h = {
        .Hall_GPIOx         = { GPIOD,      GPIOC,       GPIOA     },
        .Hall_GPIO_Pin_x    = { GPIO_PIN_2, GPIO_PIN_12, GPIO_PIN_15},
        //                      PA0(L28)       PA1(L30)       PA4(L32)
        .htimx              = { &htim2,        &htim2,        &htim3        },
        .TIM_CHANNEL_x      = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_2 },
        .Coil_GPIOx         = { GPIOB,      GPIOC,      GPIOC      },
        .Coil_GPIO_Pin_x    = { GPIO_PIN_7, GPIO_PIN_2, GPIO_PIN_3 },
    },
    .rps_max = MOTOR_MAX_SPEED,
};

MotorParameter motor_right = {
    .const_h = {
        .Hall_GPIOx         = { GPIOA,      GPIOB,      GPIOB      },
        .Hall_GPIO_Pin_x    = { GPIO_PIN_8, GPIO_PIN_4, GPIO_PIN_5 },
        //                      PC8(R02)       PA6(R13)       PB10(R25)
        .htimx              = { &htim3,        &htim3,        &htim2        },
        .TIM_CHANNEL_x      = { TIM_CHANNEL_3, TIM_CHANNEL_1, TIM_CHANNEL_3 },
        .Coil_GPIOx         = { GPIOB,       GPIOB,       GPIOB       },
        .Coil_GPIO_Pin_x    = { GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13 },
    },
    .rps_max = MOTOR_MAX_SPEED,
};

void motor_set_max_rps(MotorParameter* motor, float value)
{
    motor->rps_max = value;
}

void motor_set_rps_pcn(MotorParameter* motor, Percentage value)
{
    if (value > 100) value = 100;
    motor->rps_pcn = value;
}

void motor_set_direct(MotorParameter *motor, MotorMotion motion)
{
    motor->motion = motion;
}

void motor_set_state(MotorParameter *motor, MotorMode state)
{
    motor->state = state;
}

static void step_commutate(const MotorParameter *motor, uint8_t step)
{
    const MotorConst* const_h = &motor->const_h;
    uint8_t i;
    for (i = 0; i < 3; i++)
    {
        switch (SEQUENCE[step][i])
        {
            case HIGH_PASS:
            {
                __HAL_TIM_SET_COMPARE(const_h->htimx[i], const_h->TIM_CHANNEL_x[i], motor->pwm_duty);
                HAL_GPIO_WritePin(const_h->Coil_GPIOx[i], const_h->Coil_GPIO_Pin_x[i],  GPIO_PIN_RESET);
                break;
            }
            case LOW_PASS:
            {
                __HAL_TIM_SET_COMPARE(const_h->htimx[i], const_h->TIM_CHANNEL_x[i], 0);
                HAL_GPIO_WritePin(const_h->Coil_GPIOx[i], const_h->Coil_GPIO_Pin_x[i],  GPIO_PIN_SET);
                break;
            }
            default:
            {
                __HAL_TIM_SET_COMPARE(const_h->htimx[i], const_h->TIM_CHANNEL_x[i], 0);
                HAL_GPIO_WritePin(const_h->Coil_GPIOx[i], const_h->Coil_GPIO_Pin_x[i],  GPIO_PIN_RESET);
                break;
            }
        }
    }
}

static void motor_step_update(MotorParameter *motor)
{
    motor->hall_last = motor->hall_present;
    uint8_t hall_state =
          ((motor->const_h.Hall_GPIOx[0]->IDR & motor->const_h.Hall_GPIO_Pin_x[0]) ? 4U : 0U)
        | ((motor->const_h.Hall_GPIOx[1]->IDR & motor->const_h.Hall_GPIO_Pin_x[1]) ? 2U : 0U)
        | ((motor->const_h.Hall_GPIOx[2]->IDR & motor->const_h.Hall_GPIO_Pin_x[2]) ? 1U : 0U);
    if (hall_state == 0 || hall_state == 7) return;

    uint8_t step_next;
    switch (motor->motion_inner)
    {
        case MOTOR_DIRECTION_CLW:
        {
            motor->hall_present = hall_state;
            step_next = hall_index[motor->hall_present];
            break;
        }
        case MOTOR_DIRECTION_CCLW:
        {
            motor->hall_present = hall_state;
            step_next = (hall_index[motor->hall_present] + 3) % 6;
            break;
        }
        case MOTOR_DIRECTION_STOP:
        {
            step_next = hall_index[motor->hall_last];
            break;
        }
        default: return;
    }
    if (step_next == 0xFF) return;
    step_commutate(motor, step_next);
}

void motor_HALL_EXTI(MotorParameter *motor)
{
    motor->step_count++;
    motor_step_update(motor);
}

static void pwm_setup(const MotorParameter *motor)
{
    const MotorConst* const_h = &motor->const_h;
    HAL_TIM_PWM_Start(const_h->htimx[0], const_h->TIM_CHANNEL_x[0]);
    HAL_TIM_PWM_Start(const_h->htimx[1], const_h->TIM_CHANNEL_x[1]);
    HAL_TIM_PWM_Start(const_h->htimx[2], const_h->TIM_CHANNEL_x[2]);
}

static void datas_update(MotorParameter *motor, float ms)
{
    if (ms <= 0) return;
    motor->rps_present = (float)motor->step_count * 1000.0f / ((6 * 3) * ms);
    motor->step_count = 0;
    uint8_t diff = (hall_index[motor->hall_last] + 6 - hall_index[motor->hall_present]) % 6;
    switch (diff)
    {
        case 5:
        {
            motor->motion_present = MOTOR_DIRECTION_CLW;
            break;
        }
        case 0:
        {
            motor->motion_present = MOTOR_DIRECTION_STOP;
            break;
        }
        case 1:
        {
            motor->motion_present = MOTOR_DIRECTION_CCLW;
            break;
        }
        default: return;
    }
}

static void state_update(MotorParameter *motor)
{
    // 避免馬達應動未動
    if (
           motor->rps_present == 0
        // && motor->pwm_duty != 0
    ) motor_step_update(motor);

    if (
           (motor->state != MOTOR_STATE_LOCK)
        && (motor->motion_inner != motor->motion)
    ) {
        motor->state_inner = MOTOR_STATE_SLOW;
        if (motor->rps_present > MOTOR_STOP_GATE) return;
        motor->motion_inner = motor->motion;
    }
    motor->state_inner = motor->state;
}

static void rps_control(MotorParameter *motor, float ms)
{
    switch (motor->state_inner)
    {
        case MOTOR_STATE_CONTROL:
        {
            motor->rps_pcn_inner = motor->rps_pcn;
            break;
        }
        case MOTOR_STATE_FREE:
        {
            motor->pwm_duty = motor->rps_pcn;
            return;
        }
        case MOTOR_STATE_SLOW:
        {
            motor->rps_pcn_inner = 0;
            break;
        }
        case MOTOR_STATE_COAST:
        {
            motor->pwm_duty = 0;
            motor->rps_pcn_inner = 0;
            motor->integral_record = 0;
            return;
        }
        case MOTOR_STATE_LOCK:
        {
            motor->pwm_duty = 20;
            motor->rps_pcn_inner = 0;
            motor->motion_inner = MOTOR_DIRECTION_STOP;
            motor->integral_record = 0;
            return;
        }
        default: break;
    }
    if (
           (motor->motion_inner == MOTOR_DIRECTION_STOP)
        || (   (motor->rps_present < MOTOR_STOP_GATE)
            && (motor->rps_pcn_inner == 0))
    ) {
        motor->pwm_duty = 0;
        motor->integral_record = 0;
        return;
    }
    // PI 控制
    // 計算誤差
    float error =
        (motor->rps_max * motor->rps_pcn_inner / 100.0f)
        - motor->rps_present;
    // 累積誤差 error*秒
    float integral =
        motor->integral_record
        // Todo CHECK
        + error * ms / 1000.0f;
    // 計算 PI 控制輸出
    float output =
        (float)motor->pwm_duty
        + MOTOR_PI_KP * error
        + MOTOR_PI_KI * integral;
    // 避免積分風暴
    if      (output < 0.0f  ) motor->pwm_duty = 0;
    else if (output > 100.0f) motor->pwm_duty = 100;
    else
    {
        motor->pwm_duty = (Percentage)output;
        motor->integral_record = integral;
    }
    return;
}

#define MOTOR_TASK_DELAY_MS     50
void StartMotorTask(void *argument)
{
    HAL_TIM_Base_Start_IT(MOTOR_HTIM1);
    HAL_TIM_Base_Start_IT(MOTOR_HTIM2);
    pwm_setup(&motor_right);
    pwm_setup(&motor_left);
    const uint32_t osPeriod = pdMS_TO_TICKS(MOTOR_TASK_DELAY_MS);
    uint32_t next_wake = osKernelGetTickCount() + osPeriod;
    for(;;)
    {
        datas_update(&motor_left, MOTOR_TASK_DELAY_MS);
        datas_update(&motor_right, MOTOR_TASK_DELAY_MS);
        state_update(&motor_left);
        state_update(&motor_right);
        rps_control(&motor_left, MOTOR_TASK_DELAY_MS);
        rps_control(&motor_right, MOTOR_TASK_DELAY_MS);
        // us_sensor_enable(&us_sensor_head);
        osDelayUntil(next_wake);
        next_wake += osPeriod;
    }
    osThreadExit();
}
