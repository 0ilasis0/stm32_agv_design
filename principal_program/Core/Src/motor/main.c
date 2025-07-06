#include "motor/main.h"
#include "tim.h"
#include "us_sensor/main.h"
#include "vehicle/vehicle.h"

float max_speed = MOTOR_MAX_SPEED;

// Commutation right_SEQUENCE for 120 degree control
static const int8_t SEQUENCE[6][3] = {
  { 1, -1,  0},
  { 1,  0, -1},
  { 0,  1, -1},
  {-1,  1,  0},
  {-1,  0,  1},
  { 0, -1,  1}
};

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
    .direction_setpoint = rotate_c_clockwise,
    .current_step       = -1,
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
    .direction_setpoint = rotate_clockwise,
    .current_step       = -1,
};

static void step_commutate(const MotorParameter *motor)
{
    const MotorConst* const_h = &motor->const_h;
    const uint8_t current_step = motor->current_step;
    for (int i = 0; i < 3; i++)
    {
        if (SEQUENCE[current_step][i] == 1)
        {
            __HAL_TIM_SET_COMPARE(const_h->htimx[i], const_h->TIM_CHANNEL_x[i], motor->duty);
            HAL_GPIO_WritePin(const_h->Coil_GPIOx[i], const_h->Coil_GPIO_Pin_x[i],  GPIO_PIN_RESET);
        }
        else if (SEQUENCE[current_step][i] == -1)
        {
            __HAL_TIM_SET_COMPARE(const_h->htimx[i], const_h->TIM_CHANNEL_x[i], 0);
            HAL_GPIO_WritePin(const_h->Coil_GPIOx[i], const_h->Coil_GPIO_Pin_x[i],  GPIO_PIN_SET);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(const_h->htimx[i], const_h->TIM_CHANNEL_x[i], 0);
            HAL_GPIO_WritePin(const_h->Coil_GPIOx[i], const_h->Coil_GPIO_Pin_x[i],  GPIO_PIN_RESET);
        }
    }
}

void motor_step_update(MotorParameter *motor)
{
    const MotorConst* const_h = &motor->const_h;
    uint8_t hallState =
        (HAL_GPIO_ReadPin(const_h->Hall_GPIOx[0], const_h->Hall_GPIO_Pin_x[0]) << 2) |
        (HAL_GPIO_ReadPin(const_h->Hall_GPIOx[1], const_h->Hall_GPIO_Pin_x[1]) << 1) |
        (HAL_GPIO_ReadPin(const_h->Hall_GPIOx[2], const_h->Hall_GPIO_Pin_x[2])     );
    if (motor->direction_inner == rotate_c_clockwise)
    {
        switch(hallState)
        {
            case 2: motor->current_step = 0; break;
            case 3: motor->current_step = 1; break;
            case 1: motor->current_step = 2; break;
            case 5: motor->current_step = 3; break;
            case 4: motor->current_step = 4; break;
            case 6: motor->current_step = 5; break;
            default: return;
        }
    }
    else if(motor->direction_inner == rotate_clockwise)
    {
        switch(hallState)
        {
            case 5: motor->current_step = 0; break;
            case 4: motor->current_step = 1; break;
            case 6: motor->current_step = 2; break;
            case 2: motor->current_step = 3; break;
            case 3: motor->current_step = 4; break;
            case 1: motor->current_step = 5; break;
            default: return;
        }
    }
    step_commutate(motor);
}

void motor_set_duty(MotorParameter *motor, uint8_t value)
{
    if (value > 100) motor->duty = 100;
    motor->duty = value;
}

void PI_control(MotorParameter *motor, float ms)
{
    if (!sys_run_switch.enable_PI) return;
    if (
           (motor->rps_present < MOTOR_STOP_GATE)
        && (motor->rps_inner == 0)
    ) {
        motor_set_duty(motor, 0);
        motor->integral_record = 0;
        return;
    }
    // 計算誤差
    float error =
          (max_speed * motor->rps_inner / 100.0f)
        - motor->rps_present;
    // 累積誤差 error*秒
    float integral =
          motor->integral_record
        + error * ms / 1000.0f;
    // 計算 PI 控制輸出
    float output =
          (float)motor->duty
        + MOTOR_PI_KP * error
        + MOTOR_PI_KI * integral;
    // 避免積分風暴
    if (output < 0.0f)
    {
        motor_set_duty(motor, 0);
    }
    else if (output > 100.0f)
    {
        motor_set_duty(motor, 100);
    }
    else
    {
        motor_set_duty(motor, (uint8_t)output);
        motor->integral_record = integral;
    }
}

void motor_rps_calculate(MotorParameter *motor, float ms)
{
    if (ms <= 0) return;
    motor->rps_present = (float)motor->step_count * 1000.0f / ((6 * 3) * ms);
    motor->step_count = 0;
}

bool motor_set_speed(MotorParameter* motor, Percentage value)
{
    if (value > 100)
    {
        motor->rps_setpoint = 100;
        return false;
    }
    motor->rps_setpoint = value;
    return true;
}

inline void motor_set_direction(MotorParameter *motor, ROTATE_STATUS direction)
{
    motor->direction_setpoint = direction;
}

inline void motor_set_stop(MotorParameter *motor, bool stop)
{
    motor->stop = stop;
}

inline void motor_add_step_count(MotorParameter *motor)
{
    motor->step_count++;
}

static inline void pwm_setup(const MotorParameter *motor)
{
    const MotorConst* const_h = &motor->const_h;
    HAL_TIM_PWM_Start(const_h->htimx[0], const_h->TIM_CHANNEL_x[0]);
    HAL_TIM_PWM_Start(const_h->htimx[1], const_h->TIM_CHANNEL_x[1]);
    HAL_TIM_PWM_Start(const_h->htimx[2], const_h->TIM_CHANNEL_x[2]);
}

static void speed_direc_update(MotorParameter *motor)
{
    if (motor->direction_inner != motor->direction_setpoint)
    {
        motor->rps_inner = 0;
        if (motor->rps_present < MOTOR_STOP_GATE)
        {
            motor->direction_inner = motor->direction_setpoint;
            motor->rps_inner = motor->rps_setpoint;
        }
    }
    else if (motor->stop == 1)
    {
        motor->rps_inner = 0;
    }
    else
    {
        motor->rps_inner = motor->rps_setpoint;
    }
    // 避免馬達應動未動
    if (
           motor->rps_present == 0
        && motor->duty != 0
    ) motor_step_update(motor);
}

void StartMotorTask(void *argument)
{
    HAL_TIM_Base_Start_IT(MOTOR_HTIM1);
    HAL_TIM_Base_Start_IT(MOTOR_HTIM2);
    pwm_setup(&motor_right);
    pwm_setup(&motor_left);
    uint16_t tick = 0;
    for(;;)
    {
        speed_direc_update(&motor_right);
        speed_direc_update(&motor_left);
        // us_sensor_enable(&us_sensor_head);
        if (tick % 100 == 0)
        {
            tick = 0;
        }
        osDelay(10);
        tick++;
    }
}
