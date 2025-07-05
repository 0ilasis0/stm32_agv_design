#include "motor/main.h"
#include "tim.h"
#include "us_sensor/main.h"
#include "main/vehicle.h"

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

static const MotorConst motor_left_const = {
    .Hall_GPIOx         = { GPIOD,      GPIOC,       GPIOA     },
    .Hall_GPIO_Pin_x    = { GPIO_PIN_2, GPIO_PIN_12, GPIO_PIN_15},
    //                      PA0(L28)       PA1(L30)       PA4(L32)
    .htimx              = { &htim2,        &htim2,        &htim3        },
    .TIM_CHANNEL_x      = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_2 },
    .Coil_GPIOx         = { GPIOB,      GPIOC,      GPIOC      },
    .Coil_GPIO_Pin_x    = { GPIO_PIN_7, GPIO_PIN_2, GPIO_PIN_3 },
};
MotorParameter motor_left = {
    .const_h            = &motor_left_const,
    .direction_setpoint = rotate_c_clockwise,
    .current_step       = -1,
};

static const MotorConst motor_right_const = {
    .Hall_GPIOx         = { GPIOA,      GPIOB,      GPIOB      },
    .Hall_GPIO_Pin_x    = { GPIO_PIN_8, GPIO_PIN_4, GPIO_PIN_5 },
    //                      PC8(R02)       PA6(R13)       PB10(R25)
    .htimx              = { &htim3,        &htim3,        &htim2        },
    .TIM_CHANNEL_x      = { TIM_CHANNEL_3, TIM_CHANNEL_1, TIM_CHANNEL_3 },
    .Coil_GPIOx         = { GPIOB,       GPIOB,       GPIOB       },
    .Coil_GPIO_Pin_x    = { GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13 },
};
MotorParameter motor_right = {
    .const_h            = &motor_right_const,
    .direction_setpoint = rotate_clockwise,
    .current_step       = -1,
};

/**
  * 執行馬達換相控制
  *
  * Execute motor commutation based on current step sequence
  */
static void step_commutate(const MotorParameter *motor)
{
    const MotorConst* const_h = motor->const_h;
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

/**
  * 更新馬達轉速步數並依據霍爾感測器讀值決定下一換相步驟
  *
  * Update motor step count and determine next step from Hall sensor readings
  */
void motor_step_update(MotorParameter *motor)
{
    const MotorConst* const_h = motor->const_h;
    uint8_t hallState =
        (HAL_GPIO_ReadPin(const_h->Hall_GPIOx[0], const_h->Hall_GPIO_Pin_x[0]) << 2) |
        (HAL_GPIO_ReadPin(const_h->Hall_GPIOx[1], const_h->Hall_GPIO_Pin_x[1]) << 1) |
        (HAL_GPIO_ReadPin(const_h->Hall_GPIOx[2], const_h->Hall_GPIO_Pin_x[2])     );
    if (motor->direction_present == rotate_c_clockwise)
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
    else if(motor->direction_present == rotate_clockwise)
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

/**
  * 基於霍爾感測與時間計算即時速度
  *
  * Calculate actual speed from Hall counts and delta time
  */
void motor_rps_calculate(MotorParameter *motor, float sec)
{
    if (sec <= 0) return;
    motor->rps_present = (float)motor->step_count / (6 * 3) / sec;
    motor->step_count = 0;
}

FnState motor_set_duty(MotorParameter *motor, uint8_t value)
{
    // 限制PWM最大值&&最小值
    if (value > 100)
    {
        motor->duty = 100;
        return FNS_FAIL;
    }
    motor->duty = value;
    return FNS_OK;
}

/**
  * @brief 設定馬達速度目標值，限制範圍 0~100
  * @retval true：成功，false：超出範圍並已修正
  */
bool motor_set_speed(MotorParameter* motor, Percentage value)
{
    if (value > 100)
    {
        motor->rps_sepoint = 100;
        return false;
    }
    motor->rps_sepoint = value;
    return true;
}

/**
  * @brief 設定馬達旋轉方向（rotate_direction）
  */
inline void motor_set_direction(MotorParameter *motor, ROTATE_STATUS direction)
{
    motor->direction_present = direction;
}

inline void motor_add_step_count(MotorParameter *motor)
{
    motor->step_count++;
}

/* +PI speed control ------------------------------------------------*/
static void PI_control(MotorParameter *motor)
{
    if (!sys_run_switch.enable_PI) return;

    // 計算誤差
    float error =
          (max_speed * motor->rps_setpoint_inner / 100.0f)
        - motor->rps_present;
    // 累積誤差
    float integral =
          motor->integral_record
        + error;
    // 計算 P I 控制輸出
    float output_duty =
          (float)motor->duty
        + MOTOR_PI_KP * error
        + MOTOR_PI_KI * integral;

    // 避免太大的error
    if (output_duty >= 0.0f)
    {
        if (
               motor->rps_present == 0
            && motor->rps_sepoint == 0
        ) {
            output_duty = 0.0f;
        }
        motor_set_duty(motor, (uint8_t)output_duty);
        motor->integral_record = integral;
    }
    else
    {
        motor_set_duty(motor, 0);
    }
}

/**
  * 啟動指定馬達之 PWM 定時器
  *
  * Start PWM timers for specified motor channels
  */
static inline void pwm_setup(const MotorParameter *motor)
{
    const MotorConst* const_h = motor->const_h;
    HAL_TIM_PWM_Start(const_h->htimx[0], const_h->TIM_CHANNEL_x[0]);
    HAL_TIM_PWM_Start(const_h->htimx[1], const_h->TIM_CHANNEL_x[1]);
    HAL_TIM_PWM_Start(const_h->htimx[2], const_h->TIM_CHANNEL_x[2]);
}

static void speed_direc_update(MotorParameter *motor)
{
    if (motor->direction_setpoint != motor->direction_present)
    {
        motor->rps_setpoint_inner = 0;
        if (motor->rps_present == 0)
        {
            motor->direction_setpoint = motor->direction_present;
            motor->rps_setpoint_inner = motor->rps_sepoint;
        }
    }
    else
    {
        motor->rps_setpoint_inner = motor->rps_sepoint;
    }
    PI_control(motor);
    if (motor->rps_present == 0) motor_step_update(motor);     //motor_step_update？？
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
