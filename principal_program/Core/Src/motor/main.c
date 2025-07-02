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
    .rotate_direction   = rotate_c_clockwise,
    .currentStep        = 7,
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
    .rotate_direction   = rotate_clockwise,
    .currentStep        = 7,
};

/**
  * 啟動指定馬達之 PWM 定時器
  *
  * Start PWM timers for specified motor channels
  */
static void tim_setup(const MotorParameter *motor)
{
    const MotorConst* const_h = motor->const_h;
    HAL_TIM_PWM_Start(const_h->htimx[0], const_h->TIM_CHANNEL_x[0]);
    HAL_TIM_PWM_Start(const_h->htimx[1], const_h->TIM_CHANNEL_x[1]);
    HAL_TIM_PWM_Start(const_h->htimx[2], const_h->TIM_CHANNEL_x[2]);
    HAL_TIM_Base_Start_IT(const_h->htimx[0]);
    // HAL_TIM_Base_Start_IT(const_h->htimx[1]);
    // HAL_TIM_Base_Start_IT(const_h->htimx[2]);
}

/**
  * 設定並初始化左右馬達參數
  *
  * Motor Initialization for both motors
  */
static void setup(void)
{
    tim_setup(&motor_right);
    tim_setup(&motor_left);
    motor_step_update(&motor_right);
    motor_step_update(&motor_left);
}

/**
  * 執行馬達換相控制
  *
  * Execute motor commutation based on current step sequence
  */
static void step_commutate(const MotorParameter *motor)
{
    const MotorConst* const_h = motor->const_h;
    const uint8_t currentStep = motor->currentStep;
    for (int i = 0; i < 3; i++)
    {
        if (SEQUENCE[currentStep][i] == 1)
        {
            __HAL_TIM_SET_COMPARE(const_h->htimx[i], const_h->TIM_CHANNEL_x[i], motor->duty_value);
            HAL_GPIO_WritePin(const_h->Coil_GPIOx[i], const_h->Coil_GPIO_Pin_x[i],  GPIO_PIN_RESET);
        }
        else if (SEQUENCE[currentStep][i] == -1)
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
    if (motor->rotate_direction == rotate_c_clockwise)
    {
        switch(hallState)
        {
            case 2: motor->currentStep = 0; break;
            case 3: motor->currentStep = 1; break;
            case 1: motor->currentStep = 2; break;
            case 5: motor->currentStep = 3; break;
            case 4: motor->currentStep = 4; break;
            case 6: motor->currentStep = 5; break;
        }
    }
    else if(motor->rotate_direction == rotate_clockwise)
    {
        switch(hallState)
        {
            case 5: motor->currentStep = 0; break;
            case 4: motor->currentStep = 1; break;
            case 6: motor->currentStep = 2; break;
            case 2: motor->currentStep = 3; break;
            case 3: motor->currentStep = 4; break;
            case 1: motor->currentStep = 5; break;
        }
    }

    step_commutate(motor);
}

/**
  * 基於霍爾感測與時間計算即時速度
  *
  * Calculate actual speed from Hall counts and delta time
  */
void motor_speed_calculate(MotorParameter *motor, float sec)
{
    float real_speed = (float)motor->step_count / (6 * 3);
    motor->step_count = 0;
    real_speed /= sec;
    motor->speed_present = real_speed;
}

FnState motor_set_duty(MotorParameter *motor, int8_t value)
{
    // 限制PWM最大值&&最小值
    if (value > 100)
    {
        motor->duty_value = 100;
        return FNS_FAIL;
    }
    else if (value < 0)
    {
        motor->duty_value = 0;
        return FNS_FAIL;
    }
    motor->duty_value = value;
    return FNS_OK;
}

/**
  * @brief 設定馬達速度目標值，限制範圍 0~100
  * @retval true：成功，false：超出範圍並已修正
  */
bool motor_set_speed_setpoint(MotorParameter* motor, uint8_t value)
{
    if (value > 100)
    {
        motor->speed_sepoint_pcn = 100;
        return false;
    }
    motor->speed_sepoint_pcn = value;
    return true;
}

/**
  * @brief 設定馬達旋轉方向（rotate_direction）
  */
inline void motor_set_direction(MotorParameter *motor, ROTATE_STATUS direction)
{
    motor->rotate_direction = direction;
}

inline void motor_set_integral_record(MotorParameter *motor, float integral)
{
    motor->integral_record = integral;
}

inline void motor_add_step_count(MotorParameter *motor)
{
    motor->step_count++;
}

/* +PI speed control ------------------------------------------------*/
static void PI_control(MotorParameter *motor)
{
    if (!sys_run_switch.enable_PI) return;

    float setpoint = (float)max_speed * motor->speed_sepoint_pcn / 100;

    // 計算誤差
    float error = setpoint - motor->speed_present;
    float integral = motor->integral_record + error;
    // 計算 P I 控制輸出
    float output_pwm_Value = (float)MOTOR_PI_KP * error + MOTOR_PI_KI * integral;

    reset_duty_if_speed_zero(motor);

    // 避免太大的error
    if (motor_set_duty(motor, motor->duty_value + output_pwm_Value)) return;
    motor_set_integral_record(motor, integral);
}

void reset_duty_if_speed_zero(MotorParameter *motor)
{
    if (
        motor->speed_present == 0
        && motor->speed_sepoint_pcn == 0
        ) {
        motor_set_duty(motor, 0);
    }
}

void StartMotorTask(void *argument)
{
    setup();
    uint16_t tick = 0;
    for(;;)
    {
        if (tick % 10 == 0)
        {
            if (motor_right.speed_present == 0) motor_step_update(&motor_right);
            if (motor_left.speed_present == 0) motor_step_update(&motor_left);
            // us_sensor_enable(&us_sensor_head);
        }
        if (tick % 500 == 0)
        {
            tick = 0;
            PI_control(&motor_right);
            PI_control(&motor_left);
        }
        osDelay(1);
        tick++;
    }
}
