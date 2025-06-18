#include "motor/main.h"
#include "tim.h"
#include "motor/PI_control.h"
#include "main/vehicle.h"

// Commutation right_SEQUENCE for 120 degree control
static const int8_t SEQUENCE[6][3] = {
  { 1, -1,  0},
  { 1,  0, -1},
  { 0,  1, -1},
  {-1,  1,  0},
  {-1,  0,  1},
  { 0, -1,  1}
};

static const MotorConst motor_right_const = {
    .Hall_GPIOx         = { GPIOC,      GPIOC,      GPIOC      },
    .Hall_GPIO_Pin_x    = { GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3 },
    .TIMx               = { &htim2,        &htim2,        &htim2       },
    .TIM_CHANNEL_x      = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3},
    .Coil_GPIOx         = { GPIOB,       GPIOB,       GPIOB       },
    .Coil_GPIO_Pin_x    = { GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15 },
};
MotorParameter motor_right = {
    .rotate_direction   = clockwise,
    .currentStep        = 7,
    .motor_const        = &motor_right_const,
};

static const MotorConst motor_left_const = {
    .Hall_GPIOx         = { GPIOC,      GPIOC,      GPIOC     },
    .Hall_GPIO_Pin_x    = { GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_8},
    .TIMx               = { &htim3,        &htim3,        &htim3       },
    .TIM_CHANNEL_x      = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3},
    .Coil_GPIOx         = { GPIOC,       GPIOC,       GPIOC       },
    .Coil_GPIO_Pin_x    = { GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12 },
};
MotorParameter motor_left = {
    .rotate_direction   = counter_clockwise,
    .currentStep        = 7,
    .motor_const        = &motor_left_const,
};

/**
  * 啟動指定馬達之 PWM 定時器
  *
  * Start PWM timers for specified motor channels
  */
static inline void motor_tim_setup(const MotorParameter *motor)
{
    const MotorConst* motor_const = motor->motor_const;
    HAL_TIM_PWM_Start(motor_const->TIMx[0], motor_const->TIM_CHANNEL_x[0]);
    HAL_TIM_PWM_Start(motor_const->TIMx[1], motor_const->TIM_CHANNEL_x[1]);
    HAL_TIM_PWM_Start(motor_const->TIMx[2], motor_const->TIM_CHANNEL_x[2]);
    HAL_TIM_Base_Start_IT(motor_const->TIMx[0]);
}

/**
  * 設定並初始化左右馬達參數
  *
  * Motor Initialization for both motors
  */
void motor_setup(void)
{
    motor_tim_setup(&motor_right);
    motor_tim_setup(&motor_left);
    motor_step_update(&motor_right);
    motor_step_update(&motor_left);
}

/**
  * 執行馬達換相控制
  *
  * Execute motor commutation based on current step sequence
  */
static inline void motor_commutate(const MotorParameter *motor)
{
    const MotorConst* motor_const = motor->motor_const;
    for (int i = 0; i < 3; i++)
    {
        if (SEQUENCE[motor->currentStep][i] == 1)
        {
            __HAL_TIM_SET_COMPARE(motor_const->TIMx[i], motor_const->TIM_CHANNEL_x[i], motor->duty_value);
            HAL_GPIO_WritePin(motor_const->Coil_GPIOx[i], motor_const->Coil_GPIO_Pin_x[i],  GPIO_PIN_RESET);
        }
        else if (SEQUENCE[motor->currentStep][i] == -1)
        {
            __HAL_TIM_SET_COMPARE(motor_const->TIMx[i], motor_const->TIM_CHANNEL_x[i], 0);
            HAL_GPIO_WritePin(motor_const->Coil_GPIOx[i], motor_const->Coil_GPIO_Pin_x[i],  GPIO_PIN_SET);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(motor_const->TIMx[i], motor_const->TIM_CHANNEL_x[i], 0);
            HAL_GPIO_WritePin(motor_const->Coil_GPIOx[i], motor_const->Coil_GPIO_Pin_x[i],  GPIO_PIN_RESET);
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
    if (motor == &motor_left) return;
    const MotorConst* motor_const = motor->motor_const;
    uint8_t hallState =
        (HAL_GPIO_ReadPin(motor_const->Hall_GPIOx[0], motor_const->Hall_GPIO_Pin_x[0]) << 2) |
        (HAL_GPIO_ReadPin(motor_const->Hall_GPIOx[1], motor_const->Hall_GPIO_Pin_x[1]) << 1) |
        (HAL_GPIO_ReadPin(motor_const->Hall_GPIOx[2], motor_const->Hall_GPIO_Pin_x[2])     );
    if (motor->rotate_direction == counter_clockwise)
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
    else if(motor->rotate_direction == clockwise)
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

    motor_commutate(motor);
}

/**
  * 基於霍爾感測與時間計算即時速度
  *
  * Calculate actual speed from Hall counts and delta time
  */
float real_speed;
void motor_speed_calculate(MotorParameter *motor)
{
    if (motor == &motor_left) return;
    real_speed = (float)motor->step_count / (6 * 3);
    real_speed /= 0.1f;
    motor->speed_present = real_speed;
    motor->step_count = 0;
}

bool motor_set_duty(MotorParameter *motor, uint8_t value)
{
    // 限制PWM最大值&&最小值
    if (value > 100)
    {
        motor->duty_value = 100;
        return false;
    }
    // if (value < 0)
    // {
    //     motor->duty_value = 0;
    //     return false;
    // }
    motor->duty_value = value;
    return true;
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

inline void motor_set_adc_val(MotorParameter *motor, uint16_t value)
{
    motor->adc_value = value;
}

inline void motor_add_step_count(MotorParameter *motor)
{
    motor->step_count++;
}
