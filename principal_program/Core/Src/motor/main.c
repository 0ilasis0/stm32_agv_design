#include "motor/main.h"
#include "tim.h"
#include "cmsis_os.h"
#include "stm32g431xx.h"
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

static const ArmConst motor_right_const = {
    .Hall_GPIOx         = { GPIOC,      GPIOC,      GPIOC      },
    .Hall_GPIO_Pin_x    = { GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3 },
    // PA0(L28) PA1(L30) PB10(R25)
    .TIMx               = { &htim2,        &htim2,        &htim2       },
    .TIM_CHANNEL_x      = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3},
    .Coil_GPIOx         = { GPIOB,       GPIOB,       GPIOB       },
    .Coil_GPIO_Pin_x    = { GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15 },
};
ArmParameter motor_right = {
    .rotate_direction   = clockwise,
    .currentStep        = 7,
    .motor_const        = &motor_right_const,
};

static const ArmConst motor_left_const = {
    .Hall_GPIOx         = { GPIOC,      GPIOC,      GPIOC     },
    .Hall_GPIO_Pin_x    = { GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_8},
    // PA6(R13) PA4(L32) PB0(L34)
    .TIMx               = { &htim3,        &htim3,        &htim3       },
    .TIM_CHANNEL_x      = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3},
    .Coil_GPIOx         = { GPIOC,       GPIOC,       GPIOC       },
    .Coil_GPIO_Pin_x    = { GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12 },
};
ArmParameter motor_left = {
    .rotate_direction   = counter_clockwise,
    .currentStep        = 7,
    .motor_const        = &motor_left_const,
};

/**
  * 啟動指定馬達之 PWM 定時器
  *
  * Start PWM timers for specified motor channels
  */
static void motor_tim_setup(const ArmParameter *motor)
{
    const ArmConst* motor_const = motor->motor_const;
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
static void motor_setup(void)
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
static void motor_commutate(const ArmParameter *motor)
{
    const ArmConst* motor_const = motor->motor_const;
    const uint8_t currentStep = motor->currentStep;
    for (int i = 0; i < 3; i++)
    {
        if (SEQUENCE[currentStep][i] == 1)
        {
            __HAL_TIM_SET_COMPARE(motor_const->TIMx[i], motor_const->TIM_CHANNEL_x[i], motor->duty_value);
            HAL_GPIO_WritePin(motor_const->Coil_GPIOx[i], motor_const->Coil_GPIO_Pin_x[i],  GPIO_PIN_RESET);
        }
        else if (SEQUENCE[currentStep][i] == -1)
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
void motor_step_update(ArmParameter *motor)
{
    const ArmConst* motor_const = motor->motor_const;
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
void motor_speed_calculate(ArmParameter *motor, float sec)
{
    float real_speed = (float)motor->step_count / (6 * 3);
    motor->step_count = 0;
    real_speed /= sec;
    motor->speed_present = real_speed;
}

bool motor_set_duty(ArmParameter *motor, uint8_t value)
{
    // 限制PWM最大值&&最小值
    if (value > 100)
    {
        motor->duty_value = 100;
        return false;
    }
    if (value < 0)
    {
        motor->duty_value = 0;
        return false;
    }
    motor->duty_value = value;
    return true;
}

/**
  * @brief 設定馬達速度目標值，限制範圍 0~100
  * @retval true：成功，false：超出範圍並已修正
  */
bool motor_set_speed_setpoint(ArmParameter* motor, uint8_t value)
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
inline void motor_set_direction(ArmParameter *motor, ROTATE_STATUS direction)
{
    motor->rotate_direction = direction;
}

inline void motor_set_integral_record(ArmParameter *motor, float integral)
{
    motor->integral_record = integral;
}

inline void motor_set_adc_val(ArmParameter *motor, uint16_t value)
{
    motor->adc_value = value;
}

inline void motor_add_step_count(ArmParameter *motor)
{
    motor->step_count++;
}

void StartMotorTask(void *argument)
{
    motor_setup();
    uint16_t tick = 0;
    for(;;)
    {
        if (tick % 10 == 0) {
            if (motor_right.speed_present == 0) motor_step_update(&motor_right);
            if (motor_left.speed_present == 0) motor_step_update(&motor_left);
        }
        if (tick % 100 == 0) {
            motor_speed_calculate(&motor_right, 0.1f);
            motor_speed_calculate(&motor_left, 0.1f);
        }
        if (tick % 500 == 0) {
            tick = 0;
            motor_PI_control(&motor_right);
            motor_PI_control(&motor_left);
        }
        osDelay(1);
        tick++;
    }
}
