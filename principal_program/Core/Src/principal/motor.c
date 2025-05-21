#include "principal/motor.h"
#include "principal/PI_control.h"
#include "tim.h"

// Commutation right_SEQUENCE for 120 degree control
const int SEQUENCE[6][3] = {
  { 1, -1,  0},
  { 1,  0, -1},
  { 0,  1, -1},
  {-1,  1,  0},
  {-1,  0,  1},
  { 0, -1,  1}
};

MOTOR_PARAMETER motor_right;
MOTOR_PARAMETER motor_left;

/**
  * 初始化 Motor 參數結構並回傳
  *
  * Initialized MOTOR_PARAMETER structure and return motor parameters
  */
MOTOR_PARAMETER motor_new(
    // range 1~100
    uint8_t speed_sepoint,
    ROTATE_STATUS rotate_direction,

    double integral_record,
    uint16_t step_count,
    uint16_t adc_value,
    // range 1~100
    uint8_t duty_value,
    float present_speed,
    uint8_t currentStep,

    GPIO_TypeDef* Hall_GPIOx[3],
    uint16_t Hall_GPIO_Pin_x[3],

    GPIO_TypeDef* M_GPIOx[3],
    uint16_t M_GPIO_Pin_x[3],

    TIM_HandleTypeDef* TIMx[3],
    uint32_t TIM_CHANNEL_x[3]
) {
    MOTOR_PARAMETER motor;

    motor.speed_sepoint = speed_sepoint;
    motor.rotate_direction = rotate_direction;
    motor.integral_record = integral_record;
    motor.step_count = step_count;
    motor.adc_value = adc_value;
    motor.duty_value = duty_value;
    motor.present_speed = present_speed;
    motor.currentStep = currentStep;

    for (int i = 0; i < 3; i++) {
        motor.Hall_GPIOx[i] = Hall_GPIOx[i];
        motor.Hall_GPIO_Pin_x[i] = Hall_GPIO_Pin_x[i];
        motor.TIMx[i] = TIMx[i];
        motor.TIM_CHANNEL_x[i] = TIM_CHANNEL_x[i];
    }

    for (int i = 0; i < 3; i++) {
        motor.M_GPIOx[i] = M_GPIOx[i];
        motor.M_GPIO_Pin_x[i] = M_GPIO_Pin_x[i];
    }

    return motor;
}

/**
  * 設定並初始化左右馬達參數
  *
  * Motor Initialization for both motors
  */
void motor_setup(void) {
    motor_right = motor_new(
        30,                 // speed_sepoint  圈/s
        clockwise,          // clockwise counter_clockwise
        0.0,                // integral_record   PI積分累積儲存
        0,                  // rpm_count  計數motor speed
        0,                  // adc_value
        15,                 // duty_value ; Current PWM value (adjustable)
        0,                  // present_speed
        7,                  // currentStep

        // Hall sensor pins
        (GPIO_TypeDef* [])  {GPIOC,      GPIOC,      GPIOC      },
        (uint16_t [])       {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3 },

        // AL BL CL > right Pin definitions for the three phases
        (GPIO_TypeDef* [])  {GPIOB,       GPIOB,       GPIOB      },
        (uint16_t [])       {GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15},
        // AH:A0(L28) BH:A1(L30) CH:B10(R25)
        (TIM_HandleTypeDef* []) {&htim2,        &htim2,        &htim2       },
        (uint32_t [])           {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3}
    );
    motor_left = motor_new(
        30,                 // speed_sepoint
        counter_clockwise,  // clockwise counter_clockwise
        0.0,                // integral_record   PI積分累積儲存
        0,                  // rpm_count  計數motor speed
        0,                  // adc_value
        15,                 // duty_value ; Current PWM value (adjustable)
        0,                  // present_speed
        7,                  // currentStep

        // Hall sensor pins
        (GPIO_TypeDef* [])  {GPIOC,      GPIOC,      GPIOC     },
        (uint16_t [])       {GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_8},

        // AL BL CL > left Pin definitions for the three phases
        (GPIO_TypeDef* [])  {GPIOC,       GPIOC,       GPIOC      },
        (uint16_t [])       {GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12},

        // AH:A6(R13) BH:A4(L32) CH:B0(L34)
        (TIM_HandleTypeDef* []) {&htim3,        &htim3,        &htim3       },
        (uint32_t [])           {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3}
    );

    motor_tim_setup(&motor_right);
    motor_tim_setup(&motor_left);

    update_motor_step(&motor_right);
    update_motor_step(&motor_left);
}

/**
  * 啟動指定馬達之 PWM 定時器
  *
  * Start PWM timers for specified motor channels
  */
void motor_tim_setup(const MOTOR_PARAMETER *motor) {
    HAL_TIM_PWM_Start(motor->TIMx[0], motor->TIM_CHANNEL_x[0]);
    HAL_TIM_PWM_Start(motor->TIMx[1], motor->TIM_CHANNEL_x[1]);
    HAL_TIM_PWM_Start(motor->TIMx[2], motor->TIM_CHANNEL_x[2]);
    HAL_TIM_Base_Start_IT(motor->TIMx[0]);
}

/**
  * 執行馬達換相控制
  *
  * Execute motor commutation based on current step sequence
  */
void commutate_motor(const MOTOR_PARAMETER *motor) {
    for (int i = 0; i < 3; i++) {
        if (SEQUENCE[motor->currentStep][i] == 1) {
            __HAL_TIM_SET_COMPARE(motor->TIMx[i], motor->TIM_CHANNEL_x[i], motor->duty_value);
            HAL_GPIO_WritePin(motor->M_GPIOx[i], motor->M_GPIO_Pin_x[i],  GPIO_PIN_RESET);
        } else if (SEQUENCE[motor->currentStep][i] == -1) {
            __HAL_TIM_SET_COMPARE(motor->TIMx[i], motor->TIM_CHANNEL_x[i], 0);
            HAL_GPIO_WritePin(motor->M_GPIOx[i], motor->M_GPIO_Pin_x[i],  GPIO_PIN_SET);
        } else {
            __HAL_TIM_SET_COMPARE(motor->TIMx[i], motor->TIM_CHANNEL_x[i], 0);
            HAL_GPIO_WritePin(motor->M_GPIOx[i], motor->M_GPIO_Pin_x[i],  GPIO_PIN_RESET);
        }
    }
}

/**
  * 更新馬達轉速步數並依據霍爾感測器讀值決定下一換相步驟
  *
  * Update motor step count and determine next step from Hall sensor readings
  */
void update_motor_step(MOTOR_PARAMETER *motor) {
    int hallState =
        (HAL_GPIO_ReadPin(motor->Hall_GPIOx[0], motor->Hall_GPIO_Pin_x[0]) << 2) |
        (HAL_GPIO_ReadPin(motor->Hall_GPIOx[1], motor->Hall_GPIO_Pin_x[1]) << 1) |
        (HAL_GPIO_ReadPin(motor->Hall_GPIOx[2], motor->Hall_GPIO_Pin_x[2])     );
    if (motor->rotate_direction == counter_clockwise) {
        switch(hallState) {
            case 2: motor->currentStep = 0; break;
            case 3: motor->currentStep = 1; break;
            case 1: motor->currentStep = 2; break;
            case 5: motor->currentStep = 3; break;
            case 4: motor->currentStep = 4; break;
            case 6: motor->currentStep = 5; break;
        }
    } else if(motor->rotate_direction == clockwise) {
        switch(hallState) {
            case 5: motor->currentStep = 0; break;
            case 4: motor->currentStep = 1; break;
            case 6: motor->currentStep = 2; break;
            case 2: motor->currentStep = 3; break;
            case 3: motor->currentStep = 4; break;
            case 1: motor->currentStep = 5; break;
        }
    }

    commutate_motor(motor);
}

void set_motor_duty(MOTOR_PARAMETER *motor, int16_t value) {
    // 限制PWM最大值&&最小值
    if (value > 100) {
        motor->duty_value = 100;
    } else if(value < 0) {
        motor->duty_value = 0;
    } else {
        motor->duty_value = value;
    }
}

/**
  * 基於霍爾感測與時間計算即時速度
  *
  * Calculate actual speed from Hall counts and delta time
  */
void speed_calculate(MOTOR_PARAMETER *motor) {
    float real_speed = motor->step_count / 6;
    real_speed /= dt;
    motor->present_speed = real_speed;
    motor->step_count = 0;
}
