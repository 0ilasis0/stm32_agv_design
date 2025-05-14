#include "principal/motor.h"
#include "tim.h"

const int SEQUENCE[6][3] = {                             // Commutation right_SEQUENCE for 120 degree control
  { 1, -1,  0},
  { 1,  0, -1},
  { 0,  1, -1},
  {-1,  1,  0},
  {-1,  0,  1},
  { 0, -1,  1}
};

uint8_t max_speed = 0;

/* +struct right or left motor --------------------------------------*/
MOTOR_PARAMETER motor_right;
MOTOR_PARAMETER motor_left;
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

    GPIO_TypeDef* M_GPIOx[6],
    uint16_t M_GPIO_Pin_x[6],

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

    for (int i = 0; i < 6; i++) {
        motor.M_GPIOx[i] = M_GPIOx[i];
        motor.M_GPIO_Pin_x[i] = M_GPIO_Pin_x[i];
    }

    return motor;
}

void motor_init(void) {
    // RIGHT MOTOR pins
    GPIO_TypeDef* Hall_GPIOx_right[3] = {GPIOC, GPIOC, GPIOC};
    uint16_t Hall_GPIO_Pin_x_right[3] = {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3 };

    GPIO_TypeDef* M_GPIOx_right[6] = {GPIOA, GPIOA, GPIOB, GPIOB, GPIOB, GPIOB};
    uint16_t M_GPIO_Pin_x_right[6] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};

    TIM_HandleTypeDef* TIMx_right[3] = {&htim2, &htim2, &htim2};
    uint32_t TIM_CHANNEL_x_right[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};

    motor_right = motor_new(
        30,                                     //speed_sepoint  圈/s
        clockwise,                              //clockwise counter_clockwise
        0.0,                                    //integral_record   PI積分累積儲存
        0,                                      //rpm_count  計數motor speed
        0,                                      //adc_value
        15,                                     //duty_value ; Current PWM value (adjustable)
        0,                                      //present_speed
        7,                                      //currentStep

        Hall_GPIOx_right,                       //Hall sensor pins
        Hall_GPIO_Pin_x_right,

        M_GPIOx_right,                          //right Pin definitions for the three phases
        M_GPIO_Pin_x_right,
        //AH BH CH AL BL CL

        TIMx_right,
        TIM_CHANNEL_x_right
    );

    // LEFT MOTOR pins
    GPIO_TypeDef* Hall_GPIOx_left[3] = {GPIOC, GPIOC, GPIOC};
    uint16_t Hall_GPIO_Pin_x_left[3] = {GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_8};

    GPIO_TypeDef* M_GPIOx_left[6] = {GPIOA, GPIOA, GPIOB, GPIOC, GPIOC, GPIOC};
    uint16_t M_GPIO_Pin_x_left[6] = {GPIO_PIN_6, GPIO_PIN_4, GPIO_PIN_0, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12};

    TIM_HandleTypeDef* TIMx_left[3] = {&htim3, &htim3, &htim3};
    uint32_t TIM_CHANNEL_x_left[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};

    motor_left = motor_new(
        30,                                     //speed_sepoint
        counter_clockwise,                      //clockwise counter_clockwise
        0.0,                                    //integral_record   PI積分累積儲存
        0,                                      //rpm_count  計數motor speed
        0,                                      //adc_value
        15,                                     //duty_value ; Current PWM value (adjustable)
        0,                                      //present_speed
        7,                                      //currentStep

        Hall_GPIOx_left,                        //Hall sensor pins
        Hall_GPIO_Pin_x_left,

        //left Pin definitions for the three phases
        M_GPIOx_left,
        M_GPIO_Pin_x_left,
        //AH BH CH AL BL CL

        TIMx_left,
        TIM_CHANNEL_x_left
    );
}


// +setup -----------------------------------------------------------*/
void motor_tim_setup(const MOTOR_PARAMETER *motor) {
    HAL_TIM_PWM_Start(motor->TIMx[0], motor->TIM_CHANNEL_x[0]);
    HAL_TIM_PWM_Start(motor->TIMx[1], motor->TIM_CHANNEL_x[1]);
    HAL_TIM_PWM_Start(motor->TIMx[2], motor->TIM_CHANNEL_x[2]);
}

/* +renew motor speed and hall signal -------------------------------*/
void commutate_motor(const MOTOR_PARAMETER *motor) {
    for (int i = 0; i < 3; i++) {
        if (SEQUENCE[motor->currentStep][i] == 1) {
            __HAL_TIM_SET_COMPARE(motor->TIMx[i], motor->TIM_CHANNEL_x[i], motor->duty_value);
            HAL_GPIO_WritePin(motor->M_GPIOx[i + 3], motor->M_GPIO_Pin_x[i + 3],  GPIO_PIN_RESET);
        } else if (SEQUENCE[motor->currentStep][i] == -1) {
            __HAL_TIM_SET_COMPARE(motor->TIMx[i], motor->TIM_CHANNEL_x[i], 0);
            HAL_GPIO_WritePin(motor->M_GPIOx[i + 3], motor->M_GPIO_Pin_x[i + 3],  GPIO_PIN_SET);
        } else {
            __HAL_TIM_SET_COMPARE(motor->TIMx[i], motor->TIM_CHANNEL_x[i], 0);
            HAL_GPIO_WritePin(motor->M_GPIOx[i + 3], motor->M_GPIO_Pin_x[i + 3],  GPIO_PIN_RESET);
        }
    }
}

/* +hall senser and decide clockwise --------------------------------*/
void update_motor_step(MOTOR_PARAMETER *motor) {
    motor->step_count++;
    int hallState =
        (HAL_GPIO_ReadPin(motor->Hall_GPIOx[0], motor->Hall_GPIO_Pin_x[0]) << 2) |
        (HAL_GPIO_ReadPin(motor->Hall_GPIOx[1], motor->Hall_GPIO_Pin_x[1]) << 1) |
        (HAL_GPIO_ReadPin(motor->Hall_GPIOx[2], motor->Hall_GPIO_Pin_x[2])     );
    if (motor->rotate_direction == counter_clockwise) {          //逆
        switch(hallState) {
            case 2: motor->currentStep = 0; break;
            case 3: motor->currentStep = 1; break;
            case 1: motor->currentStep = 2; break;
            case 5: motor->currentStep = 3; break;
            case 4: motor->currentStep = 4; break;
            case 6: motor->currentStep = 5; break;
        }
    } else if(motor->rotate_direction == clockwise) {           //順
        switch(hallState) {
            case 5: motor->currentStep = 0; break;
            case 4: motor->currentStep = 1; break;
            case 6: motor->currentStep = 2; break;
            case 2: motor->currentStep = 3; break;
            case 3: motor->currentStep = 4; break;
            case 1: motor->currentStep = 5; break;
        }
    }
}

