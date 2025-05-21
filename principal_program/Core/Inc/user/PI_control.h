#ifndef USER_PI_CONTROL_H
#define USER_PI_CONTROL_H

#include "user/motor.h"

#define Kp 1.5f                                                // 比例增益
#define Ki 0.8f                                                // 積分增益
#define PI_feedbacck 0.2                                       // PI feedbacck 增益值

#define setpoint_straight  30                                  // 循跡速度目標
#define setpoint_rotate    20                                  // 原地旋轉速度目標
#define setpoint_fall_back 10                                  // 倒退速度目標

extern uint16_t max_speed;
extern float dt;                                               // 取樣時間

/*測試用--------------------------------------*/
extern uint32_t hall_sensor3;
/*測試用--------------------------------------*/

void PI_Controller(MOTOR_PARAMETER *motor);
void PI_tim_setup(void);

#endif
