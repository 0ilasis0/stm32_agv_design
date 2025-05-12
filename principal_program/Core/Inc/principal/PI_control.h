#ifndef PRINCIPAL_PI_CONTROL_H
#define PRINCIPAL_PI_CONTROL_H

#include "principal/motor.h"

#define Kp 1.5                                           // 比例增益
#define Ki 0.8                                           // 積分增益
#define dt 0.5f                                          // 取樣時間
#define PI_feedbacck 0.2                                 // PI feedbacck 增益值

#define setpoint_straight 30                             // 循跡速度目標
#define setpoint_rotate   20                             // 原地旋轉速度目標
#define setpoint_fall_back 10                            // 倒退速度目標

extern int setpoint_current;

void PI_Controller(MOTOR_PARAMETER *motor, double measurement);
void PI_tim_setup(void);

#endif
