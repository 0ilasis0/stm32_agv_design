#ifndef PI_CONTROL_H
#define PI_CONTROL_H

#include "common.h"
#include <math.h>
#include "motor.h"

#define Kp 1.5                                           // 比例增益
#define Ki 0.8                                           // 積分增益
#define dt 0.5f                                          // 取樣時間
#define PI_feedbacck 0.2                                 // PI feedbacck 增益值

#define setpoint_straight 30                             // 循跡速度恆定目標
#define setpoint_rotate   20                             // 原地旋轉速度恆定目標

extern int setpoint_current;

int PI_Controller(MOTOR_PARAMETER *motor, double measurement);
void PI_timer_setup(void);

#endif
