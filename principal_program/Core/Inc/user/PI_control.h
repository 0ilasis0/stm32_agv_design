#ifndef USER_PI_CONTROL_H
#define USER_PI_CONTROL_H

#include "user/motor.h"

#define Kp 0.5f                                                // 比例增益
#define Ki 2.5f                                               // 積分增益

#define setpoint_straight  40                                  // 循跡速度目標
#define setpoint_rotate    30                                  // 原地旋轉速度目標
#define setpoint_fall_back 20                                  // 倒退速度目標

extern float max_speed_pcn;

/*測試用--------------------------------------*/
extern uint32_t hall_sensor_node;
/*測試用--------------------------------------*/

void PI_Controller(MOTOR_PARAMETER *motor);
void PI_tim_setup(void);

#endif
