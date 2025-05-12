#ifndef SUB_MY_MAIN_H
#define SUB_MY_MAIN_H



/* +Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "tim.h"
#include "dma.h"
#include "usart.h"
#include "stm32g4xx_hal_flash.h"

/* -Includes ------------------------------------------------------------------*/



/* +define ---------------------------------------------------------------------*/
#define TRIG_PORT GPIOC         //發送端
#define TRIG_PIN GPIO_PIN_2
#define ECHO_PORT GPIOC         //接收端
#define ECHO_PIN GPIO_PIN_3

/* -define ---------------------------------------------------------------------*/



/* +typedef---------------------------------------------------------------------*/
typedef enum {
    ECHO_WAIT_RISE,                                 // 等待 Echo 拉高
    ECHO_WAIT_FALL,                                 // 等待 Echo 拉低
    ECHO_DONE                                       // Echo 完成一次測距

} EchoState;

/* -typedef---------------------------------------------------------------------*/



/* +subprogram -----------------------------------------------------------------*/
void principal_main(void);
void HCSR04_Trigger(void);
void HCSR04_PollEcho(void);
void usart3_receive_action(void);
void usart3_transmit_action(void);

/* -subprogram ------------------------------------------------------------------*/



/* +bsae_program ----------------------------------------------------------------*/
void principal_main(void);
void sub_my_main_ItButtonPC13(void);
void sub_my_main_ItTim2(void);
void sub_my_main_ItTim1(void);

/* -bsae_program ----------------------------------------------------------------*/



#endif
