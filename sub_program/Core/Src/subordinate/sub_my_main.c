#include "subordinate/sub_my_main.h"



int start = 0, stop = 0;                            // 計時起點與終點
uint32_t previous_time;                             // 用來追蹤非阻塞延遲
float distance = 0.0;                               // 計算出來的距離（單位：公分）

uint8_t rx_data[5] = {0};
uint8_t tx_data[5] = {1,0,0,0,0};

EchoState echo_state = ECHO_WAIT_RISE;              //設定超音波目前狀態(開始 結束 計算)



/* +Main -----------------------------------------------------------------------*/
void user_main(void) {
    HAL_TIM_Base_Start(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    previous_time = HAL_GetTick();                                  //紀錄當前時間

    HAL_UART_Receive_DMA(&huart3, rx_data, sizeof(rx_data));         // 第一次接收rx設定

    while (1) {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            usart3_transmit_action();
            HAL_Delay(300);                                          // 防彈跳
        }

/*        if (HAL_GetTick() - previous_time >= 500) {                // 每 500 毫秒觸發一次
            previous_time = HAL_GetTick();
            HCSR04_Trigger();
        }

         HCSR04_PollEcho();
*/
    }
}

/* -Main -----------------------------------------------------------------------*/



/* +usart ----------------------------------------------------------------------*/
void usart3_receive_action(void){

    HAL_UART_Receive_DMA(&huart3, rx_data, sizeof(rx_data));

}

void usart3_transmit_action(void){
    HAL_UART_Transmit_DMA(&huart3, tx_data, sizeof(tx_data) - 1);   // 排除結尾的 '\0' 字元

}

/* -usart ----------------------------------------------------------------------*/



/* +發出超音波 ------------------------------------------------------------------*/
void HCSR04_Trigger(void) {
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}

/* -發出超音波 ------------------------------------------------------------------*/



/* +ECHO 腳位變化並記錄時間 ------------------------------------------------------*/
void HCSR04_PollEcho(void) {
    switch (echo_state) {
        case ECHO_WAIT_RISE:
            if (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET) {
                start = __HAL_TIM_GET_COUNTER(&htim1);  // Echo 上升緣，記錄開始時間
                echo_state = ECHO_WAIT_FALL;
            }
            break;

        case ECHO_WAIT_FALL:
            if (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET) {
                stop = __HAL_TIM_GET_COUNTER(&htim1);   // Echo 下降緣，記錄結束時間
                echo_state = ECHO_DONE;
            }
            break;

        case ECHO_DONE:
            distance = (stop - start) * 0.0343f / 2.0f; // 計算距離：時間差（μs） * 聲速（0.0343 cm/μs）/ 2
            echo_state = ECHO_WAIT_RISE;
            break;
    }
}

/* -ECHO 腳位變化並記錄時間 ------------------------------------------------------*/




/* +中斷處理函數 ----------------------------------------------------------------*/
void sub_my_main_ItButtonPC13(void){

}

void sub_my_main_ItTim2(void){

}

void sub_my_main_ItTim1(void){

}

/* -中斷處理函數 ----------------------------------------------------------------*/
