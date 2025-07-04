/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for DefaultTask */
osThreadId_t DefaultTaskHandle;
const osThreadAttr_t DefaultTask_attributes = {
  .name = "DefaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ArmTask */
osThreadId_t ArmTaskHandle;
const osThreadAttr_t ArmTask_attributes = {
  .name = "ArmTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for FdCanTask */
osThreadId_t FdCanTaskHandle;
const osThreadAttr_t FdCanTask_attributes = {
  .name = "FdCanTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};
/* Definitions for RfidTask */
osThreadId_t RfidTaskHandle;
const osThreadAttr_t RfidTask_attributes = {
  .name = "RfidTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartArmTask(void *argument);
void StartFdCanTask(void *argument);
void StartRfidTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of DefaultTask */
  DefaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &DefaultTask_attributes);

  /* creation of ArmTask */
  ArmTaskHandle = osThreadNew(StartArmTask, NULL, &ArmTask_attributes);

  /* creation of FdCanTask */
  FdCanTaskHandle = osThreadNew(StartFdCanTask, NULL, &FdCanTask_attributes);

  /* creation of RfidTask */
  RfidTaskHandle = osThreadNew(StartRfidTask, NULL, &RfidTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartArmTask */
/**
* @brief Function implementing the ArmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartArmTask */
__weak void StartArmTask(void *argument)
{
  /* USER CODE BEGIN StartArmTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartArmTask */
}

/* USER CODE BEGIN Header_StartFdCanTask */
/**
* @brief Function implementing the FdCanTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFdCanTask */
__weak void StartFdCanTask(void *argument)
{
  /* USER CODE BEGIN StartFdCanTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartFdCanTask */
}

/* USER CODE BEGIN Header_StartRfidTask */
/**
* @brief Function implementing the RfidTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRfidTask */
__weak void StartRfidTask(void *argument)
{
  /* USER CODE BEGIN StartRfidTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRfidTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

