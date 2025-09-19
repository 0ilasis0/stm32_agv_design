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
  .stack_size = 512 * 4
};
/* Definitions for ArmMotorTask */
osThreadId_t ArmMotorTaskHandle;
const osThreadAttr_t ArmMotorTask_attributes = {
  .name = "ArmMotorTask",
  .priority = (osPriority_t) osPriorityRealtime1,
  .stack_size = 128 * 4
};
/* Definitions for FdCanTask */
osThreadId_t FdCanTaskHandle;
const osThreadAttr_t FdCanTask_attributes = {
  .name = "FdCanTask",
  .priority = (osPriority_t) osPriorityHigh1,
  .stack_size = 256 * 4
};
/* Definitions for RfidTask */
osThreadId_t RfidTaskHandle;
const osThreadAttr_t RfidTask_attributes = {
  .name = "RfidTask",
  .priority = (osPriority_t) osPriorityRealtime3,
  .stack_size = 256 * 4
};
/* Definitions for MapTask */
osThreadId_t MapTaskHandle;
const osThreadAttr_t MapTask_attributes = {
  .name = "MapTask",
  .priority = (osPriority_t) osPriorityRealtime2,
  .stack_size = 256 * 4
};
/* Definitions for ArmTask */
osThreadId_t ArmTaskHandle;
const osThreadAttr_t ArmTask_attributes = {
  .name = "ArmTask",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartArmMotorTask(void *argument);
void StartFdCanTask(void *argument);
void StartRfidTask(void *argument);
void StartMapTask(void *argument);
void StartArmTask(void *argument);

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

  /* creation of ArmMotorTask */
  ArmMotorTaskHandle = osThreadNew(StartArmMotorTask, NULL, &ArmMotorTask_attributes);

  /* creation of FdCanTask */
  FdCanTaskHandle = osThreadNew(StartFdCanTask, NULL, &FdCanTask_attributes);

  /* creation of RfidTask */
  RfidTaskHandle = osThreadNew(StartRfidTask, NULL, &RfidTask_attributes);

  /* creation of MapTask */
  MapTaskHandle = osThreadNew(StartMapTask, NULL, &MapTask_attributes);

  /* creation of ArmTask */
  ArmTaskHandle = osThreadNew(StartArmTask, NULL, &ArmTask_attributes);

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

/* USER CODE BEGIN Header_StartArmMotorTask */
/**
* @brief Function implementing the ArmMotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartArmMotorTask */
__weak void StartArmMotorTask(void *argument)
{
  /* USER CODE BEGIN StartArmMotorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartArmMotorTask */
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

/* USER CODE BEGIN Header_StartMapTask */
/**
* @brief Function implementing the MapTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMapTask */
__weak void StartMapTask(void *argument)
{
  /* USER CODE BEGIN StartMapTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMapTask */
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

