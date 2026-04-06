/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for timestamp_test_ */
osThreadId_t timestamp_test_Handle;
const osThreadAttr_t timestamp_test__attributes = {
  .name = "timestamp_test_",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Music_Task */
osThreadId_t Music_TaskHandle;
const osThreadAttr_t Music_Task_attributes = {
  .name = "Music_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DMControlTask */
osThreadId_t DMControlTaskHandle;
const osThreadAttr_t DMControlTask_attributes = {
  .name = "DMControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Remote_Task */
osThreadId_t Remote_TaskHandle;
const osThreadAttr_t Remote_Task_attributes = {
  .name = "Remote_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Timestamp_fuc(void *argument);
void MusicTask_func(void *argument);
void DMsetOutput(void *argument);
void tele_task(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of timestamp_test_ */
  timestamp_test_Handle = osThreadNew(Timestamp_fuc, NULL, &timestamp_test__attributes);

  /* creation of Music_Task */
  Music_TaskHandle = osThreadNew(MusicTask_func, NULL, &Music_Task_attributes);

  /* creation of DMControlTask */
  DMControlTaskHandle = osThreadNew(DMsetOutput, NULL, &DMControlTask_attributes);

  /* creation of Remote_Task */
  Remote_TaskHandle = osThreadNew(tele_task, NULL, &Remote_Task_attributes);

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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Timestamp_fuc */
/**
* @brief Function implementing the timestamp_test_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Timestamp_fuc */
__weak void Timestamp_fuc(void *argument)
{
  /* USER CODE BEGIN Timestamp_fuc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Timestamp_fuc */
}

/* USER CODE BEGIN Header_MusicTask_func */
/**
* @brief Function implementing the Music_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MusicTask_func */
__weak void MusicTask_func(void *argument)
{
  /* USER CODE BEGIN MusicTask_func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MusicTask_func */
}

/* USER CODE BEGIN Header_DMsetOutput */
/**
* @brief Function implementing the DMControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DMsetOutput */
__weak void DMsetOutput(void *argument)
{
  /* USER CODE BEGIN DMsetOutput */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DMsetOutput */
}

/* USER CODE BEGIN Header_tele_task */
/**
* @brief Function implementing the Remote_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tele_task */
__weak void tele_task(void *argument)
{
  /* USER CODE BEGIN tele_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END tele_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

