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
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "INS_task.h"
#include "chassis_task.h"
#include "rc_task.h"
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
/* Definitions for INS_TASK */
osThreadId_t INS_TASKHandle;
const osThreadAttr_t INS_TASK_attributes = {
  .name = "INS_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for CHASSIS_TASK */
osThreadId_t CHASSIS_TASKHandle;
const osThreadAttr_t CHASSIS_TASK_attributes = {
  .name = "CHASSIS_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for rcTask */
osThreadId_t rcTaskHandle;
const osThreadAttr_t rcTask_attributes = {
  .name = "rcTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void INS_Task(void *argument);
void CHASSIS_Task(void *argument);
void StartRcTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
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

  /* creation of INS_TASK */
  INS_TASKHandle = osThreadNew(INS_Task, NULL, &INS_TASK_attributes);

  /* creation of CHASSIS_TASK */
  CHASSIS_TASKHandle = osThreadNew(CHASSIS_Task, NULL, &CHASSIS_TASK_attributes);

  /* creation of rcTask */
  rcTaskHandle = osThreadNew(StartRcTask, NULL, &rcTask_attributes);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_INS_Task */
/**
* @brief Function implementing the INS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_Task */
void INS_Task(void *argument)
{
  /* USER CODE BEGIN INS_Task */
  (void)argument;

  /* Infinite loop */
  for(;;)
  {
    INS_task();
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_CHASSIS_Task */
/**
* @brief Function implementing the CHASSIS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CHASSIS_Task */
void CHASSIS_Task(void *argument)
{
  /* USER CODE BEGIN CHASSIS_Task */
  (void)argument;
  chassis_task();

  /* chassis_task() only returns when initialization fails. */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END CHASSIS_Task */
}

/* USER CODE BEGIN Header_StartRcTask */
/**
* @brief Function implementing the rcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRcTask */
void StartRcTask(void *argument)
{
  /* USER CODE BEGIN StartRcTask */
  (void)argument;
  (void)RC_Task_Init();

  /* Infinite loop */
  for(;;)
  {
    RC_Task_Process();
    osDelay(50);
  }
  /* USER CODE END StartRcTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

