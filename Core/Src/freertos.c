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
#include "robot_board_config.h"
#if ROBOT_BOARD_ROLE == ROBOT_BOARD_ARM
#include "arm_task.h"
#else
#include "chassis_task.h"
#include "ins_task.h"
#endif
#include "input_task.h"
#include "service_task.h"
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
/* 板型专属任务由用户代码持有，CubeMX 仅生成公共任务，重生成时保留本区域。 */
#if ROBOT_BOARD_ROLE == ROBOT_BOARD_CHASSIS
/* Definitions for INS_TASK */
osThreadId_t INS_TASKHandle;
const osThreadAttr_t INS_TASK_attributes = {
  .name = "INS_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Definitions for chassisTask */
osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
#else
/* Definitions for armTask */
osThreadId_t armTaskHandle;
const osThreadAttr_t armTask_attributes = {
  .name = "armTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
#endif
/* USER CODE END Variables */
/* Definitions for serviceTask */
osThreadId_t serviceTaskHandle;
const osThreadAttr_t serviceTask_attributes = {
  .name = "serviceTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for inputTask */
osThreadId_t inputTaskHandle;
const osThreadAttr_t inputTask_attributes = {
  .name = "inputTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
#if ROBOT_BOARD_ROLE == ROBOT_BOARD_CHASSIS
void INS_Task(void *argument);
void StartChassisTask(void *argument);
#else
void StartArmTask(void *argument);
#endif
/* USER CODE END FunctionPrototypes */

void StartServiceTask(void *argument);
void StartInputTask(void *argument);

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
  /* creation of serviceTask */
  serviceTaskHandle = osThreadNew(StartServiceTask, NULL, &serviceTask_attributes);

  /* creation of inputTask */
  inputTaskHandle = osThreadNew(StartInputTask, NULL, &inputTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 板型专属任务只在当前板创建；声明、属性和入口均位于 CubeMX 保留区域。 */
#if ROBOT_BOARD_ROLE == ROBOT_BOARD_CHASSIS
  INS_TASKHandle = osThreadNew(INS_Task, NULL, &INS_TASK_attributes);
  chassisTaskHandle = osThreadNew(StartChassisTask, NULL, &chassisTask_attributes);
#else
  armTaskHandle = osThreadNew(StartArmTask, NULL, &armTask_attributes);
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartServiceTask */
/**
  * @brief  Function implementing the serviceTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartServiceTask */
void StartServiceTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartServiceTask */
  (void)argument;
  ServiceTask_Run();
  osThreadExit();
  /* USER CODE END StartServiceTask */
}

/* USER CODE BEGIN Header_StartInputTask */
/**
* @brief Function implementing the inputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInputTask */
void StartInputTask(void *argument)
{
  /* USER CODE BEGIN StartInputTask */
  (void)argument;
  InputTask_Run();
  osThreadExit();
  /* USER CODE END StartInputTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
#if ROBOT_BOARD_ROLE == ROBOT_BOARD_CHASSIS
/**
 * @brief 底盘板 INS 任务入口，将 CMSIS 线程参数转换为现有运行入口。
 * @param argument 未使用；运行入口返回时结束本线程。
 */
void INS_Task(void *argument)
{
  (void)argument;
  InsTask_Run();
  osThreadExit();
}

/**
 * @brief 底盘板控制任务入口；本函数整体位于 CubeMX 保留区域。
 * @param argument 未使用；运行入口返回时结束本线程。
 */
void StartChassisTask(void *argument)
{
  (void)argument;
  ChassisTask_Run();
  osThreadExit();
}
#else
/**
 * @brief 机械臂板控制任务入口；本函数整体位于 CubeMX 保留区域。
 * @param argument 未使用；运行入口返回时结束本线程。
 */
void StartArmTask(void *argument)
{
  (void)argument;
  ArmTask_Run();
  osThreadExit();
}
#endif
/* USER CODE END Application */

