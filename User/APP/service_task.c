/**
 ******************************************************************************
 * @file    service_task.c
 * @brief   后台服务 RTOS owner 任务。
 ******************************************************************************
 */

#include "service_task.h"

#include "cmsis_os2.h"
#include "led_system.h"
#include "stm32h7xx_hal.h"
#include "vofa_system.h"

#define SERVICE_TASK_PERIOD_MS 10U

void ServiceTask_Run(void)
{
    for (;;)
    {
        LedSystem_Step(HAL_GetTick());
        VofaSystem_Step();

        osDelay(SERVICE_TASK_PERIOD_MS);
    }
}
