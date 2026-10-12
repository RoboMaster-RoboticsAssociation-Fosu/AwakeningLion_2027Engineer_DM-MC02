/**
 ******************************************************************************
 * @file    service_task.h
 * @brief   后台服务 RTOS owner 任务接口。
 ******************************************************************************
 */

#ifndef SERVICE_TASK_H
#define SERVICE_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 运行后台服务任务生命周期。
 * @note 调用者必须是 ServiceTask 对应的 FreeRTOS 线程；函数正常运行时不返回。
 */
void ServiceTask_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVICE_TASK_H */
