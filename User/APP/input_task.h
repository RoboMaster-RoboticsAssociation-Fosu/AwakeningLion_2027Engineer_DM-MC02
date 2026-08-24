/**
 ******************************************************************************
 * @file    input_task.h
 * @brief   输入系统 RTOS owner 任务接口。
 ******************************************************************************
 */

#ifndef INPUT_TASK_H
#define INPUT_TASK_H

#include "Remote_Control.h"
#include "arm_task.h"

#include <stdbool.h>

/**
 * @brief 输入任务跨周期状态。
 * @note locked 上电静态初始化为 true，只有 SW2 进入 UP/DOWN 的边沿改变它。
 */
typedef struct
{
    RC_EdgeCursor_t remote_edge_cursor;
    ArmTaskCommand_t arm_command;
    bool locked;
} InputTask_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 运行输入任务生命周期。
 * @note 调用者必须是 InputTask 对应的 FreeRTOS 线程；函数正常运行时不返回。
 */
void InputTask_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* INPUT_TASK_H */
