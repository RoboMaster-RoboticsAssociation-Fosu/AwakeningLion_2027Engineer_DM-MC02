/** @file input_task.h @brief 本板输入及板间快照转发 owner。 */
#ifndef INPUT_TASK_H
#define INPUT_TASK_H

#include "Remote_Control.h"
#include <stdbool.h>
#include "arm_task.h"

/** @brief 上电锁定；从臂直接接收 DBUS。 */
typedef struct
{
    bool locked;
    RC_EdgeCursor_t remote_edge_cursor;
    ArmTaskCommand_t arm_command;
} InputTask_t;

#ifdef __cplusplus
extern "C" {
#endif
/** @brief 1 ms 输入生命周期，包含板间接收解析和 5 ms 快照发送。 */
void InputTask_Run(void);
#ifdef __cplusplus
}
#endif
#endif /* INPUT_TASK_H */
