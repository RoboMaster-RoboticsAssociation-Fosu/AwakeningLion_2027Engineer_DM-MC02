/** @file input_task.h @brief 本板输入及板间快照转发 owner。 */
#ifndef INPUT_TASK_H
#define INPUT_TASK_H

#include "robot_board_config.h"
#include "Remote_Control.h"
#include <stdbool.h>
#if ROBOT_BOARD_ROLE == ROBOT_BOARD_ARM
#include "arm_task.h"
#endif

/** @brief 上电锁定；机械臂直接接收 DBUS，底盘使用串口转发快照。 */
typedef struct
{
    bool locked;
#if ROBOT_BOARD_ROLE == ROBOT_BOARD_ARM
    RC_EdgeCursor_t remote_edge_cursor;
    ArmTaskCommand_t arm_command;
#else
    uint8_t previous_sw2;
    bool switch_history_valid;
#endif
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
