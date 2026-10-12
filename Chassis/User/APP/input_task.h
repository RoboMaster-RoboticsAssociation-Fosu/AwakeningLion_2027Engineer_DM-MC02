/** @file input_task.h @brief 本板输入及板间快照转发 owner。 */
#ifndef INPUT_TASK_H
#define INPUT_TASK_H

#include "Remote_Control.h"
#include <stdbool.h>

/** @brief 上电锁定；底盘使用 USART10 转发的遥控快照。 */
typedef struct
{
    bool locked;
    uint8_t previous_sw2;
    bool switch_history_valid;
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
