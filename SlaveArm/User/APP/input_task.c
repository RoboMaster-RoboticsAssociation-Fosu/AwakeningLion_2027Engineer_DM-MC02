/**
 * @file input_task.c
 * @brief 遥控输入 → 本板许可快照；USART10 收发始终由本任务调度。
 * @note 新机械臂的 Keil 调试入口暂时代替旧预设/自定义控制器。
 *       范围仅机械臂目标输入，待新自定义控制器正式接入时移除该调试入口。
 */
#include "input_task.h"
#include "board_link_system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"
#include "rc_system.h"

static InputTask_t g_input_task = {.locked = true};

void InputTask_Run(void)
{
    TickType_t wake_time = xTaskGetTickCount();
    Remote_EdgeCursor_Init(&g_input_task.remote_edge_cursor);

    for (;;)
    {
        uint32_t sw2_edges;
        bool ready;

        RcSystem_Process();
        Remote_EdgeCursor_Poll(&g_input_task.remote_edge_cursor);
        sw2_edges = g_input_task.remote_edge_cursor.switch_edges[REMOTE_SWITCH_SW2];
        ready = (RcSystem_IsReady() != 0U) && !remote_ctrl.rc_lost;
        if (ready && RC_SWITCH_MOVED_TO_UP(sw2_edges))
        {
            g_input_task.locked = false;
        }
        if (RC_SWITCH_MOVED_TO_DOWN(sw2_edges) ||
            (ready && (remote_ctrl.rc.s[REMOTE_SWITCH_SW2] == RC_SW_DOWN)))
        {
            /* 失能优先；重连首帧即使没有边沿，SW2 下位也必须保持失能。 */
            g_input_task.locked = true;
        }
        g_input_task.arm_command.enabled = !g_input_task.locked;
        g_input_task.arm_command.freeze_targets = !ready && !g_input_task.locked;
        ArmTask_Notify(&g_input_task.arm_command);
        BoardLinkSystem_Process(HAL_GetTick());
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
    }
}
