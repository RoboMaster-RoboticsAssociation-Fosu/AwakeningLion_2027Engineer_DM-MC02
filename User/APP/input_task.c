/**
 * @file input_task.c
 * @brief 遥控输入 → 本板许可快照；USART10 收发始终由本任务调度。
 * @note 新机械臂的 Keil 调试入口暂时代替旧预设/自定义控制器。
 *       范围仅机械臂目标输入，待新自定义控制器正式接入时移除该调试入口。
 */
#include "input_task.h"
#include "board_link.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"
#if ROBOT_BOARD_ROLE == ROBOT_BOARD_ARM
#include "rc_system.h"
#else
#include "chassis_task.h"
#endif

static InputTask_t g_input_task = {.locked = true};

void InputTask_Run(void)
{
    TickType_t wake_time = xTaskGetTickCount();
#if ROBOT_BOARD_ROLE == ROBOT_BOARD_ARM
    Remote_EdgeCursor_Init(&g_input_task.remote_edge_cursor);
#endif

    for (;;)
    {
#if ROBOT_BOARD_ROLE == ROBOT_BOARD_ARM
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
        BoardLink_Process(HAL_GetTick());
#else
        uint8_t sw2;
        bool ready;

        BoardLink_Process(HAL_GetTick());
        ready = g_board_link.remote_valid && !remote_ctrl.rc_lost;
        sw2 = remote_ctrl.rc.s[REMOTE_SWITCH_SW2];
        if (ready)
        {
            if (g_input_task.switch_history_valid &&
                (sw2 != g_input_task.previous_sw2))
            {
                if (sw2 == RC_SW_UP) { g_input_task.locked = false; }
                if (sw2 == RC_SW_DOWN) { g_input_task.locked = true; }
            }
            g_input_task.previous_sw2 = sw2;
            g_input_task.switch_history_valid = true;
        }
        /* 沿用已有锁定/rc_lost 门控；板间 50 ms 超时和重连策略留待后续确认。 */
        ChassisTask_Notify(ready && !g_input_task.locked);
#endif
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
    }
}
