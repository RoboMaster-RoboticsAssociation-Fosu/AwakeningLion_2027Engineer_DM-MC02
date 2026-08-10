/**
 ******************************************************************************
 * @file    input_task.c
 * @brief   输入系统 RTOS owner 任务。
 ******************************************************************************
 */

#include "input_task.h"

#include "arm_task.h"
#include "chassis_task.h"
#include "cmsis_os2.h"
#include "rc_system.h"

static InputTask_t g_input_task = {
    .locked = true,
};

void InputTask_Run(void)
{
    Remote_EdgeCursor_Init(&g_input_task.remote_edge_cursor);

    for (;;)
    {
        uint32_t sw2_edges;
        bool output_enabled;

        RcSystem_Process();
        Remote_EdgeCursor_Poll(&g_input_task.remote_edge_cursor);
        sw2_edges = g_input_task.remote_edge_cursor
                        .switch_edges[REMOTE_SWITCH_SW2];

        if (RC_SWITCH_MOVED_TO_UP(sw2_edges))
        {
            g_input_task.locked = false;
        }
        if (RC_SWITCH_MOVED_TO_DOWN(sw2_edges))
        {
            /* 同周期累计多个边沿时，锁定优先于解锁。 */
            g_input_task.locked = true;
        }

        output_enabled = !g_input_task.locked &&
                         (RcSystem_IsReady() != 0U);
        ChassisTask_Notify(output_enabled);
        ArmTask_Notify(output_enabled);
        osDelay(1U);
    }
}
