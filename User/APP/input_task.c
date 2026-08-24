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
    .arm_command = {
        .mode = ARM_CONTROL_MODE_PRESET,
        .action = ARM_PRESET_ACTION_NONE,
    },
    .locked = true,
};

static void input_task_update_arm_command(void)
{
    uint32_t iw_edges = g_input_task.remote_edge_cursor.iw_edges;
    uint8_t sw1 = remote_ctrl.rc.s[REMOTE_SWITCH_SW1];
    uint8_t sw2 = remote_ctrl.rc.s[REMOTE_SWITCH_SW2];
    bool enter_up = (iw_edges & RC_IW_EDGE_ENTER_UP) != 0U;
    bool enter_down = (iw_edges & RC_IW_EDGE_ENTER_DOWN) != 0U;

    if (sw2 == RC_SW_UP)
    {
        g_input_task.arm_command.mode = ARM_CONTROL_MODE_PRESET;
    }
    /* HOLD 暂不启用，保留原入口，仅注释掉模式切换。 */
    /* else if (sw2 == RC_SW_MID)
    {
        g_input_task.arm_command.mode = ARM_CONTROL_MODE_HOLD;
    } */

    if ((sw2 != RC_SW_UP) ||
        (!enter_up && !enter_down))
    {
        return;
    }

    if (sw1 == RC_SW_DOWN)
    {
        g_input_task.arm_command.action = ARM_PRESET_ACTION_NORMAL;
        g_input_task.arm_command.action_sequence++;
        return;
    }

    if (sw1 == RC_SW_MID)
    {
        g_input_task.arm_command.action = ARM_PRESET_ACTION_CUSTOM_1;
        g_input_task.arm_command.action_sequence++;
        return;
    }

    if (sw1 != RC_SW_UP)
    {
        return;
    }

    if (enter_up && enter_down)
    {
        /* 多个边沿在一次轮询中累积时，以拨轮最终所在方向为准。 */
        if (remote_ctrl.rc.iw < 0)
        {
            g_input_task.arm_command.action = ARM_PRESET_ACTION_WAVE_1;
        }
        else if (remote_ctrl.rc.iw > 0)
        {
            g_input_task.arm_command.action = ARM_PRESET_ACTION_WAVE_2;
        }
        else
        {
            return;
        }
    }
    else
    {
        g_input_task.arm_command.action = enter_up
                                              ? ARM_PRESET_ACTION_WAVE_1
                                              : ARM_PRESET_ACTION_WAVE_2;
    }
    g_input_task.arm_command.action_sequence++;
}

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
            /* 每次重新进入 PRESET 时先清除上一次已执行的动作。 */
            g_input_task.arm_command.action = ARM_PRESET_ACTION_NONE;
            g_input_task.arm_command.action_sequence++;
        }
        if (RC_SWITCH_MOVED_TO_DOWN(sw2_edges))
        {
            /* 同周期累计多个边沿时，锁定优先于解锁。 */
            g_input_task.locked = true;
        }

        input_task_update_arm_command();
        if (g_input_task.locked)
        {
            g_input_task.arm_command.enabled = false;
        }
        else if (RcSystem_IsReady() != 0U)
        {
            g_input_task.arm_command.enabled = true;
        }
        else
        {
            g_input_task.arm_command.enabled = false;
        }
        output_enabled = g_input_task.arm_command.enabled;
        g_input_task.arm_command.enabled = output_enabled;
        ChassisTask_Notify(output_enabled);
        ArmTask_Notify(&g_input_task.arm_command);
        osDelay(1U);
    }
}
