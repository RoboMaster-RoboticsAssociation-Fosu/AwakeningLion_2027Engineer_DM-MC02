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
#include "custom_controller_system.h"
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

    if ((sw2 == RC_SW_MID) &&
        (sw1 == RC_SW_UP) &&
        (iw_edges != RC_IW_EDGE_NONE))
    {
        g_input_task.arm_command.mode = ARM_CONTROL_MODE_HOLD;
        return;
    }

    if ((sw2 == RC_SW_MID) &&
        (sw1 == RC_SW_MID) &&
        (iw_edges != RC_IW_EDGE_NONE))
    {
        g_input_task.arm_command.mode = ARM_CONTROL_MODE_CUSTOM;
        return;
    }

    if (sw2 == RC_SW_UP)
    {
        g_input_task.arm_command.mode = ARM_CONTROL_MODE_PRESET;
    }

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
        bool rc_ready;
        bool chassis_enabled;
        bool mode_exit_to_normal = false;

        RcSystem_Process();
        CustomControllerSystem_Process();
        CustomControllerSystem_CopySnapshot(
            &g_input_task.arm_command.custom_controller);
        Remote_EdgeCursor_Poll(&g_input_task.remote_edge_cursor);
        sw2_edges = g_input_task.remote_edge_cursor
                        .switch_edges[REMOTE_SWITCH_SW2];

        if (RC_SWITCH_MOVED_TO_UP(sw2_edges))
        {
            g_input_task.locked = false;
            mode_exit_to_normal =
                (g_input_task.arm_command.mode == ARM_CONTROL_MODE_CUSTOM) ||
                (g_input_task.arm_command.mode == ARM_CONTROL_MODE_HOLD);
            if (mode_exit_to_normal)
            {
                /* CUSTOM/HOLD退出时进入动作模式并执行normal姿态。 */
                g_input_task.arm_command.mode = ARM_CONTROL_MODE_PRESET;
                g_input_task.arm_command.action =
                    ARM_PRESET_ACTION_NORMAL;
            }
            else
            {
                /* 普通解锁不自动执行任何预设动作。 */
                g_input_task.arm_command.action = ARM_PRESET_ACTION_NONE;
            }
            g_input_task.arm_command.action_sequence++;
        }
        if (RC_SWITCH_MOVED_TO_DOWN(sw2_edges))
        {
            /* 同周期累计多个边沿时，锁定优先于解锁。 */
            g_input_task.locked = true;
        }

        rc_ready = RcSystem_IsReady() != 0U;
        if (rc_ready && !mode_exit_to_normal)
        {
            input_task_update_arm_command();
        }
        if (g_input_task.locked)
        {
            g_input_task.arm_command.enabled = false;
            g_input_task.arm_command.freeze_targets = false;
            chassis_enabled = false;
        }
        else if (rc_ready)
        {
            g_input_task.arm_command.enabled = true;
            g_input_task.arm_command.freeze_targets = false;
            chassis_enabled = true;
        }
        else
        {
            /* RC掉线只停止底盘；机械臂继续使能并冻结当前已应用角度。 */
            g_input_task.arm_command.enabled = true;
            g_input_task.arm_command.freeze_targets = true;
            chassis_enabled = false;
        }
        ChassisTask_Notify(chassis_enabled);
        ArmTask_Notify(&g_input_task.arm_command);
        osDelay(1U);
    }
}
