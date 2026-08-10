/**
 ******************************************************************************
 * @file    arm_task.c
 * @brief   机械臂 Damiao 装配实例与未完成阶段的安全基线。
 ******************************************************************************
 */

#include "arm_task.h"

#include "cmsis_os2.h"
#include "fdcan.h"

#include <stddef.h>

#define ARM_DAMIAO_ALL_MOTORS_MASK \
    ((uint8_t)((1UL << ARM_DAMIAO_MOTOR_COUNT) - 1UL))
#define ARM_TASK_NOTIFY_ENABLED      (1UL << 0U)
#define ARM_TASK_NOTIFY_DISABLED     (1UL << 1U)
#define ARM_TASK_NOTIFY_MASK         \
    (ARM_TASK_NOTIFY_ENABLED | ARM_TASK_NOTIFY_DISABLED)

ArmTask_t g_arm_task = {
    .damiao_motor = {
        [ARM_DAMIAO_BIG_YAW] =
            DM_MOTOR_CREATE(Mit_mode, DM_J4340, 0x00U, 0x10U),
        [ARM_DAMIAO_PITCH1] =
            DM_MOTOR_CREATE(Mit_mode, DM_J10010L, 0x01U, 0x11U),
        [ARM_DAMIAO_ROLL1] =
            DM_MOTOR_CREATE(Mit_mode, DM_J4340, 0x02U, 0x12U),
        [ARM_DAMIAO_ROLL2] =
            DM_MOTOR_CREATE(Mit_mode, DM_J4310, 0x03U, 0x13U),
        [ARM_DAMIAO_GRIP] =
            DM_MOTOR_CREATE(Mit_mode, DM_J4310, 0x04U, 0x14U),
        [ARM_DAMIAO_PITCH3] =
            DM_MOTOR_CREATE(Mit_mode, DM_J4310, 0x05U, 0x15U),
    },
    .command = {
        .requested_enabled = false,
        .pending_mask = ARM_DAMIAO_ALL_MOTORS_MASK,
        .next_motor_index = 0U,
    },
};

static osThreadId_t g_arm_task_thread = NULL;

void ArmTask_OnCan2Rx(uint32_t identifier,
                      uint8_t *data,
                      uint32_t data_len)
{
    uint32_t index;

    if (data == NULL)
    {
        return;
    }

    for (index = 0U; index < ARM_DAMIAO_MOTOR_COUNT; index++)
    {
        DM_Motor_Info_t *motor = &g_arm_task.damiao_motor[index];

        if (identifier == motor->ID_Set.RxIdentifier)
        {
            DM_Motor_Info_Update(motor, data, data_len);
            return;
        }
    }
}

static void arm_task_step(bool enabled)
{
    uint32_t offset;

    /* TEMPORARY ARM COMMAND BASELINE BEGIN:
     * Reason: Arm control target generation is not implemented yet.
     * Scope: follow the vehicle lock state with Damiao enable/disable commands;
     *        never generate an MIT control target.
     * Remove when: ArmTask target generation is integrated and verified on
     *              the real mechanism.
     */
    if (enabled != g_arm_task.command.requested_enabled)
    {
        g_arm_task.command.requested_enabled = enabled;
        g_arm_task.command.pending_mask = ARM_DAMIAO_ALL_MOTORS_MASK;
        g_arm_task.command.next_motor_index = 0U;
    }

    for (offset = 0U; offset < ARM_DAMIAO_MOTOR_COUNT; offset++)
    {
        uint32_t index =
            (g_arm_task.command.next_motor_index + offset) %
            ARM_DAMIAO_MOTOR_COUNT;
        uint8_t motor_mask = (uint8_t)(1UL << index);
        uint8_t send_result;
        DM_Motor_Info_t *motor;

        if ((g_arm_task.command.pending_mask & motor_mask) == 0U)
        {
            continue;
        }

        motor = &g_arm_task.damiao_motor[index];
        if (enabled)
        {
            send_result = DM_Enable_Motor(&hfdcan2,
                                          (uint16_t)motor->ID_Set.TxIdentifier,
                                          MIT_MODE,
                                          0U);
        }
        else
        {
            send_result = DM_Disable_Motor(&hfdcan2,
                                           (uint16_t)motor->ID_Set.TxIdentifier,
                                           MIT_MODE,
                                           0U);
        }

        g_arm_task.debug.result_valid = true;
        g_arm_task.debug.command_enabled = enabled;
        g_arm_task.debug.motor = (ArmDamiaoMotor_e)index;
        g_arm_task.debug.return_value = send_result;

        if (send_result == 0U)
        {
            g_arm_task.command.pending_mask &= (uint8_t)~motor_mask;
        }
        g_arm_task.command.next_motor_index =
            (uint8_t)((index + 1U) % ARM_DAMIAO_MOTOR_COUNT);
        return;
    }
    /* TEMPORARY ARM COMMAND BASELINE END */
}

void ArmTask_Notify(bool enabled)
{
    if (g_arm_task_thread == NULL)
    {
        return;
    }

    (void)osThreadFlagsSet(
        g_arm_task_thread,
        enabled ? ARM_TASK_NOTIFY_ENABLED : ARM_TASK_NOTIFY_DISABLED);
}

void ArmTask_Run(void)
{
    uint32_t notification;
    bool enabled;

    g_arm_task_thread = osThreadGetId();

    for (;;)
    {
        /* 两个输入周期都未收到通知时按失能处理，禁止沿用旧输出许可。 */
        notification = osThreadFlagsWait(ARM_TASK_NOTIFY_MASK,
                                         osFlagsWaitAny,
                                         2U);
        enabled = ((notification & osFlagsError) == 0U) &&
                  ((notification & ARM_TASK_NOTIFY_DISABLED) == 0U) &&
                  ((notification & ARM_TASK_NOTIFY_ENABLED) != 0U);
        arm_task_step(enabled);
    }
}
