/**
 ******************************************************************************
 * @file    arm_task.c
 * @brief   机械臂 Damiao 装配实例与未完成阶段的安全基线。
 ******************************************************************************
 */

#include "arm_task.h"

#include "FreeRTOS.h"
#include "bsp_dwt.h"
#include "cmsis_os2.h"
#include "bsp_uart.h"
#include "fdcan.h"
#include "led_system.h"
#include "task.h"
#include "usart.h"

#include <math.h>
#include <stddef.h>

#define ARM_TASK_NOTIFY_ENABLED      (1UL << 0U)
#define ARM_TASK_NOTIFY_DISABLED     (1UL << 1U)
#define ARM_TASK_NOTIFY_MASK         \
    (ARM_TASK_NOTIFY_ENABLED | ARM_TASK_NOTIFY_DISABLED)
#define ARM_PI_RAD                   3.14159265358979323846f
#define ARM_TWO_PI_RAD               (2.0f * ARM_PI_RAD)
#define ARM_DJI_ENCODER_COUNTS_PER_TURN 8192.0f
#define ARM_DJI_FEEDBACK_TIMEOUT_MS      50U
#define ARM_UNITREE_FEEDBACK_TIMEOUT_MS  20U
#define ARM_DAMIAO_RESPONSE_TIMEOUT_MS   10U
#define ARM_DAMIAO_RETRY_INTERVAL_MS     50U
#define ARM_DAMIAO_MAX_RETRIES           5U
#define ARM_DAMIAO_MAX_ATTEMPTS          (1U + ARM_DAMIAO_MAX_RETRIES)
/* roll1 硬件尚未实装：保留实例和映射，但不得发送命令或触发失联故障。 */
#define ARM_DAMIAO_INSTALLED_MASK                                      \
    (((1UL << ARM_DAMIAO_MOTOR_COUNT) - 1UL) &                         \
     ~(1UL << ARM_DAMIAO_ROLL1))
#define ARM_JOINT_INSTALLED_MASK                                       \
    (((1UL << ARM_JOINT_COUNT) - 1UL) & ~(1UL << ARM_JOINT_ROLL1))

typedef struct
{
    float direction;
    float zero_rad;
    bool wrap_to_pi;
} ArmJointMapping_t;

/* 固定机构映射：驱动反馈先统一为输出轴 rad，再应用方向与零点。
 * Damiao 反馈为输出轴 rad；GM6020 使用单圈编码器转 rad；GO8010 驱动
 * 已应用 6.33:1 减速比，因此这里不能再次除以 UNITREE_8010_RATIO。
 */
static const ArmJointMapping_t g_arm_joint_mapping[ARM_JOINT_COUNT] = {
    [ARM_JOINT_BIG_YAW] = {1.0f, 0.0f, false},
    [ARM_JOINT_PITCH1] = {-1.0f, 0.0f, false},
    [ARM_JOINT_ROLL1] = {1.0f, 0.0f, false},
    [ARM_JOINT_PITCH2] = {-1.0f, 0.0f, false},
    [ARM_JOINT_ROLL2] = {1.0f, 0.0f, false},
    [ARM_JOINT_PITCH3] = {1.0f, 0.1f, false},
    [ARM_JOINT_ROLL3] = {1.0f, 3.7306414f, true},
    [ARM_JOINT_GRIP] = {1.0f, 1.8f, false},
};

static const ArmJoint_e
    g_arm_damiao_joint_map[ARM_DAMIAO_MOTOR_COUNT] = {
        [ARM_DAMIAO_BIG_YAW] = ARM_JOINT_BIG_YAW,
        [ARM_DAMIAO_PITCH1] = ARM_JOINT_PITCH1,
        [ARM_DAMIAO_ROLL1] = ARM_JOINT_ROLL1,
        [ARM_DAMIAO_ROLL2] = ARM_JOINT_ROLL2,
        [ARM_DAMIAO_GRIP] = ARM_JOINT_GRIP,
        [ARM_DAMIAO_PITCH3] = ARM_JOINT_PITCH3,
    };

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
    .roll3_motor = {
        .Motor_Type = DJI_GM6020,
        .ID_Set = {
            .TxIdentifier = DJI_GM6020_CURRENT_GROUP5_7_TX_ID,
            .RxIdentifier = DJI_GM6020_MOTOR5_RX_ID,
        },
    },
    .pitch2_motor =
        UNITREE_MOTOR_CREATE(UNITREE_GO8010, 1U, 1U),
};

static osThreadId_t g_arm_task_thread = NULL;
static uint8_t g_next_damiao_motor_index = 0U;
static uint32_t g_arm_task_start_time_ms = 0U;

static bool arm_task_time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static bool arm_task_feedback_is_fresh(bool feedback_valid,
                                       uint32_t last_feedback_time_ms,
                                       uint32_t now_ms,
                                       uint32_t timeout_ms)
{
    return feedback_valid &&
           ((uint32_t)(now_ms - last_feedback_time_ms) <= timeout_ms);
}

static bool arm_task_damiao_state_matches_request(
    const DM_Motor_Info_t *motor,
    DM_Motor_State_e feedback_state)
{
    if (motor->Command.requested_enabled)
    {
        return feedback_state == DM_MOTOR_STATE_ENABLED;
    }

    return feedback_state == DM_MOTOR_STATE_DISABLED;
}

static ArmMotorLinkState_e arm_task_damiao_confirmed_state(
    const DM_Motor_Info_t *motor)
{
    return motor->Command.requested_enabled
               ? ARM_MOTOR_LINK_DM_ENABLED_CONFIRMED
               : ARM_MOTOR_LINK_DM_DISABLED_CONFIRMED;
}

static bool arm_task_damiao_feedback_is_confirmed(
    ArmMotorLinkState_e link_state)
{
    return (link_state == ARM_MOTOR_LINK_DM_DISABLED_CONFIRMED) ||
           (link_state == ARM_MOTOR_LINK_DM_ENABLED_CONFIRMED) ||
           (link_state == ARM_MOTOR_LINK_ONLINE);
}

static void arm_task_update_motor_link_state(uint32_t now_ms)
{
    uint32_t damiao_feedback_sequence[ARM_DAMIAO_MOTOR_COUNT];
    DM_Motor_State_e damiao_feedback_state[ARM_DAMIAO_MOTOR_COUNT];
    bool roll3_feedback_valid;
    uint32_t roll3_feedback_time_ms;
    bool pitch2_feedback_valid;
    uint32_t pitch2_feedback_time_ms;
    uint32_t index;

    /* 反馈由中断更新；一次性复制判定证据，避免混用不同帧的字段。 */
    taskENTER_CRITICAL();
    for (index = 0U; index < ARM_DAMIAO_MOTOR_COUNT; index++)
    {
        const DM_Motor_Info_t *motor = &g_arm_task.damiao_motor[index];

        damiao_feedback_sequence[index] = motor->feedback_sequence;
        damiao_feedback_state[index] = motor->Data.state;
    }
    roll3_feedback_valid = g_arm_task.roll3_motor.feedback_valid;
    roll3_feedback_time_ms =
        g_arm_task.roll3_motor.last_feedback_time_ms;
    pitch2_feedback_valid =
        g_arm_task.pitch2_motor.Data.feedback_valid;
    pitch2_feedback_time_ms =
        g_arm_task.pitch2_motor.Data.last_feedback_time_ms;
    taskEXIT_CRITICAL();

    for (index = 0U; index < ARM_DAMIAO_MOTOR_COUNT; index++)
    {
        DM_Motor_Info_t *motor = &g_arm_task.damiao_motor[index];
        ArmMotorLinkState_e *link_state =
            &g_arm_task.motor_link_state[g_arm_damiao_joint_map[index]];
        bool new_feedback =
            damiao_feedback_sequence[index] !=
            motor->Command.request_feedback_sequence;
        bool state_matches = arm_task_damiao_state_matches_request(
            motor,
            damiao_feedback_state[index]);

        if ((ARM_DAMIAO_INSTALLED_MASK & (1UL << index)) == 0U)
        {
            /* 未实装槽位不产生通信命令，也不属于在线健康检查对象。 */
            motor->Command.pending = false;
            motor->Command.awaiting_response = false;
            motor->Command.failed = false;
            *link_state = ARM_MOTOR_LINK_UNKNOWN;
            continue;
        }

        if (motor->Command.awaiting_response)
        {
            if (new_feedback)
            {
                /* 统一反馈帧没有独立 ACK：消费新帧后必须用状态码确认
                 * 使能(1)或失能(0)，不能把任意反馈都当成命令成功。
                 */
                motor->Command.request_feedback_sequence =
                    damiao_feedback_sequence[index];
                if (state_matches)
                {
                    motor->Command.awaiting_response = false;
                    motor->Command.pending = false;
                    motor->Command.failed = false;
                    *link_state = arm_task_damiao_confirmed_state(motor);
                }
                else if (motor->Command.attempt_count >=
                         ARM_DAMIAO_MAX_ATTEMPTS)
                {
                    motor->Command.awaiting_response = false;
                    motor->Command.pending = false;
                    motor->Command.failed = true;
                    *link_state = ARM_MOTOR_LINK_DM_COMMAND_FAILED;
                }
                else
                {
                    motor->Command.pending = true;
                    *link_state = ARM_MOTOR_LINK_UNKNOWN;
                }
            }
            else if ((uint32_t)(now_ms - motor->Command.request_time_ms) >
                     ARM_DAMIAO_RESPONSE_TIMEOUT_MS)
            {
                if (motor->Command.attempt_count >=
                    ARM_DAMIAO_MAX_ATTEMPTS)
                {
                    motor->Command.awaiting_response = false;
                    motor->Command.pending = false;
                    motor->Command.failed = true;
                    *link_state = ARM_MOTOR_LINK_DM_COMMAND_FAILED;
                }
                else
                {
                    motor->Command.pending = true;
                    *link_state = ARM_MOTOR_LINK_UNKNOWN;
                }
            }
            else
            {
                *link_state = ARM_MOTOR_LINK_UNKNOWN;
            }
        }
        else if (motor->Command.failed)
        {
            /* 最后一次请求的迟到反馈只有状态匹配时才能解除异常。 */
            if (new_feedback)
            {
                motor->Command.request_feedback_sequence =
                    damiao_feedback_sequence[index];
                if (state_matches)
                {
                    motor->Command.failed = false;
                    motor->Command.pending = false;
                    *link_state = arm_task_damiao_confirmed_state(motor);
                }
                else
                {
                    *link_state = ARM_MOTOR_LINK_DM_COMMAND_FAILED;
                }
            }
            else
            {
                *link_state = ARM_MOTOR_LINK_DM_COMMAND_FAILED;
            }
        }
        else
        {
            /* Damiao 一发一收；成功确认后没有持续反馈证据，保持最近一次
             * 命令确认状态，不能用时间推导 ONLINE 或 OFFLINE。
             */
        }
    }

    if (!roll3_feedback_valid)
    {
        g_arm_task.motor_link_state[ARM_JOINT_ROLL3] =
            ((uint32_t)(now_ms - g_arm_task_start_time_ms) >
             ARM_DJI_FEEDBACK_TIMEOUT_MS)
                ? ARM_MOTOR_LINK_OFFLINE
                : ARM_MOTOR_LINK_UNKNOWN;
    }
    else if (arm_task_feedback_is_fresh(
                 true,
                 roll3_feedback_time_ms,
                 now_ms,
                 ARM_DJI_FEEDBACK_TIMEOUT_MS))
    {
        g_arm_task.motor_link_state[ARM_JOINT_ROLL3] =
            ARM_MOTOR_LINK_ONLINE;
    }
    else
    {
        g_arm_task.motor_link_state[ARM_JOINT_ROLL3] =
            ARM_MOTOR_LINK_OFFLINE;
    }

    if (!pitch2_feedback_valid)
    {
        g_arm_task.motor_link_state[ARM_JOINT_PITCH2] =
            ((uint32_t)(now_ms - g_arm_task_start_time_ms) >
             ARM_UNITREE_FEEDBACK_TIMEOUT_MS)
                ? ARM_MOTOR_LINK_OFFLINE
                : ARM_MOTOR_LINK_UNKNOWN;
    }
    else if (arm_task_feedback_is_fresh(
                 true,
                 pitch2_feedback_time_ms,
                 now_ms,
                 ARM_UNITREE_FEEDBACK_TIMEOUT_MS))
    {
        g_arm_task.motor_link_state[ARM_JOINT_PITCH2] =
            ARM_MOTOR_LINK_ONLINE;
    }
    else
    {
        g_arm_task.motor_link_state[ARM_JOINT_PITCH2] =
            ARM_MOTOR_LINK_OFFLINE;
    }
}

static void arm_task_update_error(void)
{
    uint16_t dm_command_failed_mask = 0U;
    uint16_t motor_lost_mask = 0U;
    uint16_t motor_mask = 0U;
    uint32_t index;
    uint32_t joint;

    for (index = 0U; index < ARM_DAMIAO_MOTOR_COUNT; index++)
    {
        if (((ARM_DAMIAO_INSTALLED_MASK & (1UL << index)) != 0U) &&
            g_arm_task.damiao_motor[index].Command.failed)
        {
            dm_command_failed_mask |=
                (uint16_t)(1UL << g_arm_damiao_joint_map[index]);
        }
    }

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        if (((ARM_JOINT_INSTALLED_MASK & (1UL << joint)) != 0U) &&
            (g_arm_task.motor_link_state[joint] == ARM_MOTOR_LINK_OFFLINE))
        {
            motor_lost_mask |= (uint16_t)(1UL << joint);
        }
    }

    motor_mask = dm_command_failed_mask | motor_lost_mask;
    g_arm_task.error.motor_mask = motor_mask;
    if (dm_command_failed_mask != 0U)
    {
        g_arm_task.error.type = ARM_TASK_ERROR_DM_COMMAND_FAILED;
    }
    else if (motor_lost_mask != 0U)
    {
        g_arm_task.error.type = ARM_TASK_ERROR_MOTOR_LOST;
    }
    else
    {
        g_arm_task.error.type = ARM_TASK_ERROR_NONE;
    }
}

static float arm_task_wrap_to_pi(float angle_rad)
{
    angle_rad = fmodf(angle_rad + ARM_PI_RAD, ARM_TWO_PI_RAD);
    if (angle_rad < 0.0f)
    {
        angle_rad += ARM_TWO_PI_RAD;
    }
    return angle_rad - ARM_PI_RAD;
}

static void arm_task_store_joint_feedback(ArmJoint_e joint,
                                          float output_angle_rad,
                                          bool valid)
{
    const ArmJointMapping_t *mapping;
    float joint_angle_rad;

    if (joint >= ARM_JOINT_COUNT)
    {
        return;
    }

    mapping = &g_arm_joint_mapping[joint];
    joint_angle_rad =
        mapping->direction * output_angle_rad - mapping->zero_rad;
    if (mapping->wrap_to_pi)
    {
        joint_angle_rad = arm_task_wrap_to_pi(joint_angle_rad);
    }

    g_arm_task.joint_feedback[joint].angle_rad = joint_angle_rad;
    g_arm_task.joint_feedback[joint].valid = valid;
}

static void arm_task_update_joint_feedback(void)
{
    const Unitree_Motor_Data_t *pitch2 = &g_arm_task.pitch2_motor.Data;

    arm_task_store_joint_feedback(
        ARM_JOINT_BIG_YAW,
        g_arm_task.damiao_motor[ARM_DAMIAO_BIG_YAW].Data.pos,
        arm_task_damiao_feedback_is_confirmed(
            g_arm_task.motor_link_state[ARM_JOINT_BIG_YAW]));
    arm_task_store_joint_feedback(
        ARM_JOINT_PITCH1,
        g_arm_task.damiao_motor[ARM_DAMIAO_PITCH1].Data.pos,
        arm_task_damiao_feedback_is_confirmed(
            g_arm_task.motor_link_state[ARM_JOINT_PITCH1]));
    arm_task_store_joint_feedback(
        ARM_JOINT_ROLL1,
        g_arm_task.damiao_motor[ARM_DAMIAO_ROLL1].Data.pos,
        arm_task_damiao_feedback_is_confirmed(
            g_arm_task.motor_link_state[ARM_JOINT_ROLL1]));
    arm_task_store_joint_feedback(
        ARM_JOINT_PITCH2,
        pitch2->Pos - pitch2->Pos_Offset,
        (g_arm_task.motor_link_state[ARM_JOINT_PITCH2] ==
             ARM_MOTOR_LINK_ONLINE) &&
            (pitch2->cnt >= 5U));
    arm_task_store_joint_feedback(
        ARM_JOINT_ROLL2,
        g_arm_task.damiao_motor[ARM_DAMIAO_ROLL2].Data.pos,
        arm_task_damiao_feedback_is_confirmed(
            g_arm_task.motor_link_state[ARM_JOINT_ROLL2]));
    arm_task_store_joint_feedback(
        ARM_JOINT_PITCH3,
        g_arm_task.damiao_motor[ARM_DAMIAO_PITCH3].Data.pos,
        arm_task_damiao_feedback_is_confirmed(
            g_arm_task.motor_link_state[ARM_JOINT_PITCH3]));
    arm_task_store_joint_feedback(
        ARM_JOINT_ROLL3,
        (float)g_arm_task.roll3_motor.Data.Encoder * ARM_TWO_PI_RAD /
            ARM_DJI_ENCODER_COUNTS_PER_TURN,
        g_arm_task.motor_link_state[ARM_JOINT_ROLL3] ==
            ARM_MOTOR_LINK_ONLINE);
    arm_task_store_joint_feedback(
        ARM_JOINT_GRIP,
        g_arm_task.damiao_motor[ARM_DAMIAO_GRIP].Data.pos,
        arm_task_damiao_feedback_is_confirmed(
            g_arm_task.motor_link_state[ARM_JOINT_GRIP]));
}

void ArmTask_OnCan2Rx(uint32_t identifier,
                      uint8_t *data,
                      uint32_t data_len)
{
    uint32_t index;

    if (data == NULL)
    {
        return;
    }

    if (identifier == g_arm_task.roll3_motor.ID_Set.RxIdentifier)
    {
        DJI_Motor_Info_Update(&g_arm_task.roll3_motor, data, data_len);
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

void ArmTask_OnUsart10Rx(const uint8_t *data, uint16_t data_len)
{
    (void)Unitree_Motor_Info_Update(
        &g_arm_task.pitch2_motor,
        data,
        data_len);
}

static void arm_task_apply_motor_safe_state(bool enabled, uint32_t now_ms)
{
    const int16_t roll3_current[4] = {0, 0, 0, 0};
    bool command_changed = false;
    uint32_t offset;

    /* roll3 尚未接入双环 PID，本阶段始终发送零电流。ID 5 位于 0x2FE
     * 电流控制组的第一个槽位，其余三个未使用槽位也保持为零。
     */
    g_arm_task.debug.roll3_can_return_value =
        DJI_Motor_SendGroupCurrent(
            &hfdcan2,
            g_arm_task.roll3_motor.ID_Set.TxIdentifier,
            roll3_current);

    /* pitch2 尚未接入位置目标：解锁只进入零增益、零前馈 FOC；锁定时
     * 使用协议 LOCK。DMA 忙时不等待，由下一次 ArmTask 周期重试。
     */
    Unitree_Motor_Cmd(
        &g_arm_task.pitch2_motor,
        enabled ? UNITREE_MODE_FOC : UNITREE_MODE_LOCK,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f);
    g_arm_task.debug.pitch2_uart_return_value =
        Unitree_Motor_Ctrl(&huart10, &g_arm_task.pitch2_motor);

    /* TEMPORARY ARM COMMAND BASELINE BEGIN:
     * Reason: Arm control target generation is not implemented yet.
     * Scope: follow the vehicle lock state with Damiao enable/disable commands;
     *        never generate an MIT control target.
     * Remove when: ArmTask target generation is integrated and verified on
     *              the real mechanism.
     */
    for (offset = 0U; offset < ARM_DAMIAO_MOTOR_COUNT; offset++)
    {
        DM_Motor_Info_t *motor = &g_arm_task.damiao_motor[offset];

        if ((ARM_DAMIAO_INSTALLED_MASK & (1UL << offset)) == 0U)
        {
            continue;
        }

        if (enabled != motor->Command.requested_enabled)
        {
            motor->Command.requested_enabled = enabled;
            motor->Command.pending = true;
            motor->Command.awaiting_response = false;
            motor->Command.failed = false;
            motor->Command.attempt_count = 0U;
            motor->Command.next_retry_time_ms = now_ms;
            taskENTER_CRITICAL();
            motor->Command.request_feedback_sequence =
                motor->feedback_sequence;
            taskEXIT_CRITICAL();
            g_arm_task.motor_link_state[g_arm_damiao_joint_map[offset]] =
                ARM_MOTOR_LINK_UNKNOWN;
            command_changed = true;
        }
    }
    if (command_changed)
    {
        g_next_damiao_motor_index = 0U;
    }

    for (offset = 0U; offset < ARM_DAMIAO_MOTOR_COUNT; offset++)
    {
        uint32_t index =
            (g_next_damiao_motor_index + offset) %
            ARM_DAMIAO_MOTOR_COUNT;
        uint8_t send_result;
        DM_Motor_Info_t *motor = &g_arm_task.damiao_motor[index];

        if ((ARM_DAMIAO_INSTALLED_MASK & (1UL << index)) == 0U)
        {
            continue;
        }

        if (!motor->Command.pending ||
            !arm_task_time_reached(now_ms,
                                   motor->Command.next_retry_time_ms))
        {
            continue;
        }

        motor->Command.attempt_count++;
        if (motor->Command.requested_enabled)
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

        g_arm_task.debug.damiao.result_valid = true;
        g_arm_task.debug.damiao.command_enabled = enabled;
        g_arm_task.debug.damiao.motor = (ArmDamiaoMotor_e)index;
        g_arm_task.debug.damiao.return_value = send_result;

        if (send_result == 0U)
        {
            /* request_feedback_sequence 只在消费反馈时前移。重试发送不能
             * 覆盖确认基线，否则会吞掉恰好在重试前到达的有效反馈。
             */
            motor->Command.pending = false;
            motor->Command.awaiting_response = true;
            motor->Command.request_time_ms = now_ms;
            motor->Command.next_retry_time_ms =
                now_ms + ARM_DAMIAO_RETRY_INTERVAL_MS;
            g_arm_task.motor_link_state[g_arm_damiao_joint_map[index]] =
                ARM_MOTOR_LINK_UNKNOWN;
        }
        else if (motor->Command.attempt_count >= ARM_DAMIAO_MAX_ATTEMPTS)
        {
            motor->Command.pending = false;
            motor->Command.awaiting_response = false;
            motor->Command.failed = true;
            g_arm_task.motor_link_state[g_arm_damiao_joint_map[index]] =
                ARM_MOTOR_LINK_DM_COMMAND_FAILED;
        }
        else
        {
            motor->Command.next_retry_time_ms =
                now_ms + ARM_DAMIAO_RETRY_INTERVAL_MS;
            g_arm_task.motor_link_state[g_arm_damiao_joint_map[index]] =
                ARM_MOTOR_LINK_UNKNOWN;
        }
        g_next_damiao_motor_index =
            (uint8_t)((index + 1U) % ARM_DAMIAO_MOTOR_COUNT);
        return;
    }
    /* TEMPORARY ARM COMMAND BASELINE END */
}

static void arm_task_step(bool enabled)
{
    uint32_t now_ms = DWT_GetTimeMs();

    BSP_USART10_RecoverRxIfPending();
    arm_task_update_motor_link_state(now_ms);
    arm_task_apply_motor_safe_state(enabled, now_ms);
    arm_task_update_error();
    (void)LedSystem_SetFault(
        LED_SYSTEM_FAULT_ARM_MOTOR_ERROR,
        g_arm_task.error.type != ARM_TASK_ERROR_NONE);
    arm_task_update_joint_feedback();
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
    bool enabled = false;

    g_arm_task_thread = osThreadGetId();
    g_arm_task_start_time_ms = DWT_GetTimeMs();

    for (;;)
    {
        /* 仅在收到有效通知时更新输出许可。两个输入周期内
         * 没有通知时保持机械臂当前状态，避免任务调度抖动反复触发
         * Damiao 使能/失能。上电初值仍为失能。
         */
        notification = osThreadFlagsWait(ARM_TASK_NOTIFY_MASK,
                                         osFlagsWaitAny,
                                         2U);
        if ((notification & osFlagsError) == 0U)
        {
            /* ENABLED/DISABLED 同时累积时仍以失能为优先。 */
            enabled =
                ((notification & ARM_TASK_NOTIFY_DISABLED) == 0U) &&
                ((notification & ARM_TASK_NOTIFY_ENABLED) != 0U);
        }
        arm_task_step(enabled);

        if (g_arm_task.error.type != ARM_TASK_ERROR_NONE)
        {
            /* 电机命令确认失败或失联期间只运行通信恢复和安全输出；
             * 未来控制链不得越过此门。
             */
            continue;
        }

        /* 后续机械臂目标、运动学和控制链从这里接入。 */
    }
}
