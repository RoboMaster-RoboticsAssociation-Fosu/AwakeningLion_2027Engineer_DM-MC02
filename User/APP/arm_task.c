/**
 ******************************************************************************
 * @file    arm_task.c
 * @brief   机械臂任务、固定装配、目标状态机与控制链。
 ******************************************************************************
 */

#include "arm_task.h"

#include "FreeRTOS.h"
#include "arm_gravity_compensation.h"
#include "bsp_dwt.h"
#include "cmsis_os2.h"
#include "bsp_uart.h"
#include "fdcan.h"
#include "led_system.h"
#include "Remote_Control.h"
#include "task.h"
#include "usart.h"

#include <math.h>
#include <stddef.h>

#define ARM_TASK_NOTIFY_COMMAND         (1UL << 0U)
#define ARM_PI_RAD                   3.14159265358979323846f
#define ARM_TWO_PI_RAD               (2.0f * ARM_PI_RAD)
#define ARM_DJI_ENCODER_COUNTS_PER_TURN 8192.0f
#define ARM_TASK_PERIOD_S                0.001f
#define ARM_JOINT_TARGET_STEP_RAD        0.0005f
#define ARM_CAN2_SEND_PERIOD_MS          2U
#define ARM_DJI_FEEDBACK_TIMEOUT_MS      50U
#define ARM_UNITREE_FEEDBACK_TIMEOUT_MS  20U
#define ARM_DAMIAO_CONTROL_TIMEOUT_MS    50U
#define ARM_DAMIAO_RESPONSE_TIMEOUT_MS   10U
#define ARM_DAMIAO_RETRY_INTERVAL_MS     50U
#define ARM_DAMIAO_MAX_RETRIES           5U
#define ARM_DAMIAO_MAX_ATTEMPTS          (1U + ARM_DAMIAO_MAX_RETRIES)
#define ARM_HOLD_MOVING_THRESHOLD_RAD_S  0.08f
#define ARM_HOLD_SETTLE_TIME_MS           50U
#define ARM_FEEDBACK_JUMP_LIMIT_RAD       0.35f
#define ARM_GRIP_CONTROL_SENSITIVITY      2.0f
#define ARM_GRIP_TARGET_MIN_RAD           (-2.3f)
#define ARM_GRIP_TARGET_MAX_RAD           0.6f
#define ARM_PITCH2_TORQUE_MIN_NM          (-1.42f)
#define ARM_PITCH2_TORQUE_MAX_NM          1.43f
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

typedef struct
{
    float kp;
    float kd;
    float max_step_rad;
} ArmJointControlConfig_t;

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

/** Motor-side position gains and fixed joint-space target steps. */
static const ArmJointControlConfig_t
    g_arm_joint_control_config[ARM_JOINT_COUNT] = {
        [ARM_JOINT_BIG_YAW] = {25.0f, 0.01f, ARM_JOINT_TARGET_STEP_RAD},
        [ARM_JOINT_PITCH1] = {63.0f, 0.07f, ARM_JOINT_TARGET_STEP_RAD},
        [ARM_JOINT_ROLL1] = {0.0f, 0.0f, ARM_JOINT_TARGET_STEP_RAD},
        [ARM_JOINT_PITCH2] = {1.0f, 0.06f, ARM_JOINT_TARGET_STEP_RAD},
        [ARM_JOINT_ROLL2] = {7.0f, 0.01f, ARM_JOINT_TARGET_STEP_RAD},
        [ARM_JOINT_PITCH3] = {10.0f, 0.02f, ARM_JOINT_TARGET_STEP_RAD},
        [ARM_JOINT_ROLL3] = {0.0f, 0.0f, ARM_JOINT_TARGET_STEP_RAD},
        [ARM_JOINT_GRIP] = {18.0f, 0.10f, ARM_JOINT_TARGET_STEP_RAD},
    };

/*
 * Preset poses use calibrated joint-space radians in ArmJoint_e order.
 * Trigger conditions are defined by InputTask:
 * - SW2=UP enters PRESET mode and enables iw action triggers.
 * - SW1=DOWN plus an iw edge selects NORMAL.
 * - SW1=MID plus an iw edge selects CUSTOM_1.
 * - SW1=UP plus iw entering the upper region selects WAVE_1.
 * - SW1=UP plus iw entering the lower region selects WAVE_2.
 * - SW2=MID does not trigger a preset; the HOLD entry remains disabled.
 */
static const ArmPose_t g_arm_pose_normal = {
    .joint_rad = {
        [ARM_JOINT_BIG_YAW] = 0.0f,
        [ARM_JOINT_PITCH1] = 0.0f,
        [ARM_JOINT_ROLL1] = 0.0f,
        [ARM_JOINT_PITCH2] = 0.0f,
        [ARM_JOINT_ROLL2] = 0.0f,
        [ARM_JOINT_PITCH3] = 0.0f,
        [ARM_JOINT_ROLL3] = 0.0f,
    },
};
static const ArmPose_t g_arm_pose_wave_1 = {
    .joint_rad = {
        [ARM_JOINT_BIG_YAW] = 0.0f,
        [ARM_JOINT_PITCH1] = 0.64218f,
        [ARM_JOINT_ROLL1] = 0.0f,
        [ARM_JOINT_PITCH2] = 1.0447f,
        [ARM_JOINT_ROLL2] = 0.8f,
        [ARM_JOINT_PITCH3] = 1.3f,
        [ARM_JOINT_ROLL3] = 0.0f,
    },
};
static const ArmPose_t g_arm_pose_wave_2 = {
    .joint_rad = {
        [ARM_JOINT_BIG_YAW] = 0.0f,
        [ARM_JOINT_PITCH1] = 0.64218f,
        [ARM_JOINT_ROLL1] = 0.0f,
        [ARM_JOINT_PITCH2] = 1.0447f,
        [ARM_JOINT_ROLL2] = -0.8f,
        [ARM_JOINT_PITCH3] = 1.3f,
        [ARM_JOINT_ROLL3] = 0.0f,
    },
};
static const ArmPose_t g_arm_pose_custom_1 = {
    .joint_rad = {
        [ARM_JOINT_BIG_YAW] = 0.0f,
        [ARM_JOINT_PITCH1] = 0.5f,
        [ARM_JOINT_ROLL1] = 0.0f,
        [ARM_JOINT_PITCH2] = 0.75f,
        [ARM_JOINT_ROLL2] = 0.0f,
        [ARM_JOINT_PITCH3] = -0.32f,
        [ARM_JOINT_ROLL3] = 0.0f,
    },
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
            /* Pitch1 使用位置-速度模式：发送 ID 为 0x100 + 0x01。 */
            DM_MOTOR_CREATE(Pos_mode, DM_J10010L, 0x01U, 0x11U),
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
    .pid = {
        .roll3_angle = {
            .kp = 5.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .mode = PID_POSITIONAL_MODE,
            .improvement_flags = PID_IMPROVEMENT_OUTPUT_LIMIT |
                                 PID_IMPROVEMENT_INTEGRAL_LIMIT,
            .improvement = {
                .output_min = -100.0f,
                .output_max = 100.0f,
                .integral_limit = 10.0f,
            },
            .initialized = true,
            .last_status = PID_STATUS_OK,
        },
        .roll3_speed = {
            .kp = 40.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .mode = PID_POSITIONAL_MODE,
            .improvement_flags = PID_IMPROVEMENT_OUTPUT_LIMIT |
                                 PID_IMPROVEMENT_INTEGRAL_LIMIT,
            .improvement = {
                .output_min = -15000.0f,
                .output_max = 15000.0f,
                .integral_limit = 500.0f,
            },
            .initialized = true,
            .last_status = PID_STATUS_OK,
        },
    },
    .control = {
        .command = {
            .enabled = false,
            .mode = ARM_CONTROL_MODE_HOLD,
            .action = ARM_PRESET_ACTION_NONE,
        },
        .previous_mode = ARM_CONTROL_MODE_HOLD,
    },
};

static osThreadId_t g_arm_task_thread = NULL;
static uint8_t g_next_damiao_motor_index = 0U;
static uint32_t g_arm_task_start_time_ms = 0U;
static ArmTaskCommand_t g_arm_task_pending_command = {
    .enabled = false,
    .mode = ARM_CONTROL_MODE_HOLD,
    .action = ARM_PRESET_ACTION_NONE,
};

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
    uint32_t damiao_feedback_time_ms[ARM_DAMIAO_MOTOR_COUNT];
    bool damiao_feedback_valid[ARM_DAMIAO_MOTOR_COUNT];
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
        damiao_feedback_time_ms[index] = motor->last_feedback_time_ms;
        damiao_feedback_valid[index] = motor->feedback_valid;
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
            uint8_t control_mask = (uint8_t)(1UL << index);

            if (!motor->Command.requested_enabled ||
                ((g_arm_task.control.dm_control_started_mask & control_mask) ==
                 0U))
            {
                continue;
            }

            if (damiao_feedback_valid[index] &&
                (damiao_feedback_sequence[index] !=
                 g_arm_task.control.dm_control_feedback_sequence[index]))
            {
                g_arm_task.control.dm_control_feedback_sequence[index] =
                    damiao_feedback_sequence[index];
                *link_state =
                    (damiao_feedback_state[index] ==
                     DM_MOTOR_STATE_ENABLED)
                        ? ARM_MOTOR_LINK_ONLINE
                        : ARM_MOTOR_LINK_OFFLINE;
            }
            else if ((*link_state == ARM_MOTOR_LINK_ONLINE) &&
                     arm_task_feedback_is_fresh(
                         damiao_feedback_valid[index],
                         damiao_feedback_time_ms[index],
                         now_ms,
                         ARM_DAMIAO_CONTROL_TIMEOUT_MS))
            {
                /* A previously confirmed control link stays online while its
                 * most recent response remains fresh. */
            }
            else if ((uint32_t)(
                         now_ms -
                         g_arm_task.control.dm_first_control_time_ms[index]) >
                     ARM_DAMIAO_CONTROL_TIMEOUT_MS)
            {
                *link_state = ARM_MOTOR_LINK_OFFLINE;
            }
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
                                          bool valid,
                                          uint32_t sample_token)
{
    const ArmJointMapping_t *mapping;
    ArmFeedbackJumpFilter_t *filter;
    float joint_angle_rad;
    float filtered_angle_rad;

    if (joint >= ARM_JOINT_COUNT)
    {
        return;
    }

    mapping = &g_arm_joint_mapping[joint];
    filter = &g_arm_task.control.feedback_filter[joint];
    if (!valid)
    {
        ArmControlFilter_ResetFeedback(filter, sample_token);
        g_arm_task.joint_feedback[joint].valid = false;
        return;
    }

    joint_angle_rad =
        mapping->direction * output_angle_rad - mapping->zero_rad;
    if (mapping->wrap_to_pi)
    {
        joint_angle_rad = arm_task_wrap_to_pi(joint_angle_rad);
    }

    if (ArmControlFilter_UpdateFeedback(
            filter,
            joint_angle_rad,
            sample_token,
            ARM_FEEDBACK_JUMP_LIMIT_RAD,
            mapping->wrap_to_pi,
            &filtered_angle_rad))
    {
        g_arm_task.joint_feedback[joint].angle_rad = filtered_angle_rad;
        g_arm_task.joint_feedback[joint].valid = true;
    }
    else
    {
        g_arm_task.joint_feedback[joint].valid = false;
    }
}

static void arm_task_update_joint_feedback(void)
{
    const Unitree_Motor_Data_t *pitch2 = &g_arm_task.pitch2_motor.Data;

    arm_task_store_joint_feedback(
        ARM_JOINT_BIG_YAW,
        g_arm_task.damiao_motor[ARM_DAMIAO_BIG_YAW].Data.pos,
        arm_task_damiao_feedback_is_confirmed(
            g_arm_task.motor_link_state[ARM_JOINT_BIG_YAW]),
        g_arm_task.damiao_motor[ARM_DAMIAO_BIG_YAW].feedback_sequence);
    arm_task_store_joint_feedback(
        ARM_JOINT_PITCH1,
        g_arm_task.damiao_motor[ARM_DAMIAO_PITCH1].Data.pos,
        arm_task_damiao_feedback_is_confirmed(
            g_arm_task.motor_link_state[ARM_JOINT_PITCH1]),
        g_arm_task.damiao_motor[ARM_DAMIAO_PITCH1].feedback_sequence);
    arm_task_store_joint_feedback(
        ARM_JOINT_ROLL1,
        g_arm_task.damiao_motor[ARM_DAMIAO_ROLL1].Data.pos,
        arm_task_damiao_feedback_is_confirmed(
            g_arm_task.motor_link_state[ARM_JOINT_ROLL1]),
        g_arm_task.damiao_motor[ARM_DAMIAO_ROLL1].feedback_sequence);
    arm_task_store_joint_feedback(
        ARM_JOINT_PITCH2,
        pitch2->Pos - pitch2->Pos_Offset,
        (g_arm_task.motor_link_state[ARM_JOINT_PITCH2] ==
             ARM_MOTOR_LINK_ONLINE) &&
            (pitch2->cnt >= 5U),
        pitch2->last_feedback_time_ms);
    arm_task_store_joint_feedback(
        ARM_JOINT_ROLL2,
        g_arm_task.damiao_motor[ARM_DAMIAO_ROLL2].Data.pos,
        arm_task_damiao_feedback_is_confirmed(
            g_arm_task.motor_link_state[ARM_JOINT_ROLL2]),
        g_arm_task.damiao_motor[ARM_DAMIAO_ROLL2].feedback_sequence);
    arm_task_store_joint_feedback(
        ARM_JOINT_PITCH3,
        g_arm_task.damiao_motor[ARM_DAMIAO_PITCH3].Data.pos,
        arm_task_damiao_feedback_is_confirmed(
            g_arm_task.motor_link_state[ARM_JOINT_PITCH3]),
        g_arm_task.damiao_motor[ARM_DAMIAO_PITCH3].feedback_sequence);
    arm_task_store_joint_feedback(
        ARM_JOINT_ROLL3,
        (float)g_arm_task.roll3_motor.Data.Encoder * ARM_TWO_PI_RAD /
            ARM_DJI_ENCODER_COUNTS_PER_TURN,
        g_arm_task.motor_link_state[ARM_JOINT_ROLL3] ==
            ARM_MOTOR_LINK_ONLINE,
        g_arm_task.roll3_motor.last_feedback_time_ms);
    arm_task_store_joint_feedback(
        ARM_JOINT_GRIP,
        g_arm_task.damiao_motor[ARM_DAMIAO_GRIP].Data.pos,
        arm_task_damiao_feedback_is_confirmed(
            g_arm_task.motor_link_state[ARM_JOINT_GRIP]),
        g_arm_task.damiao_motor[ARM_DAMIAO_GRIP].feedback_sequence);
}

void ArmTask_OnCan1Rx(uint32_t identifier,
                      uint8_t *data,
                      uint32_t data_len)
{
    if (data == NULL)
    {
        return;
    }

    if (identifier == g_arm_task.roll3_motor.ID_Set.RxIdentifier)
    {
        DJI_Motor_Info_Update(&g_arm_task.roll3_motor, data, data_len);
    }
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

static void arm_task_process_enable_state(bool enabled, uint32_t now_ms)
{
    bool command_changed = false;
    uint32_t offset;

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
        g_arm_task.control.dm_control_started_mask = 0U;
        g_arm_task.control.target_initialized = false;
        g_arm_task.control.hold.initialized = false;
        g_arm_task.control.control_active = false;
        (void)pid_reset(&g_arm_task.pid.roll3_angle);
        (void)pid_reset(&g_arm_task.pid.roll3_speed);
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
                                          (uint16_t)motor->Mode,
                                          0U);
        }
        else
        {
            send_result = DM_Disable_Motor(&hfdcan2,
                                           (uint16_t)motor->ID_Set.TxIdentifier,
                                           (uint16_t)motor->Mode,
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
}

static float arm_task_clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static bool arm_task_joint_is_installed(ArmJoint_e joint)
{
    return (joint < ARM_JOINT_COUNT) &&
           ((ARM_JOINT_INSTALLED_MASK & (1UL << joint)) != 0U);
}

static float arm_task_joint_velocity_rad_s(ArmJoint_e joint)
{
    uint32_t index;

    if (joint == ARM_JOINT_PITCH2)
    {
        return g_arm_joint_mapping[joint].direction *
               g_arm_task.pitch2_motor.Data.Vel;
    }
    if (joint == ARM_JOINT_ROLL3)
    {
        return g_arm_joint_mapping[joint].direction *
               (float)g_arm_task.roll3_motor.Data.Velocity *
               ARM_TWO_PI_RAD / 60.0f;
    }

    for (index = 0U; index < ARM_DAMIAO_MOTOR_COUNT; index++)
    {
        if (g_arm_damiao_joint_map[index] == joint)
        {
            return g_arm_joint_mapping[joint].direction *
                   g_arm_task.damiao_motor[index].Data.vel;
        }
    }

    return 0.0f;
}

static void arm_task_capture_joint_targets(void)
{
    uint32_t joint;

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        if (arm_task_joint_is_installed((ArmJoint_e)joint) &&
            g_arm_task.joint_feedback[joint].valid)
        {
            g_arm_task.control.desired_joint_target_rad[joint] =
                g_arm_task.joint_feedback[joint].angle_rad;
            g_arm_task.control.joint_target_rad[joint] =
                g_arm_task.joint_feedback[joint].angle_rad;
        }
    }
    g_arm_task.control.target_initialized = true;
}

static const ArmPose_t *arm_task_get_preset_pose(ArmPresetAction_e action)
{
    switch (action)
    {
        case ARM_PRESET_ACTION_NORMAL:
            return &g_arm_pose_normal;
        case ARM_PRESET_ACTION_WAVE_1:
            return &g_arm_pose_wave_1;
        case ARM_PRESET_ACTION_WAVE_2:
            return &g_arm_pose_wave_2;
        case ARM_PRESET_ACTION_CUSTOM_1:
            return &g_arm_pose_custom_1;
        case ARM_PRESET_ACTION_NONE:
        default:
            return NULL;
    }
}

static void arm_task_update_preset_targets(
    const ArmTaskCommand_t *command)
{
    const ArmPose_t *pose;
    uint32_t joint;

    if (command->action_sequence !=
        g_arm_task.control.consumed_action_sequence)
    {
        g_arm_task.control.consumed_action_sequence =
            command->action_sequence;
        g_arm_task.control.active_action = command->action;
    }

    pose = arm_task_get_preset_pose(g_arm_task.control.active_action);
    if (pose == NULL)
    {
        return;
    }

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        if (!arm_task_joint_is_installed((ArmJoint_e)joint))
        {
            continue;
        }
        if ((joint == ARM_JOINT_GRIP) &&
            g_arm_task.joint_feedback[joint].valid)
        {
            float channel_ratio =
                (float)remote_ctrl.rc.ch[3] / (float)RC_CH_VALUE_MAX;
            float grip_step_rad =
                channel_ratio * g_arm_joint_control_config[joint]
                                      .max_step_rad *
                ARM_GRIP_CONTROL_SENSITIVITY;

            /* 夹爪不使用姿态表目标，ch[3] 仅以低灵敏度增量控制。 */
            g_arm_task.control.desired_joint_target_rad[joint] =
                arm_task_clamp_float(
                    g_arm_task.control.desired_joint_target_rad[joint] +
                        grip_step_rad,
                    ARM_GRIP_TARGET_MIN_RAD,
                    ARM_GRIP_TARGET_MAX_RAD);
            continue;
        }
        g_arm_task.control.desired_joint_target_rad[joint] =
            pose->joint_rad[joint];
    }
}

static void arm_task_update_hold_targets(uint32_t now_ms)
{
    uint32_t joint;

    if (!g_arm_task.control.hold.initialized)
    {
        arm_task_capture_joint_targets();
        for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
        {
            g_arm_task.control.hold.moving[joint] = false;
            g_arm_task.control.hold.settle_start_time_ms[joint] = 0U;
        }
        g_arm_task.control.hold.initialized = true;
    }

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        float speed_rad_s;

        if (!arm_task_joint_is_installed((ArmJoint_e)joint) ||
            !g_arm_task.joint_feedback[joint].valid)
        {
            continue;
        }

        speed_rad_s = fabsf(
            arm_task_joint_velocity_rad_s((ArmJoint_e)joint));
        if (speed_rad_s >= ARM_HOLD_MOVING_THRESHOLD_RAD_S)
        {
            g_arm_task.control.hold.moving[joint] = true;
            g_arm_task.control.hold.settle_start_time_ms[joint] = 0U;
            g_arm_task.control.desired_joint_target_rad[joint] =
                g_arm_task.joint_feedback[joint].angle_rad;
            continue;
        }

        if (!g_arm_task.control.hold.moving[joint])
        {
            continue;
        }

        g_arm_task.control.desired_joint_target_rad[joint] =
            g_arm_task.joint_feedback[joint].angle_rad;
        if (g_arm_task.control.hold.settle_start_time_ms[joint] == 0U)
        {
            g_arm_task.control.hold.settle_start_time_ms[joint] = now_ms;
        }
        else if ((uint32_t)(
                     now_ms -
                     g_arm_task.control.hold.settle_start_time_ms[joint]) >=
                 ARM_HOLD_SETTLE_TIME_MS)
        {
            g_arm_task.control.hold.moving[joint] = false;
            g_arm_task.control.hold.settle_start_time_ms[joint] = 0U;
        }
    }
}

static void arm_task_update_target_ramps(void)
{
    uint32_t joint;

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        if (!arm_task_joint_is_installed((ArmJoint_e)joint))
        {
            continue;
        }

        if (joint == ARM_JOINT_GRIP)
        {
            /* 夹爪已经通过遥控灵敏度限幅，不再叠加关节斜坡。 */
            g_arm_task.control.joint_target_rad[joint] =
                g_arm_task.control.desired_joint_target_rad[joint];
            continue;
        }

        float ramped_target_rad =
            g_arm_task.control.joint_target_rad[joint];

        if (ArmControlFilter_SlewAngle(
                g_arm_task.control.joint_target_rad[joint],
                g_arm_task.control.desired_joint_target_rad[joint],
                g_arm_joint_control_config[joint].max_step_rad,
                joint == ARM_JOINT_ROLL3,
                &ramped_target_rad))
        {
            g_arm_task.control.joint_target_rad[joint] =
                ramped_target_rad;
        }
    }
}

static void arm_task_update_control_targets(
    const ArmTaskCommand_t *command,
    uint32_t now_ms)
{
    if (command->mode != g_arm_task.control.previous_mode)
    {
        g_arm_task.control.previous_mode = command->mode;
        g_arm_task.control.hold.initialized = false;
        if (command->mode == ARM_CONTROL_MODE_PRESET)
        {
            /* Entering PRESET never replays the previously selected action. */
            g_arm_task.control.active_action = ARM_PRESET_ACTION_NONE;
        }
    }

    if (command->mode == ARM_CONTROL_MODE_HOLD)
    {
        arm_task_update_hold_targets(now_ms);
    }
    else
    {
        arm_task_update_preset_targets(command);
        if ((g_arm_task.control.active_action != ARM_PRESET_ACTION_NONE) &&
            !g_arm_task.control.target_initialized)
        {
            arm_task_capture_joint_targets();
            arm_task_update_preset_targets(command);
        }
    }

    if ((command->mode == ARM_CONTROL_MODE_HOLD) ||
        (g_arm_task.control.active_action != ARM_PRESET_ACTION_NONE))
    {
        arm_task_update_target_ramps();
    }
}

static bool arm_task_joint_to_motor_target(ArmJoint_e joint,
                                           float joint_target_rad,
                                           float *motor_target_rad)
{
    const ArmJointMapping_t *mapping;

    if ((joint >= ARM_JOINT_COUNT) || (motor_target_rad == NULL))
    {
        return false;
    }

    mapping = &g_arm_joint_mapping[joint];
    *motor_target_rad =
        mapping->direction * (joint_target_rad + mapping->zero_rad);
    if (joint == ARM_JOINT_PITCH2)
    {
        *motor_target_rad += g_arm_task.pitch2_motor.Data.Pos_Offset;
    }
    return true;
}

static ArmGravityOutput_t arm_task_calculate_gravity(void)
{
    ArmGravityInput_t input;
    ArmGravityOutput_t output = {0};

    if (!g_arm_task.joint_feedback[ARM_JOINT_PITCH1].valid ||
        !g_arm_task.joint_feedback[ARM_JOINT_PITCH2].valid ||
        !g_arm_task.joint_feedback[ARM_JOINT_ROLL2].valid ||
        !g_arm_task.joint_feedback[ARM_JOINT_PITCH3].valid)
    {
        return output;
    }

    input.pitch1_rad =
        g_arm_task.joint_feedback[ARM_JOINT_PITCH1].angle_rad;
    input.pitch2_rad =
        g_arm_task.joint_feedback[ARM_JOINT_PITCH2].angle_rad;
    input.roll2_rad =
        g_arm_task.joint_feedback[ARM_JOINT_ROLL2].angle_rad;
    input.pitch3_rad =
        g_arm_task.joint_feedback[ARM_JOINT_PITCH3].angle_rad;
    if (!ArmGravityCompensation_Calculate(&input, &output))
    {
        return (ArmGravityOutput_t){0};
    }

    output.pitch2_torque_nm = arm_task_clamp_float(
        output.pitch2_torque_nm,
        ARM_PITCH2_TORQUE_MIN_NM,
        ARM_PITCH2_TORQUE_MAX_NM);
    return output;
}

static bool arm_task_all_motors_ready(const ArmTaskCommand_t *command)
{
    uint32_t index;
    uint32_t joint;

    if (!command->enabled ||
        (g_arm_task.motor_link_state[ARM_JOINT_PITCH2] !=
         ARM_MOTOR_LINK_ONLINE) ||
        (g_arm_task.motor_link_state[ARM_JOINT_ROLL3] !=
         ARM_MOTOR_LINK_ONLINE))
    {
        return false;
    }

    for (index = 0U; index < ARM_DAMIAO_MOTOR_COUNT; index++)
    {
        ArmMotorLinkState_e state;

        if ((ARM_DAMIAO_INSTALLED_MASK & (1UL << index)) == 0U)
        {
            continue;
        }
        state = g_arm_task.motor_link_state[g_arm_damiao_joint_map[index]];
        if ((state != ARM_MOTOR_LINK_DM_ENABLED_CONFIRMED) &&
            (state != ARM_MOTOR_LINK_ONLINE))
        {
            return false;
        }
    }

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        if (arm_task_joint_is_installed((ArmJoint_e)joint) &&
            !g_arm_task.joint_feedback[joint].valid)
        {
            return false;
        }
    }
    return true;
}

static int16_t arm_task_compute_roll3_current(void)
{
    float feedback_rad =
        g_arm_task.joint_feedback[ARM_JOINT_ROLL3].angle_rad;
    float target_rad = g_arm_task.control
                           .joint_target_rad[ARM_JOINT_ROLL3];
    float nearest_target_rad =
        feedback_rad + arm_task_wrap_to_pi(target_rad - feedback_rad);
    float target_speed_rpm = 0.0f;
    float current_command = 0.0f;

    if (pid_compute(&g_arm_task.pid.roll3_angle,
                    nearest_target_rad * 180.0f / ARM_PI_RAD,
                    feedback_rad * 180.0f / ARM_PI_RAD,
                    ARM_TASK_PERIOD_S,
                    &target_speed_rpm) != PID_STATUS_OK)
    {
        return 0;
    }
    if (pid_compute(&g_arm_task.pid.roll3_speed,
                    target_speed_rpm,
                    (float)g_arm_task.roll3_motor.Data.Velocity,
                    ARM_TASK_PERIOD_S,
                    &current_command) != PID_STATUS_OK)
    {
        return 0;
    }

    current_command = arm_task_clamp_float(
        current_command,
        -15000.0f,
        15000.0f);
    return (int16_t)current_command;
}

static void arm_task_record_can2_result(uint8_t send_result)
{
    g_arm_task.debug.can2_send_count++;
    if (send_result != 0U)
    {
        g_arm_task.debug.can2_send_failure_count++;
    }
}

static void arm_task_mark_dm_control_started(uint32_t index,
                                             uint32_t now_ms)
{
    uint8_t control_mask = (uint8_t)(1UL << index);

    if ((g_arm_task.control.dm_control_started_mask & control_mask) != 0U)
    {
        return;
    }

    taskENTER_CRITICAL();
    g_arm_task.control.dm_control_feedback_sequence[index] =
        g_arm_task.damiao_motor[index].feedback_sequence;
    taskEXIT_CRITICAL();
    g_arm_task.control.dm_first_control_time_ms[index] = now_ms;
    g_arm_task.control.dm_control_started_mask |= control_mask;
}

static float arm_task_damiao_torque_feedforward(
    ArmJoint_e joint,
    const ArmGravityOutput_t *gravity)
{
    switch (joint)
    {
        case ARM_JOINT_ROLL2:
            return gravity->roll2_torque_nm;
        case ARM_JOINT_PITCH3:
            return gravity->pitch3_torque_nm;
        case ARM_JOINT_PITCH1:
            /* Source behavior: calculate Pitch1 gravity but do not apply it. */
        default:
            return 0.0f;
    }
}

static void arm_task_send_can2_control(bool enabled,
                                        bool control_allowed,
                                        const ArmGravityOutput_t *gravity,
                                        int16_t roll3_current,
                                        uint32_t now_ms)
{
    int16_t roll3_group_current[4] = {0, 0, 0, 0};
    bool preset_action_active =
        g_arm_task.control.active_action != ARM_PRESET_ACTION_NONE;
    uint32_t index;

    if (g_arm_task.control.can2_send_initialized &&
        ((uint32_t)(now_ms -
                    g_arm_task.control.last_can2_send_time_ms) <
         ARM_CAN2_SEND_PERIOD_MS))
    {
        return;
    }
    g_arm_task.control.last_can2_send_time_ms = now_ms;
    g_arm_task.control.can2_send_initialized = true;

    for (index = 0U; index < ARM_DAMIAO_MOTOR_COUNT; index++)
    {
        DM_Motor_Info_t *motor = &g_arm_task.damiao_motor[index];
        ArmJoint_e joint = g_arm_damiao_joint_map[index];
        float motor_target_rad = 0.0f;
        float motor_target_velocity_rad_s = 0.0f;
        float kp = 0.0f;
        float kd = 0.0f;
        float torque_ff = 0.0f;
        uint8_t send_result;

        if ((ARM_DAMIAO_INSTALLED_MASK & (1UL << index)) == 0U)
        {
            continue;
        }
        if (motor->Command.pending || motor->Command.awaiting_response ||
            motor->Command.failed)
        {
            continue;
        }

        /* DOWN/失能状态也发送零控制 MIT 观测帧，维持达妙一发一收。 */
        if (control_allowed && preset_action_active)
        {
            (void)arm_task_joint_to_motor_target(
                joint,
                g_arm_task.control.joint_target_rad[joint],
                &motor_target_rad);
            if (joint == ARM_JOINT_PITCH1)
            {
                /* Pos_mode 的 v_des 是位置梯形规划的最大速度。 */
                motor_target_velocity_rad_s = 0.8f;
            }
            kp = g_arm_joint_control_config[joint].kp;
            kd = g_arm_joint_control_config[joint].kd;
            torque_ff = arm_task_damiao_torque_feedforward(
                joint,
                gravity);
        }

        /*
         * 无动作时发送零控制 MIT 观测帧，维持达妙一发一收：
         * 位置、速度、Kp、Kd 和 torque_ff 均为 0，不使用预设目标。
         * 有效动作触发后，上方逻辑才装载正式位置控制参数。
         */
        send_result = DM_Motor_Ctrl(
            &hfdcan2,
            motor,
            motor_target_rad,
            motor_target_velocity_rad_s,
            kp,
            kd,
            torque_ff,
            0U);
        arm_task_record_can2_result(send_result);
        if (send_result == 0U)
        {
            arm_task_mark_dm_control_started(index, now_ms);
        }
    }

    if (enabled && control_allowed && !preset_action_active)
    {
        /* GM6020 持续上报反馈，不需要用发送帧维持观测。 */
        return;
    }

    roll3_group_current[0] = control_allowed ? roll3_current : 0;
    g_arm_task.debug.roll3_can_return_value =
        DJI_Motor_SendGroupCurrent(
            &hfdcan1,
            g_arm_task.roll3_motor.ID_Set.TxIdentifier,
            roll3_group_current);
    arm_task_record_can2_result(
        g_arm_task.debug.roll3_can_return_value);
}

static void arm_task_send_pitch2_control(
    bool enabled,
    bool control_allowed,
    const ArmGravityOutput_t *gravity)
{
    float motor_target_rad = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float torque_ff = 0.0f;
    bool preset_action_active =
        g_arm_task.control.active_action != ARM_PRESET_ACTION_NONE;

    if (control_allowed && preset_action_active)
    {
        (void)arm_task_joint_to_motor_target(
            ARM_JOINT_PITCH2,
            g_arm_task.control.joint_target_rad[ARM_JOINT_PITCH2],
            &motor_target_rad);
        kp = g_arm_joint_control_config[ARM_JOINT_PITCH2].kp;
        kd = g_arm_joint_control_config[ARM_JOINT_PITCH2].kd;
        torque_ff = gravity->pitch2_torque_nm;
    }

    Unitree_Motor_Cmd(
        &g_arm_task.pitch2_motor,
        (enabled && control_allowed && preset_action_active)
            ? UNITREE_MODE_FOC
            : UNITREE_MODE_LOCK,
        torque_ff,
        0.0f,
        motor_target_rad,
        kp,
        kd);
    g_arm_task.debug.pitch2_uart_return_value =
        Unitree_Motor_Ctrl(&huart10, &g_arm_task.pitch2_motor);
}

static void arm_task_deactivate_control(const ArmTaskCommand_t *command)
{
    if (g_arm_task.control.control_active)
    {
        (void)pid_reset(&g_arm_task.pid.roll3_angle);
        (void)pid_reset(&g_arm_task.pid.roll3_speed);
    }
    g_arm_task.control.control_active = false;
    g_arm_task.control.target_initialized = false;
    g_arm_task.control.hold.initialized = false;
    g_arm_task.control.active_action = ARM_PRESET_ACTION_NONE;
    g_arm_task.control.previous_mode = command->mode;
    g_arm_task.control.consumed_action_sequence =
        command->action_sequence;
}

static void arm_task_step(void)
{
    const ArmTaskCommand_t *command = &g_arm_task.control.command;
    uint32_t now_ms = DWT_GetTimeMs();
    ArmGravityOutput_t gravity = {0};
    bool control_allowed;
    int16_t roll3_current = 0;

    BSP_USART10_RecoverRxIfPending();
    arm_task_update_motor_link_state(now_ms);
    arm_task_process_enable_state(command->enabled, now_ms);
    arm_task_update_error();
    arm_task_update_joint_feedback();

    /* 故障只上报；已经运行的控制链不因故障状态被动停控。 */
    control_allowed = command->enabled &&
                      (g_arm_task.control.control_active ||
                       arm_task_all_motors_ready(command));
    if (control_allowed)
    {
        if (!g_arm_task.control.control_active)
        {
            (void)pid_reset(&g_arm_task.pid.roll3_angle);
            (void)pid_reset(&g_arm_task.pid.roll3_speed);
            g_arm_task.control.target_initialized = false;
            g_arm_task.control.hold.initialized = false;
            g_arm_task.control.active_action = ARM_PRESET_ACTION_NONE;
            g_arm_task.control.control_active = true;
        }

        arm_task_update_control_targets(command, now_ms);
        gravity = arm_task_calculate_gravity();
        roll3_current = arm_task_compute_roll3_current();
    }
    else
    {
        arm_task_deactivate_control(command);
    }

    arm_task_send_pitch2_control(
        command->enabled,
        control_allowed,
        &gravity);
    arm_task_send_can2_control(
        command->enabled,
        control_allowed,
        &gravity,
        roll3_current,
        now_ms);

    (void)LedSystem_SetFault(
        LED_SYSTEM_FAULT_ARM_MOTOR_ERROR,
        g_arm_task.error.type != ARM_TASK_ERROR_NONE);
}

void ArmTask_Notify(const ArmTaskCommand_t *command)
{
    if ((command == NULL) || (g_arm_task_thread == NULL) ||
        ((command->mode != ARM_CONTROL_MODE_PRESET) &&
         (command->mode != ARM_CONTROL_MODE_HOLD)) ||
        (command->action > ARM_PRESET_ACTION_CUSTOM_1))
    {
        return;
    }

    taskENTER_CRITICAL();
    g_arm_task_pending_command = *command;
    taskEXIT_CRITICAL();
    (void)osThreadFlagsSet(g_arm_task_thread, ARM_TASK_NOTIFY_COMMAND);
}

void ArmTask_Run(void)
{
    uint32_t notification;

    g_arm_task_thread = osThreadGetId();
    g_arm_task_start_time_ms = DWT_GetTimeMs();

    for (;;)
    {
        /* Two missed InputTask periods retain the latest complete command.
         * The power-on local snapshot remains disabled until the first input.
         */
        notification = osThreadFlagsWait(ARM_TASK_NOTIFY_COMMAND,
                                         osFlagsWaitAny,
                                         2U);
        if ((notification & osFlagsError) == 0U)
        {
            taskENTER_CRITICAL();
            g_arm_task.control.command = g_arm_task_pending_command;
            taskEXIT_CRITICAL();
        }
        arm_task_step();
    }
}
