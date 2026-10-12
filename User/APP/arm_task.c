/**
 * @file arm_task.c
 * @brief 新机械臂固定装配及电机内部 MIT 角度闭环。
 * @note 调试阶段只由 Keil 改目标/增益；原因是新自定义控制器协议尚未接入。
 *       范围：全部 8 轴；移除条件：新控制器目标输入经过确认并正式接入。
 */
#include "arm_task.h"
#include "robot_board_config.h"

#if ROBOT_BOARD_ROLE == ROBOT_BOARD_ARM
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_dwt.h"
#include "fdcan.h"
#include "led_system.h"
#include <math.h>

#define ARM_TASK_PERIOD_MS 1U
#define ARM_FEEDBACK_TIMEOUT_MS 50U
#define ARM_MANAGEMENT_PERIOD_MS 10U
#define ARM_ALL_MOTORS_MASK ((uint16_t)((1UL << ARM_JOINT_COUNT) - 1UL))

ArmTask_t g_arm_task = {
    .damiao_motor = {
        [ARM_JOINT_YAW] = DM_MOTOR_CREATE(Mit_mode, DM_J4340, 0x01U, 0x11U),
        [ARM_JOINT_PITCH1] = DM_MOTOR_CREATE(Mit_mode, DM_J10010L, 0x02U, 0x12U),
        [ARM_JOINT_ROLL1] = DM_MOTOR_CREATE(Mit_mode, DM_J4340, 0x03U, 0x13U),
        [ARM_JOINT_PITCH2] = DM_MOTOR_CREATE(Mit_mode, DM_J4340, 0x04U, 0x14U),
        [ARM_JOINT_ROLL2] = DM_MOTOR_CREATE(Mit_mode, DM_J4310, 0x05U, 0x15U),
        [ARM_JOINT_PITCH3] = DM_MOTOR_CREATE(Mit_mode, DM_J4310, 0x06U, 0x16U),
    },
    .rs05_motor = {
        [0] = RS05_MOTOR_CREATE(RS05_MOTOR_PROTOCOL_MIT, 0x07U, 0xFDU),
        [1] = RS05_MOTOR_CREATE(RS05_MOTOR_PROTOCOL_MIT, 0x08U, 0xFDU),
    },
    .control = {.state = ARM_CONTROL_DISABLED},
};

/* 两个厂商各自保留驱动实例，不建立通用电机注册表或转发适配函数。 */
static FDCAN_HandleTypeDef *const g_arm_dm_bus[ARM_DAMIAO_MOTOR_COUNT] = {
    &hfdcan1, &hfdcan1, &hfdcan1, &hfdcan2, &hfdcan2, &hfdcan2,
};
static ArmTaskCommand_t g_arm_input;
static uint32_t g_arm_enable_sequence[ARM_JOINT_COUNT];
static uint32_t g_arm_start_time_ms;
static uint32_t g_arm_management_time_ms;
static uint32_t g_arm_enable_time_ms;
static bool g_arm_management_started;
static bool g_arm_previous_enabled;
static uint16_t g_arm_enable_sent_mask;

void ArmTask_Notify(const ArmTaskCommand_t *command)
{
    if (command == NULL)
    {
        return;
    }
    taskENTER_CRITICAL();
    g_arm_input = *command;
    taskEXIT_CRITICAL();
}

void ArmTask_OnCanRx(FDCAN_HandleTypeDef *hcan,
                      const FDCAN_RxHeaderTypeDef *header,
                      const uint8_t *data, uint32_t data_length)
{
    uint32_t index;
    uint32_t first;

    if ((hcan == NULL) || (header == NULL) || (data == NULL) ||
        (data_length != 8U) || (header->DataLength != FDCAN_DLC_BYTES_8) ||
        (header->IdType != FDCAN_STANDARD_ID) ||
        (header->RxFrameType != FDCAN_DATA_FRAME) ||
        (header->FDFormat != FDCAN_CLASSIC_CAN) ||
        (header->BitRateSwitch != FDCAN_BRS_OFF))
    {
        return;
    }
    if (hcan == &hfdcan3)
    {
        for (index = 0U; index < ARM_RS05_MOTOR_COUNT; index++)
        {
            (void)RS05_Motor_OnRx(&g_arm_task.rs05_motor[index], header, data,
                                  data_length, DWT_GetTimeMs());
        }
        return;
    }
    if ((hcan != &hfdcan1) && (hcan != &hfdcan2))
    {
        return;
    }
    first = hcan == &hfdcan1 ? 0U : 3U;
    for (index = first; index < first + 3U; index++)
    {
        DM_Motor_Info_t *motor = &g_arm_task.damiao_motor[index];
        if (header->Identifier != motor->ID_Set.RxIdentifier)
        {
            continue;
        }
        if ((data[0] & 0x0FU) == motor->ID_Set.TxIdentifier)
        {
            DM_Motor_Info_Update(motor, data, header->DataLength);
        }
        return;
    }
}

/** @brief 在同一临界区提取本周期运动反馈，不持锁发送 CAN。 */
static uint32_t arm_task_snapshot_feedback(void)
{
    uint32_t index;
    uint32_t now_ms;

    taskENTER_CRITICAL();
    /* 同步取时，避免 tick 边界上的新反馈时间晚于 owner 先前取到的 now。 */
    now_ms = DWT_GetTimeMs();
    for (index = 0U; index < ARM_DAMIAO_MOTOR_COUNT; index++)
    {
        DM_Motor_Info_t *motor = &g_arm_task.damiao_motor[index];
        ArmJointFeedback_t *feedback = &g_arm_task.joint_feedback[index];
        if (motor->feedback_valid &&
            ((uint32_t)(now_ms - motor->last_feedback_time_ms) > ARM_FEEDBACK_TIMEOUT_MS))
        {
            motor->feedback_valid = false;
        }
        feedback->angle_rad = motor->Data.pos;
        feedback->velocity_rad_s = motor->Data.vel;
        feedback->torque_nm = motor->Data.tor;
        feedback->received_time_ms = motor->last_feedback_time_ms;
        feedback->sequence = motor->feedback_sequence;
        feedback->state = (uint8_t)motor->Data.state;
        feedback->valid = motor->feedback_valid;
        feedback->fault = motor->feedback_valid &&
            (motor->Data.state != DM_MOTOR_STATE_DISABLED) &&
            (motor->Data.state != DM_MOTOR_STATE_ENABLED);
    }
    for (index = 0U; index < ARM_RS05_MOTOR_COUNT; index++)
    {
        RS05_Motor_Info_t *motor = &g_arm_task.rs05_motor[index];
        ArmJointFeedback_t *feedback =
            &g_arm_task.joint_feedback[ARM_JOINT_ROLL3 + index];
        if (motor->feedback.valid &&
            ((uint32_t)(now_ms - motor->feedback.last_update_ms) > ARM_FEEDBACK_TIMEOUT_MS))
        {
            motor->feedback.valid = false;
        }
        feedback->angle_rad = motor->feedback.data.position_rad;
        feedback->velocity_rad_s = motor->feedback.data.velocity_rad_s;
        feedback->torque_nm = motor->feedback.data.torque_nm;
        feedback->received_time_ms = motor->feedback.last_update_ms;
        feedback->sequence = motor->feedback.sequence;
        feedback->state = motor->feedback.data.state;
        feedback->valid = motor->feedback.valid;
        feedback->fault = motor->feedback.valid && motor->feedback.data.fault;
    }
    taskEXIT_CRITICAL();
    return now_ms;
}

/** @brief 聚合全部电机健康状态；DM 的使能码 1 与 RS05 的运行码 2 分别判断。 */
static uint16_t arm_task_feedback_gate(uint32_t now_ms, ArmTaskError_e *reason)
{
    uint16_t mask = 0U;
    uint32_t joint;
    bool starting = (uint32_t)(now_ms - g_arm_start_time_ms) <= ARM_FEEDBACK_TIMEOUT_MS;

    *reason = ARM_TASK_ERROR_MOTOR_LOST;
    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        const ArmJointFeedback_t *feedback = &g_arm_task.joint_feedback[joint];
        uint8_t running_state = joint < ARM_DAMIAO_MOTOR_COUNT ? 1U : 2U;
        bool fresh = feedback->valid &&
            ((uint32_t)(now_ms - feedback->received_time_ms) <= ARM_FEEDBACK_TIMEOUT_MS);

        if (!fresh)
        {
            g_arm_task.motor_link_state[joint] = starting
                ? ARM_MOTOR_LINK_UNKNOWN : ARM_MOTOR_LINK_OFFLINE;
            if (!starting || (g_arm_task.control.state != ARM_CONTROL_DISABLED))
            {
                mask |= (uint16_t)(1UL << joint);
            }
        }
        else if (feedback->fault || !isfinite(feedback->angle_rad) ||
                 !isfinite(feedback->velocity_rad_s) || !isfinite(feedback->torque_nm) ||
                 ((g_arm_task.control.state == ARM_CONTROL_ACTIVE) &&
                  (feedback->state != running_state)))
        {
            g_arm_task.motor_link_state[joint] = ARM_MOTOR_LINK_ERROR;
            mask |= (uint16_t)(1UL << joint);
            *reason = ARM_TASK_ERROR_MOTOR_FAULT;
        }
        else
        {
            g_arm_task.motor_link_state[joint] = feedback->state == running_state
                ? ARM_MOTOR_LINK_ENABLED
                : (feedback->state == 0U ? ARM_MOTOR_LINK_DISABLED : ARM_MOTOR_LINK_ONLINE);
        }
    }
    return mask;
}

/** @brief 整臂失能的真实硬件边界；即使某一路失败，仍尝试其他所有电机。 */
static uint16_t arm_task_disable_all(void)
{
    uint32_t index;
    uint16_t failed = 0U;

    for (index = 0U; index < ARM_DAMIAO_MOTOR_COUNT; index++)
    {
        DM_Motor_Info_t *motor = &g_arm_task.damiao_motor[index];
        if (DM_Disable_Motor(g_arm_dm_bus[index],
                             (uint16_t)motor->ID_Set.TxIdentifier, Mit_mode, 0U) != 0U)
        {
            failed |= (uint16_t)(1UL << index);
        }
    }
    for (index = 0U; index < ARM_RS05_MOTOR_COUNT; index++)
    {
        if (RS05_Motor_Disable(&hfdcan3, &g_arm_task.rs05_motor[index]) != RS05_MOTOR_OK)
        {
            failed |= (uint16_t)(1UL << (ARM_JOINT_ROLL3 + index));
        }
    }
    return failed;
}

/**
 * @brief 失能时周期下发失能命令，探测运动反馈。
 * @note RS05 已提前配置好运控模式，本任务不设置或读取运行模式。
 *       每次最多每路 CAN 3 帧，分开于正常 1 ms 运控，无阻塞等待。
 */
static void arm_task_manage_disabled(uint32_t now_ms, bool force)
{
    uint16_t failed;

    if (!force && g_arm_management_started &&
        ((uint32_t)(now_ms - g_arm_management_time_ms) < ARM_MANAGEMENT_PERIOD_MS))
    {
        return;
    }
    g_arm_management_started = true;
    g_arm_management_time_ms = now_ms;
    failed = arm_task_disable_all();
    if (failed != 0U)
    {
        g_arm_task.error.type = ARM_TASK_ERROR_SEND_FAILED;
        g_arm_task.error.motor_mask = failed;
    }
}

/** @brief 只有八轴反馈新鲜、无故障且确认停止，才接受新的使能边沿。 */
static uint16_t arm_task_not_ready_mask(uint32_t now_ms)
{
    uint32_t joint;
    uint16_t mask = 0U;

    taskENTER_CRITICAL();
    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        const ArmJointFeedback_t *feedback = &g_arm_task.joint_feedback[joint];
        if (!feedback->valid || feedback->fault ||
            (feedback->state != 0U) ||
            ((uint32_t)(now_ms - feedback->received_time_ms) > ARM_FEEDBACK_TIMEOUT_MS))
        {
            mask |= (uint16_t)(1UL << joint);
        }
    }
    taskEXIT_CRITICAL();
    return mask;
}

/** @brief 逐轴发送使能并等待本次请求之后的新运行反馈；不靠 HAL_OK 推断使能。 */
static uint16_t arm_task_enable_pending(uint32_t now_ms, uint16_t *confirmed)
{
    uint32_t joint;
    uint16_t failed = 0U;
    bool send_due = !g_arm_management_started ||
        ((uint32_t)(now_ms - g_arm_management_time_ms) >= ARM_MANAGEMENT_PERIOD_MS);

    *confirmed = 0U;
    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        const ArmJointFeedback_t *feedback = &g_arm_task.joint_feedback[joint];
        uint16_t bit = (uint16_t)(1UL << joint);
        uint8_t running_state = joint < ARM_DAMIAO_MOTOR_COUNT ? 1U : 2U;
        if (((g_arm_enable_sent_mask & bit) != 0U) && feedback->valid &&
            (feedback->sequence != g_arm_enable_sequence[joint]) &&
            (feedback->state == running_state))
        {
            *confirmed |= bit;
            continue;
        }
        if (!send_due)
        {
            continue;
        }
        if (joint < ARM_DAMIAO_MOTOR_COUNT)
        {
            if (DM_Enable_Motor(g_arm_dm_bus[joint],
                                (uint16_t)g_arm_task.damiao_motor[joint].ID_Set.TxIdentifier,
                                Mit_mode, 0U) != 0U)
            {
                failed |= bit;
                continue;
            }
        }
        else if (RS05_Motor_Enable(&hfdcan3,
                    &g_arm_task.rs05_motor[joint - ARM_JOINT_ROLL3]) != RS05_MOTOR_OK)
        {
            failed |= bit;
            continue;
        }
        g_arm_enable_sent_mask |= bit;
    }
    if (send_due)
    {
        g_arm_management_started = true;
        g_arm_management_time_ms = now_ms;
    }
    return failed;
}

/**
 * @brief 每周期先校验八轴目标，再由厂商驱动限幅并下发 MIT 帧。
 * @note 速度和前馈力矩均为 0；本模块不计算 PD，不增加产品力矩上限。
 */
static uint16_t arm_task_send_motion(bool freeze_targets, ArmTaskError_e *reason)
{
    float target[ARM_JOINT_COUNT];
    float kp[ARM_JOINT_COUNT];
    float kd[ARM_JOINT_COUNT];
    uint16_t failed = 0U;
    uint32_t joint;

    *reason = ARM_TASK_ERROR_COMMAND_INVALID;
    /* BEGIN Keil 目标调试入口：新控制器协议未接入，首版逐轴在调试器修改目标。
     * 范围仅目标来源；新控制器目标输入正式接入后，替换此处的直接目标读取。
     * 增益仍作为电机内部运控参数保存，H7 不计算 PD。
     */
    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        target[joint] = freeze_targets ? g_arm_task.control.applied_target_rad[joint]
                                      : g_arm_task.control.joint_target_rad[joint];
        kp[joint] = g_arm_task.control.mit_gain[joint].kp;
        kd[joint] = g_arm_task.control.mit_gain[joint].kd;
        if (!isfinite(target[joint]) || !isfinite(kp[joint]) || !isfinite(kd[joint]))
        {
            failed |= (uint16_t)(1UL << joint);
        }
    }
    /* END Keil 目标调试入口。 */
    if (failed != 0U)
    {
        return failed;
    }
    *reason = ARM_TASK_ERROR_SEND_FAILED;
    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        bool sent;
        if (joint < ARM_DAMIAO_MOTOR_COUNT)
        {
            DM_Motor_MitCommand_t applied;
            sent = DM_Motor_Ctrl(g_arm_dm_bus[joint], &g_arm_task.damiao_motor[joint],
                                 target[joint], 0.0f, kp[joint], kd[joint], 0.0f, 0U, &applied) == 0U;
            if (sent)
            {
                target[joint] = applied.pos;
                kp[joint] = applied.kp;
                kd[joint] = applied.kd;
            }
        }
        else
        {
            RS05_Motor_MotionCommand_t applied;
            RS05_Motor_Status_e status = RS05_Motor_ControlMotion(&hfdcan3,
                &g_arm_task.rs05_motor[joint - ARM_JOINT_ROLL3],
                target[joint], 0.0f, kp[joint], kd[joint], 0.0f, &applied);
            sent = (status == RS05_MOTOR_OK) || (status == RS05_MOTOR_CLAMPED);
            if (sent)
            {
                target[joint] = applied.position_rad;
                kp[joint] = applied.kp;
                kd[joint] = applied.kd;
            }
        }
        if (!sent)
        {
            failed |= (uint16_t)(1UL << joint);
            break;
        }
        g_arm_task.control.applied_target_rad[joint] = target[joint];
        /* 回写钳位/冻结后的目标，恢复 RC 时不回放掉线期间的 Keil 目标修改。 */
        g_arm_task.control.joint_target_rad[joint] = target[joint];
        /* 驱动已成功提交本帧，调试器中的增益同步为协议限幅后的值。 */
        g_arm_task.control.mit_gain[joint].kp = kp[joint];
        g_arm_task.control.mit_gain[joint].kd = kd[joint];
    }
    return failed;
}

/** @brief 一个完整控制周期；全局故障锁存只在看到遥控失能命令后解除。 */
static void arm_task_control_step(const ArmTaskCommand_t *input, uint32_t now_ms)
{
    ArmTaskError_e reason;
    uint16_t unhealthy = arm_task_feedback_gate(now_ms, &reason);
    uint16_t failed = 0U;
    bool enable_edge = input->enabled && !g_arm_previous_enabled;
    bool force_disable = false;
    uint32_t joint;

    g_arm_previous_enabled = input->enabled;
    if (!input->enabled)
    {
        force_disable = g_arm_task.control.state != ARM_CONTROL_DISABLED;
        g_arm_task.control.state = ARM_CONTROL_DISABLED;
        g_arm_task.control.fault_latched = false;
    }
    else if (unhealthy != 0U)
    {
        g_arm_task.error.type = reason;
        g_arm_task.error.motor_mask = unhealthy;
        force_disable = g_arm_task.control.state != ARM_CONTROL_DISABLED;
        g_arm_task.control.state = ARM_CONTROL_DISABLED;
        g_arm_task.control.fault_latched = true;
    }
    else if (enable_edge && !g_arm_task.control.fault_latched)
    {
        failed = arm_task_not_ready_mask(now_ms);
        if (failed != 0U)
        {
            reason = ARM_TASK_ERROR_NOT_READY;
        }
        else
        {
            for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
            {
                float angle = g_arm_task.joint_feedback[joint].angle_rad;
                g_arm_task.control.joint_target_rad[joint] = angle;
                g_arm_task.control.applied_target_rad[joint] = angle;
                g_arm_enable_sequence[joint] = g_arm_task.joint_feedback[joint].sequence;
            }
            g_arm_enable_sent_mask = 0U;
            g_arm_enable_time_ms = now_ms;
            g_arm_management_started = false;
            g_arm_task.control.state = ARM_CONTROL_ENABLING;
            g_arm_task.error.type = ARM_TASK_ERROR_NONE;
            g_arm_task.error.motor_mask = 0U;
        }
    }

    if ((failed == 0U) && (g_arm_task.control.state == ARM_CONTROL_ENABLING))
    {
        uint16_t confirmed;
        failed = arm_task_enable_pending(now_ms, &confirmed);
        reason = ARM_TASK_ERROR_SEND_FAILED;
        if (confirmed == ARM_ALL_MOTORS_MASK)
        {
            g_arm_task.control.state = ARM_CONTROL_ACTIVE;
        }
        else if ((uint32_t)(now_ms - g_arm_enable_time_ms) > ARM_FEEDBACK_TIMEOUT_MS)
        {
            failed = (uint16_t)(ARM_ALL_MOTORS_MASK & (uint16_t)~confirmed);
            reason = ARM_TASK_ERROR_ENABLE_TIMEOUT;
        }
    }
    if ((failed == 0U) && (g_arm_task.control.state == ARM_CONTROL_ACTIVE))
    {
        failed = arm_task_send_motion(input->freeze_targets, &reason);
    }
    if (failed != 0U)
    {
        g_arm_task.error.type = reason;
        g_arm_task.error.motor_mask = failed;
        g_arm_task.control.fault_latched = true;
        g_arm_task.control.state = ARM_CONTROL_DISABLED;
        force_disable = true;
    }
    if (g_arm_task.control.state == ARM_CONTROL_DISABLED)
    {
        arm_task_manage_disabled(now_ms, force_disable);
        if (!input->enabled && (unhealthy != 0U))
        {
            g_arm_task.error.type = reason;
            g_arm_task.error.motor_mask = unhealthy;
        }
    }
    (void)LedSystem_SetFault(LED_SYSTEM_FAULT_ARM_MOTOR_ERROR,
        g_arm_task.control.fault_latched || (unhealthy != 0U));
}

void ArmTask_Run(void)
{
    TickType_t wake_time = xTaskGetTickCount();
    ArmTaskCommand_t input;
    uint32_t now_ms;

    g_arm_start_time_ms = DWT_GetTimeMs();
    for (;;)
    {
        taskENTER_CRITICAL();
        input = g_arm_input;
        taskEXIT_CRITICAL();
        now_ms = arm_task_snapshot_feedback();
        arm_task_control_step(&input, now_ms);
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(ARM_TASK_PERIOD_MS));
    }
}
#endif /* ROBOT_BOARD_ROLE == ROBOT_BOARD_ARM */
