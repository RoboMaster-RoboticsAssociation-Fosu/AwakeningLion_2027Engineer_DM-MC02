/**
 ******************************************************************************
 * @file    arm_task.h
 * @brief   机械臂任务及其 Damiao 电机装配实例。
 ******************************************************************************
 */

#ifndef ARM_TASK_H
#define ARM_TASK_H

#include "DM_Motor.h"
#include "DJI_Motor.h"
#include "Unitree_Motor.h"
#include "arm_control_filter.h"
#include "pid.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 六个 Damiao 电机在机械臂任务中的固定装配顺序。
 * @note ARM_DAMIAO_ROLL1 当前保留槽位但硬件未实装。
 */
typedef enum
{
    ARM_DAMIAO_BIG_YAW = 0,
    ARM_DAMIAO_PITCH1,
    ARM_DAMIAO_ROLL1,
    ARM_DAMIAO_ROLL2,
    ARM_DAMIAO_GRIP,
    ARM_DAMIAO_PITCH3,
    ARM_DAMIAO_MOTOR_COUNT
} ArmDamiaoMotor_e;

/**
 * @brief 机械臂物理关节角顺序。
 * @note roll1 当前硬件未实装；保留枚举和实例供后续装配，不参与使能、
 *       在线故障判定或正式机构控制。
 */
typedef enum
{
    ARM_JOINT_BIG_YAW = 0,
    ARM_JOINT_PITCH1,
    ARM_JOINT_ROLL1,
    ARM_JOINT_PITCH2,
    ARM_JOINT_ROLL2,
    ARM_JOINT_PITCH3,
    ARM_JOINT_ROLL3,
    ARM_JOINT_GRIP,
    ARM_JOINT_COUNT
} ArmJoint_e;

/** @brief Mechanical-arm control mode selected by InputTask. */
typedef enum
{
    ARM_CONTROL_MODE_PRESET = 1,
    ARM_CONTROL_MODE_HOLD = 2,
} ArmControlMode_e;

/** @brief Preset action selected by a valid remote-control edge. */
typedef enum
{
    ARM_PRESET_ACTION_NONE = 0,
    ARM_PRESET_ACTION_NORMAL,
    ARM_PRESET_ACTION_WAVE_1,
    ARM_PRESET_ACTION_WAVE_2,
    ARM_PRESET_ACTION_CUSTOM_1,
} ArmPresetAction_e;

/** @brief A complete mechanism pose in ArmJoint_e order, in rad. */
typedef struct
{
    float joint_rad[ARM_JOINT_COUNT];
} ArmPose_t;

/**
 * @brief InputTask command consumed by ArmTask.
 * @note action_sequence increments for every valid preset trigger, including
 *       repeatedly selecting the same action.
 */
typedef struct
{
    bool enabled;
    ArmControlMode_e mode;
    ArmPresetAction_e action;
    uint32_t action_sequence;
} ArmTaskCommand_t;

/**
 * @brief 机械臂电机通信的当前可观测状态。
 * @note Damiao 在使能/失能阶段保持命令确认状态；进入周期控制后，根据
 *       一发一收反馈切换为 ONLINE/OFFLINE。
 */
typedef enum
{
    ARM_MOTOR_LINK_UNKNOWN = 0,
    ARM_MOTOR_LINK_ONLINE,
    ARM_MOTOR_LINK_OFFLINE,
    ARM_MOTOR_LINK_DM_DISABLED_CONFIRMED,
    ARM_MOTOR_LINK_DM_ENABLED_CONFIRMED,
    ARM_MOTOR_LINK_DM_COMMAND_FAILED,
} ArmMotorLinkState_e;

/** @brief 机械臂任务当前异常类型。 */
typedef enum
{
    ARM_TASK_ERROR_NONE = 0,
    ARM_TASK_ERROR_DM_COMMAND_FAILED,
    ARM_TASK_ERROR_MOTOR_LOST,
} ArmTaskError_e;

/**
 * @brief 机械臂当前主异常及关联电机集合，位序使用 ArmJoint_e。
 * @note Damiao 命令确认失败优先于电机离线；motor_mask 合并两类异常电机。
 *       新命令确认成功或连续反馈恢复后由 ArmTask 自动清除对应异常。
 */
typedef struct
{
    ArmTaskError_e type;
    uint16_t motor_mask;
} ArmTaskError_t;

/** @brief 统一到机构坐标系的关节角反馈。 */
typedef struct
{
    float angle_rad;
    bool valid;
} ArmJointFeedback_t;

/**
 * @brief 最近一次 Damiao 使能/失能发送的调试观测量。
 * @note return_value 是 CAN 发送入队结果，0 表示入队成功，不代表电机应答。
 */
typedef struct
{
    bool result_valid;
    bool command_enabled;
    ArmDamiaoMotor_e motor;
    uint8_t return_value;
} ArmDamiaoDebug_t;

/** @brief 机械臂任务的通信调试观测量。 */
typedef struct
{
    ArmDamiaoDebug_t damiao;
    uint8_t roll3_can_return_value;
    uint8_t pitch2_uart_return_value;
    uint32_t can2_send_count;
    uint32_t can2_send_failure_count;
} ArmTaskDebug_t;

/** @brief Local PID instances owned by ArmTask. */
typedef struct
{
    PidController_t roll3_angle;
    PidController_t roll3_speed;
} ArmTaskPid_t;

/** @brief Per-joint state for hand-guided pose holding. */
typedef struct
{
    bool initialized;
    bool moving[ARM_JOINT_COUNT];
    uint32_t settle_start_time_ms[ARM_JOINT_COUNT];
} ArmHoldState_t;

/** @brief Cross-cycle state of the mechanical-arm control chain. */
typedef struct
{
    ArmTaskCommand_t command;
    ArmHoldState_t hold;
    ArmFeedbackJumpFilter_t feedback_filter[ARM_JOINT_COUNT];
    float desired_joint_target_rad[ARM_JOINT_COUNT];
    float joint_target_rad[ARM_JOINT_COUNT];
    uint32_t consumed_action_sequence;
    uint32_t last_can2_send_time_ms;
    uint32_t dm_first_control_time_ms[ARM_DAMIAO_MOTOR_COUNT];
    uint32_t dm_control_feedback_sequence[ARM_DAMIAO_MOTOR_COUNT];
    uint8_t dm_control_started_mask;
    ArmControlMode_e previous_mode;
    ArmPresetAction_e active_action;
    bool can2_send_initialized;
    bool target_initialized;
    bool control_active;
} ArmTaskControl_t;

/**
 * @brief 机械臂任务实例。
 * @note damiao_motor 直接持有机器人装配实例，不增加通用电机抽象层。
 */
typedef struct
{
    DM_Motor_Info_t damiao_motor[ARM_DAMIAO_MOTOR_COUNT];
    DJI_Motor_Info_t roll3_motor;
    Unitree_Motor_Info_t pitch2_motor;
    ArmTaskPid_t pid;
    ArmJointFeedback_t joint_feedback[ARM_JOINT_COUNT];
    ArmMotorLinkState_e motor_link_state[ARM_JOINT_COUNT];
    ArmTaskControl_t control;
    ArmTaskError_t error;
    volatile ArmTaskDebug_t debug;
} ArmTask_t;

extern ArmTask_t g_arm_task;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 接收 CAN1 中断转发的 roll3 GM6020 反馈帧。
 * @param identifier 经典 CAN 标准标识符。
 * @param data 8 字节反馈数据。
 * @param data_len HAL FDCAN DLC 编码。
 */
void ArmTask_OnCan1Rx(uint32_t identifier,
                       uint8_t *data,
                       uint32_t data_len);

/**
 * @brief 接收 CAN2 中断转发的 Damiao 反馈帧。
 * @param identifier 经典 CAN 标准标识符。
 * @param data 8 字节反馈数据。
 * @param data_len HAL FDCAN DLC 编码。
 */
void ArmTask_OnCan2Rx(uint32_t identifier,
                      uint8_t *data,
                      uint32_t data_len);

/** @brief 接收 USART10 中断转发的 pitch2 GO8010 反馈帧。 */
void ArmTask_OnUsart10Rx(const uint8_t *data, uint16_t data_len);

/**
 * @brief 发布本周期控制命令并唤醒机械臂任务。
 * @param command 输入任务生成的输出许可、模式和动作快照。
 * @note 仅在任务上下文调用；任务尚未启动时通知会被安全忽略。
 */
void ArmTask_Notify(const ArmTaskCommand_t *command);

/**
 * @brief 运行机械臂控制任务生命周期。
 * @note 调用者必须是 armTask 对应的 FreeRTOS 线程；函数正常运行时不返回。
 */
void ArmTask_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* ARM_TASK_H */
