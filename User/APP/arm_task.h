/**
 * @file arm_task.h
 * @brief 新机械臂：六台 DM 与两台 RS05 的 MIT 运控 owner。
 * @note 输出轴角度 rad，正方向沿用电机；不沿用旧机构零位、预设或 H7 PID。
 */
#ifndef ARM_TASK_H
#define ARM_TASK_H

#include "DM_Motor.h"
#include "RS05_Motor.h"
#include <stdbool.h>
#include <stdint.h>

#define ARM_DAMIAO_MOTOR_COUNT 6U
#define ARM_RS05_MOTOR_COUNT 2U

/** @brief 固定机构顺序同时用于 Keil 调参与 VOFA 通道。 */
typedef enum
{
    ARM_JOINT_YAW = 0,
    ARM_JOINT_PITCH1,
    ARM_JOINT_ROLL1,
    ARM_JOINT_PITCH2,
    ARM_JOINT_ROLL2,
    ARM_JOINT_PITCH3,
    ARM_JOINT_ROLL3,
    ARM_JOINT_GRIP,
    ARM_JOINT_COUNT
} ArmJoint_e;

/** @brief 输入任务只负责输出许可与 RC 掉线的目标冻结。 */
typedef struct
{
    bool enabled;
    bool freeze_targets;
} ArmTaskCommand_t;

/** @brief 电机内部运控增益，初值均为 0，必须通过 Keil 逐轴设置。 */
typedef struct
{
    float kp;
    float kd;
} ArmMitGain_t;

/** @brief owner 在临界区提取的运动反馈，不把厂商不同状态码混为一谈。 */
typedef struct
{
    float angle_rad;
    float velocity_rad_s;
    float torque_nm;
    uint32_t received_time_ms;
    uint32_t sequence;
    uint8_t state;
    bool valid;
    bool fault;
} ArmJointFeedback_t;

typedef enum
{
    ARM_MOTOR_LINK_UNKNOWN = 0,
    ARM_MOTOR_LINK_ONLINE,
    ARM_MOTOR_LINK_OFFLINE,
    ARM_MOTOR_LINK_DISABLED,
    ARM_MOTOR_LINK_ENABLED,
    ARM_MOTOR_LINK_ERROR
} ArmMotorLinkState_e;

typedef enum
{
    ARM_CONTROL_DISABLED = 0,
    ARM_CONTROL_ENABLING,
    ARM_CONTROL_ACTIVE
} ArmControlState_e;

typedef enum
{
    ARM_TASK_ERROR_NONE = 0,
    ARM_TASK_ERROR_MOTOR_LOST,
    ARM_TASK_ERROR_MOTOR_FAULT,
    ARM_TASK_ERROR_NOT_READY,
    ARM_TASK_ERROR_COMMAND_INVALID,
    ARM_TASK_ERROR_SEND_FAILED,
    ARM_TASK_ERROR_ENABLE_TIMEOUT
} ArmTaskError_e;

/** @brief 当前/最近一次停机原因，motor_mask 的 bit 顺序与 ArmJoint_e 一致。 */
typedef struct
{
    ArmTaskError_e type;
    uint16_t motor_mask;
} ArmTaskError_t;

/** @brief Keil 修改目标/增益；发送成功后回写驱动限幅结果，保存目标供 RC 掉线冻结。 */
typedef struct
{
    volatile float joint_target_rad[ARM_JOINT_COUNT];
    volatile ArmMitGain_t mit_gain[ARM_JOINT_COUNT];
    float applied_target_rad[ARM_JOINT_COUNT];
    ArmControlState_e state;
    bool fault_latched;
} ArmTaskControl_t;

/** @brief 固定装配；只有 ArmTask 发送电机命令，CAN ISR 只发布厂商反馈。 */
typedef struct
{
    DM_Motor_Info_t damiao_motor[ARM_DAMIAO_MOTOR_COUNT];
    RS05_Motor_Info_t rs05_motor[ARM_RS05_MOTOR_COUNT];
    ArmJointFeedback_t joint_feedback[ARM_JOINT_COUNT];
    ArmMotorLinkState_e motor_link_state[ARM_JOINT_COUNT];
    ArmTaskControl_t control;
    ArmTaskError_t error;
} ArmTask_t;

extern ArmTask_t g_arm_task;

#ifdef __cplusplus
extern "C" {
#endif

/** @brief InputTask 发布许可快照；任务上下文调用，内部保护跨任务复制。 */
void ArmTask_Notify(const ArmTaskCommand_t *command);
/** @brief CAN1/2/3 的统一接收边界；完整 HAL 帧头用于区分厂商与帧类型。 */
void ArmTask_OnCanRx(FDCAN_HandleTypeDef *hcan,
                      const FDCAN_RxHeaderTypeDef *header,
                      const uint8_t *data, uint32_t data_length);
/** @brief 1 ms 生命周期：输入/反馈快照 → 整臂门控 → 目标 → MIT 发送 → 等待。 */
void ArmTask_Run(void);

#ifdef __cplusplus
}
#endif
#endif /* ARM_TASK_H */
