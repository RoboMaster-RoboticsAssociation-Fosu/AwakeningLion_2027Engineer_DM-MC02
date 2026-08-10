/**
 ******************************************************************************
 * @file    arm_task.h
 * @brief   机械臂任务及其 Damiao 电机装配实例。
 ******************************************************************************
 */

#ifndef ARM_TASK_H
#define ARM_TASK_H

#include "DM_Motor.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 六个 Damiao 电机在机械臂任务中的固定装配顺序。
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
 * @brief Damiao 使能/失能切换状态。
 */
typedef struct
{
    bool requested_enabled;
    uint8_t pending_mask;
    uint8_t next_motor_index;
} ArmDamiaoCommandState_t;

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

/**
 * @brief 机械臂任务实例。
 * @note damiao_motor 直接持有机器人装配实例，不增加通用电机抽象层。
 */
typedef struct
{
    DM_Motor_Info_t damiao_motor[ARM_DAMIAO_MOTOR_COUNT];
    ArmDamiaoCommandState_t command;
    volatile ArmDamiaoDebug_t debug;
} ArmTask_t;

extern ArmTask_t g_arm_task;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 接收 CAN2 中断转发的 Damiao 反馈帧。
 * @param identifier 经典 CAN 标准标识符。
 * @param data 8 字节反馈数据。
 * @param data_len HAL FDCAN DLC 编码。
 */
void ArmTask_OnCan2Rx(uint32_t identifier,
                      uint8_t *data,
                      uint32_t data_len);

/**
 * @brief 发布本周期整车输出许可并唤醒机械臂任务。
 * @param enabled true 逐个使能电机；false 逐个失能电机。
 * @note 仅在任务上下文调用；任务尚未启动时通知会被安全忽略。
 */
void ArmTask_Notify(bool enabled);

/**
 * @brief 运行机械臂控制任务生命周期。
 * @note 调用者必须是 armTask 对应的 FreeRTOS 线程；函数正常运行时不返回。
 */
void ArmTask_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* ARM_TASK_H */
