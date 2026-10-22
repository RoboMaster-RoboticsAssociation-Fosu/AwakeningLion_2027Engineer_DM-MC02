/**
 ******************************************************************************
 * @file    chassis_task.h
 * @brief   六台 M3508、四台 DM-J4340 的底盘控制与逐台调试接口。
 ******************************************************************************
 */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "pid.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 底盘车轮在控制数组中的固定顺序。
 */
typedef enum
{
    CHASSIS_WHEEL_RIGHT_FRONT = 0,
    CHASSIS_WHEEL_LEFT_FRONT,
    CHASSIS_WHEEL_LEFT_BACK,
    CHASSIS_WHEEL_RIGHT_BACK,
    CHASSIS_WHEEL_COUNT
} ChassisWheel_e;

/**
 * @brief CAN3 履带电机顺序；两侧 C620 的硬件 ID 分别为 1、2。
 */
typedef enum
{
    CHASSIS_TRACK_LEFT = 0,
    CHASSIS_TRACK_RIGHT,
    CHASSIS_TRACK_COUNT
} ChassisTrack_e;

/** @brief CAN2 单连杆腿顺序，与四个麦轮的位置对应。 */
typedef enum
{
    CHASSIS_LEG_RIGHT_FRONT = 0,
    CHASSIS_LEG_LEFT_FRONT,
    CHASSIS_LEG_LEFT_BACK,
    CHASSIS_LEG_RIGHT_BACK,
    CHASSIS_LEG_COUNT
} ChassisLeg_e;

/** @brief 调试器电机编号；0 不选择电机，同一时刻只运行一个编号。 */
typedef enum
{
    CHASSIS_TEST_MOTOR_NONE = 0,
    CHASSIS_TEST_WHEEL_RIGHT_FRONT,
    CHASSIS_TEST_WHEEL_LEFT_FRONT,
    CHASSIS_TEST_WHEEL_LEFT_BACK,
    CHASSIS_TEST_WHEEL_RIGHT_BACK,
    CHASSIS_TEST_TRACK_LEFT,
    CHASSIS_TEST_TRACK_RIGHT,
    CHASSIS_TEST_LEG_RIGHT_FRONT,
    CHASSIS_TEST_LEG_LEFT_FRONT,
    CHASSIS_TEST_LEG_LEFT_BACK,
    CHASSIS_TEST_LEG_RIGHT_BACK,
    CHASSIS_TEST_MOTOR_COUNT
} ChassisTestMotor_e;

/** @brief 调试入口的当前结果；发送成功不代表实机已执行运动。 */
typedef enum
{
    CHASSIS_TEST_IDLE = 0,
    CHASSIS_TEST_REMOTE_LOCKED,
    CHASSIS_TEST_WAIT_FEEDBACK,
    CHASSIS_TEST_ENABLING,
    CHASSIS_TEST_RUNNING,
    CHASSIS_TEST_STOPPING,
    CHASSIS_TEST_INVALID_COMMAND,
    CHASSIS_TEST_FEEDBACK_LOST,
    CHASSIS_TEST_MOTOR_FAULT,
    CHASSIS_TEST_SEND_FAILED
} ChassisTestStatus_e;

/* BEGIN 首版调试器入口：仅用于逐台验证总线、速度环与腿部角度 PD。
 * 编辑顺序：run=0 -> 修改 motor -> 等待停止 -> run=1 -> 修改目标。
 * 正式遥控/姿态目标接入并验证后移除此入口；不作为未来控制协议预留层。
 */
typedef struct
{
    uint32_t motor;      /* ChassisTestMotor_e；切换电机自动清除 run。 */
    uint32_t run;        /* 0 停止，1 请求测试；还必须取得遥控输出许可。 */
    float speed_rpm;    /* M3508 减速前电机转速，带符号 rpm。 */
    float position_deg; /* J4340 输出轴绝对角度，deg；不增加机械行程限位。 */
    float leg_kp;       /* H7 位置增益，N.m/rad；不是 MIT 报文中的 kp。 */
    float leg_kd;       /* H7 阻尼增益，N.m.s/rad；不是 MIT 报文中的 kd。 */
} ChassisTestCommand_t;
/* END 首版调试器入口。 */

/**
 * @brief 底盘固定实例；各 CAN 反馈仅由对应 ISR 更新，控制由 ChassisTask 持有。
 */
typedef struct
{
    DJI_Motor_Info_t wheel_motor[CHASSIS_WHEEL_COUNT];
    PidController_t wheel_speed_pid[CHASSIS_WHEEL_COUNT];
    float wheel_speed_reference_rpm[CHASSIS_WHEEL_COUNT];
    DJI_Motor_Info_t track_motor[CHASSIS_TRACK_COUNT];
    PidController_t track_speed_pid[CHASSIS_TRACK_COUNT];
    float track_speed_reference_rpm[CHASSIS_TRACK_COUNT];
    DM_Motor_Info_t leg_motor[CHASSIS_LEG_COUNT];
    /* 仅在 MIT 控制帧成功入队后更新，含驱动协议限幅，不代表实测力矩。 */
    float leg_submitted_torque_nm[CHASSIS_LEG_COUNT];
    volatile ChassisTestCommand_t command; /* 调试器异步输入；不是线程同步锁。 */
    uint32_t active_test_motor;            /* 只读观测；停止确认期间仍保留旧编号。 */
    ChassisTestStatus_e test_status;       /* 只读观测。 */
} ChassisTask_t;

/**
 * @brief 底盘任务实例。
 * @note command 供调试器编辑；反馈由 ISR 发布；其余字段由 ChassisTask 写入。
 */
extern ChassisTask_t g_chassis;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 接收 CAN1 中断转发的四个 M3508 反馈帧。
 * @param identifier 经典 CAN 标准标识符。
 * @param data 8 字节反馈数据。
 * @param data_len HAL FDCAN DLC 编码。
 */
void ChassisTask_OnCan1Rx(uint32_t identifier,
                          uint8_t *data,
                          uint32_t data_len);

/** @brief 分发 CAN2 的 J4340 反馈；校验反馈 CAN ID 和负载内电机 ID。 */
void ChassisTask_OnCan2Rx(uint32_t identifier,
                          const uint8_t *data,
                          uint32_t data_len);

/** @brief 分发 CAN3 的左右履带 M3508 反馈，不与 CAN1 相同 ID 混用。 */
void ChassisTask_OnCan3Rx(uint32_t identifier,
                          uint8_t *data,
                          uint32_t data_len);

/**
 * @brief 发布本周期整车输出许可并唤醒底盘任务。
 * @param enabled true 允许闭环输出；false 清测试运行请求、零电流并失能腿电机。
 * @note 仅在任务上下文调用；任务尚未启动时通知会被安全忽略。
 */
void ChassisTask_Notify(bool enabled);

/**
 * @brief 运行底盘控制任务生命周期。
 * @note 调用者必须是 chassisTask 对应的 FreeRTOS 线程；函数正常运行时不返回。
 */
void ChassisTask_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_TASK_H */
