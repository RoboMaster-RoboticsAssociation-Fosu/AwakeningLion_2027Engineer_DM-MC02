/**
 ******************************************************************************
 * @file    chassis_task.h
 * @brief   M3508 麦轮底盘控制任务接口。
 ******************************************************************************
 */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "DJI_Motor.h"
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
 * @brief 四轮底盘任务实例。
 */
typedef struct
{
    DJI_Motor_Info_t wheel_motor[CHASSIS_WHEEL_COUNT];
    PidController_t wheel_speed_pid[CHASSIS_WHEEL_COUNT];
    float wheel_speed_reference_rpm[CHASSIS_WHEEL_COUNT];
} ChassisTask_t;

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

/**
 * @brief 发布本周期整车输出许可并唤醒底盘任务。
 * @param enabled true 允许闭环输出；false 复位 PID 并发送零电流。
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
