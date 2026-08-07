/**
 ******************************************************************************
 * @file    chassis_task.h
 * @brief   M3508 麦轮底盘任务接口。
 ******************************************************************************
 */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "DJI_Motor.h"
#include "pid.h"

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
 * @brief 四轮底盘控制实例。
 */
typedef struct
{
    DJI_Motor_Info_Typedef wheel_motor[CHASSIS_WHEEL_COUNT];
    PidController wheel_speed_pid[CHASSIS_WHEEL_COUNT];
    float wheel_speed_reference_rpm[CHASSIS_WHEEL_COUNT];
} Chassis_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 接收 CAN2 中断转发的四个 M3508 反馈帧。
 * @param identifier 经典 CAN 标准标识符。
 * @param data 8 字节反馈数据。
 * @param data_len HAL FDCAN DLC 编码。
 */
void Chassis_TaskOnCan2Rx(uint32_t identifier,
                          uint8_t *data,
                          uint32_t data_len);

/**
 * @brief 底盘 FreeRTOS owner 任务。
 * @note 遥控器在线且四轮反馈有效时执行速度闭环；任一条件失效时
 *       持续发送零电流。
 */
void chassis_task(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_TASK_H */
