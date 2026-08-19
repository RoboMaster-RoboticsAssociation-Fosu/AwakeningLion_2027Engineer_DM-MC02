/**
 ******************************************************************************
 * @file    vofa_system.h
 * @brief   VOFA 调参视图选择与周期上传接口。
 ******************************************************************************
 */

#ifndef VOFA_SYSTEM_H
#define VOFA_SYSTEM_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 第一版 VOFA 调参视图。
 * @note 视图只改变上传通道的含义，不修改任何控制状态。
 */
typedef enum
{
    VOFA_VIEW_ARM_JOINT_ANGLES = 0,
    VOFA_VIEW_CHASSIS_SPEED_PID,
    VOFA_VIEW_REMOTE_INPUT,
    VOFA_VIEW_INS_ATTITUDE,
    VOFA_VIEW_ARM_MOTOR_LINK_STATE,
    VOFA_VIEW_ARM_CONTROL_ANGLES,
    VOFA_VIEW_COUNT
} VofaView_e;

/**
 * @brief VOFA 视图选择与发送诊断状态。
 * @note 调试器可改写 requested_view 切换视图；其余字段只读。
 */
typedef struct
{
    volatile VofaView_e requested_view;
    VofaView_e active_view;
    VofaView_e previous_view;
    uint32_t last_send_status;
    uint32_t busy_drop_count;
    uint32_t send_error_count;
} VofaSystem_t;

extern VofaSystem_t g_vofa_system;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 请求切换调参视图，由下一次 VofaSystem_Step() 生效。
 * @return true 表示视图有效且已接受请求。
 * @note 仅从任务上下文调用。
 */
bool VofaSystem_SelectView(VofaView_e view);

/**
 * @brief 请求返回上一个调参视图。
 * @note 仅从任务上下文调用。
 */
void VofaSystem_ReturnView(void);

/**
 * @brief 读取当前视图的任务内部量并通过 UART7 非阻塞上传。
 * @note 由 ServiceTask 每 10 ms 调用；UART 忙时丢弃本帧。
 */
void VofaSystem_Step(void);

#ifdef __cplusplus
}
#endif

#endif /* VOFA_SYSTEM_H */
