/**
 ******************************************************************************
 * @file    custom_controller_system.h
 * @brief   UART7自定义控制器接收、在线状态和输入快照。
 ******************************************************************************
 */

#ifndef CUSTOM_CONTROLLER_SYSTEM_H
#define CUSTOM_CONTROLLER_SYSTEM_H

#include "custom_controller_protocol.h"

#include <stdbool.h>
#include <stdint.h>

/** @brief InputTask读取的最近一帧合法自定义控制器快照。 */
typedef struct
{
    bool online;
    uint8_t work_mode;
    float joint_target_rad[CUSTOM_CONTROLLER_JOINT_COUNT];
    bool button_pressed;
    uint32_t valid_frame_sequence;
} CustomControllerSnapshot_t;

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 启动UART7 Receive-to-IDLE中断接收。 */
void CustomControllerSystem_Init(void);

/** @brief 在InputTask中解析缓存、恢复接收并更新200 ms在线状态。 */
void CustomControllerSystem_Process(void);

/** @brief 在任务上下文复制最近一帧合法输入及当前在线状态。 */
void CustomControllerSystem_CopySnapshot(
    CustomControllerSnapshot_t *snapshot);

/** @brief UART7 Receive-to-IDLE回调入口，仅搬运本次接收数据。 */
void CustomControllerSystem_UartRxEvent(uint16_t size);

/** @brief UART7错误回调入口，记录后交由InputTask恢复。 */
void CustomControllerSystem_UartError(void);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_CONTROLLER_SYSTEM_H */
