/** @file arm_usb_system.h @brief 从臂 USB 通信 owner；不下发电机命令。 */
#ifndef ARM_USB_SYSTEM_H
#define ARM_USB_SYSTEM_H

#include "arm_usb_protocol.h"

/** @brief Keil 可观测的接收结果与通信计数；只有 ServiceTask 发布这些字段。 */
typedef struct
{
    ArmUsbCommand_t received_command;
    uint16_t command_sequence;
    uint32_t rx_command_count;
    uint32_t rx_rejected_count;
    uint32_t rx_overflow_count;
    uint32_t tx_frame_count;
    uint32_t tx_not_sent_count;
} ArmUsbSystem_t;

extern ArmUsbSystem_t g_arm_usb;

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CDC 初始化/反初始化时丢弃旧流，通知 owner 清理半帧；不改变电机状态。 */
void ArmUsbSystem_OnUsbReset(void);
/** @brief USB 接收回调只复制字节到有界队列；不解析、不保存外部缓冲指针。 */
void ArmUsbSystem_OnUsbRx(const uint8_t *data, uint32_t length);
/** @brief ServiceTask 每 10 ms 处理接收并上传一次最新反馈，USB 忙时跳过本次上传。 */
void ArmUsbSystem_Step(void);

#ifdef __cplusplus
}
#endif
#endif /* ARM_USB_SYSTEM_H */
