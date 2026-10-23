/** @file bsp_usb.h @brief 从臂 USB CDC 单帧发送；发送缓冲生命周期由本模块负责。 */
#ifndef BSP_USB_H
#define BSP_USB_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 任务上下文发送一帧；先检查配置/空闲状态，再复制到 CDC 持久缓冲。
 * @return true 表示提交成功；未配置、忙、空指针或非法长度返回 false，不覆盖在途数据。
 * @note 不等待完成、不重试；调用返回后调用者可立即复用 data。
 */
bool USB_Transmit_Data(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif
#endif /* BSP_USB_H */
