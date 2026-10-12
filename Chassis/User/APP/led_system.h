/** @file led_system.h @brief 底盘后台服务的绿灯运行指示。 */
#ifndef LED_SYSTEM_H
#define LED_SYSTEM_H

#include "bsp_ws2812.h"
#include <stdint.h>

/** @brief 由 ServiceTask 独占的 LED 发送观测量。 */
typedef struct
{
    BspWs2812Status_e last_send_status;
    uint32_t send_failure_count;
} LedSystem_t;

extern LedSystem_t g_led_system;

#ifdef __cplusplus
extern "C" {
#endif
/** @brief 绿灯常亮；仅首次调用或上次发送失败时访问 SPI6。 */
void LedSystem_Step(uint32_t now_ms);
#ifdef __cplusplus
}
#endif

#endif /* LED_SYSTEM_H */
