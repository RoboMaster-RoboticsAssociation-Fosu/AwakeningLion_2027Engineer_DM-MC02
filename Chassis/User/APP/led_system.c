/** @file led_system.c @brief 底盘绿灯运行指示，保留发送失败后的周期重试。 */
#include "led_system.h"
#include <stdbool.h>

LedSystem_t g_led_system = {.last_send_status = BSP_WS2812_STATUS_OK};
static bool g_led_sent = false;

void LedSystem_Step(uint32_t now_ms)
{
    (void)now_ms;
    if (g_led_sent)
    {
        return;
    }
    g_led_system.last_send_status = BSP_WS2812_SetColor(0U, 255U, 0U);
    if (g_led_system.last_send_status == BSP_WS2812_STATUS_OK)
    {
        g_led_sent = true;
    }
    else
    {
        /* 失败不标记已发送，下一服务周期继续尝试同一绿灯输出。 */
        g_led_system.send_failure_count++;
    }
}
