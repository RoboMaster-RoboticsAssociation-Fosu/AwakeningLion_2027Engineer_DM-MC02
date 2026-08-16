/**
 ******************************************************************************
 * @file    led_system.h
 * @brief   板载状态灯的故障聚合与灯效系统。
 ******************************************************************************
 */

#ifndef LED_SYSTEM_H
#define LED_SYSTEM_H

#include "bsp_ws2812.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 板载状态灯灯效语义。
 *
 * - 无故障：绿灯常亮，表示 ServiceTask 和 LED 输出正常运行。
 * - 机械臂电机异常：红灯双闪；亮 150 ms、灭 150 ms、再亮 150 ms，
 *   随后灭 900 ms并循环。触发条件为已实装 Damiao 使能/失能确认
 *   失败，或 GM6020、GO8010 及未来控制链中的 Damiao 离线。
 *
 * 多个故障同时存在时，LedSystem 显示优先级最高的故障；优先级相同
 * 时使用固定显示顺序，避免灯效在不同故障之间来回跳变。
 */

/**
 * @brief 可由各任务上报的 LED 故障源。
 * @note 每个故障源只能由其所属模块维护；优先级和灯效由 LedSystem 内部决定。
 */
typedef enum
{
    LED_SYSTEM_FAULT_NONE = 0,
    /** @brief 机械臂电机命令确认失败或被判定为离线。 */
    LED_SYSTEM_FAULT_ARM_MOTOR_ERROR,
    LED_SYSTEM_FAULT_COUNT
} LedSystemFault_e;

/** @brief 当前灯效阶段，供调试器观测。 */
typedef enum
{
    LED_SYSTEM_PHASE_SOLID = 0,
    LED_SYSTEM_PHASE_PULSE_ON,
    LED_SYSTEM_PHASE_PULSE_OFF,
    LED_SYSTEM_PHASE_PULSE_GAP
} LedSystemPhase_e;

/** @brief RGB 输出值。 */
typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} LedSystemColor_t;

/** @brief 多生产者故障聚合状态。 */
typedef struct
{
    volatile uint32_t active_mask;
    LedSystemFault_e displayed_fault;
} LedSystemFaultState_t;

/** @brief LED 系统调试观测量。 */
typedef struct
{
    LedSystemColor_t requested_color;
    LedSystemPhase_e phase;
    BspWs2812Status_e last_send_status;
    uint32_t send_failure_count;
} LedSystemDebug_t;

/** @brief LED 系统跨周期状态。 */
typedef struct
{
    LedSystemFaultState_t fault;
    volatile LedSystemDebug_t debug;
} LedSystem_t;

extern LedSystem_t g_led_system;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 设置或清除一个持续故障状态。
 * @param fault 故障源，不能为 LED_SYSTEM_FAULT_NONE。
 * @param active true 表示故障存在，false 表示故障已恢复。
 * @return 参数有效返回 true，否则返回 false。
 * @note 仅允许在任务上下文调用；重复设置同一状态是幂等操作。
 */
bool LedSystem_SetFault(LedSystemFault_e fault, bool active);

/**
 * @brief 聚合故障并推进非阻塞灯效状态机。
 * @param now_ms 当前毫秒时间戳，允许 uint32_t 自然回绕。
 * @note 只能由 ServiceTask 周期调用；仅在颜色改变或上次发送失败时访问 SPI6。
 */
void LedSystem_Step(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* LED_SYSTEM_H */
