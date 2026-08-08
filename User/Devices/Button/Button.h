/**
 ******************************************************************************
 * @file    Button.h
 * @brief   GPIO 按键消抖、短按边沿和长按状态接口。
 ******************************************************************************
 */

#ifndef BUTTON_H
#define BUTTON_H

#include "stm32h7xx_hal.h"

#include <stdint.h>

#define BUTTON_DEFAULT_LONG_PRESS_MS   (500U)
#define BUTTON_INTERRUPT_QUEUE_LENGTH  (4U)

/**
 * @brief 按键采样后端。
 */
typedef enum
{
    BUTTON_BACKEND_POLLING = 0,
    BUTTON_BACKEND_INTERRUPT
} ButtonBackend_e;

/**
 * @brief 按键硬件和时序配置。
 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState pressed_level;
    uint16_t debounce_ms;
    uint16_t long_press_ms;
    ButtonBackend_e backend;
} ButtonConfig_t;

/**
 * @brief Button_Update() 本周期发布的按键状态。
 *
 * pressed_edge、released_edge 和 long_press_edge 只保持一个更新周期；
 * long_pressed 从达到长按阈值起保持到按键稳定释放。
 */
typedef struct
{
    uint8_t raw_pressed;
    uint8_t pressed;
    uint8_t pressed_edge;
    uint8_t released_edge;
    uint8_t long_pressed;
    uint8_t long_press_edge;
} ButtonState_t;

/**
 * @brief 按键模块实例。
 *
 * @note 配置由创建者持有；其余字段属于 Button.c 的跨周期运行状态，
 *       外部模块只应通过 Button_GetState() 读取已发布状态。
 */
typedef struct
{
    ButtonConfig_t config;
    ButtonState_t state;
    uint8_t candidate_pressed;
    uint8_t candidate_valid;
    uint8_t long_press_reported;
    uint32_t candidate_tick;
    uint32_t press_tick;
    uint8_t interrupt_queue_pressed[BUTTON_INTERRUPT_QUEUE_LENGTH];
    uint32_t interrupt_queue_tick[BUTTON_INTERRUPT_QUEUE_LENGTH];
    uint8_t interrupt_queue_head;
    uint8_t interrupt_queue_tail;
    uint8_t interrupt_queue_count;
} Button_t;

/**
 * @brief 创建轮询按键实例的静态初始化值。
 */
#define BUTTON_CREATE_POLLING(port_ptr, pin_value, pressed_level_value,         \
                              debounce_ms_value, long_press_ms_value)           \
    {                                                                           \
        .config = {                                                             \
            .port = (port_ptr),                                                 \
            .pin = (pin_value),                                                 \
            .pressed_level = (pressed_level_value),                             \
            .debounce_ms = (debounce_ms_value),                                 \
            .long_press_ms = (long_press_ms_value),                             \
            .backend = BUTTON_BACKEND_POLLING,                                  \
        },                                                                      \
        .state = {0},                                                           \
    }

/**
 * @brief 创建双边沿中断按键实例的静态初始化值。
 */
#define BUTTON_CREATE_INTERRUPT(port_ptr, pin_value, pressed_level_value,       \
                                debounce_ms_value, long_press_ms_value)         \
    {                                                                           \
        .config = {                                                             \
            .port = (port_ptr),                                                 \
            .pin = (pin_value),                                                 \
            .pressed_level = (pressed_level_value),                             \
            .debounce_ms = (debounce_ms_value),                                 \
            .long_press_ms = (long_press_ms_value),                             \
            .backend = BUTTON_BACKEND_INTERRUPT,                                \
        },                                                                      \
        .state = {0},                                                           \
    }

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 清空按键实例运行状态。
 */
HAL_StatusTypeDef Button_Init(Button_t *button);

/**
 * @brief 更新按键消抖、边沿和长按状态。
 * @param button 按键实例。
 * @param now_ms 单调递增的毫秒时间。
 */
HAL_StatusTypeDef Button_Update(Button_t *button, uint32_t now_ms);

/**
 * @brief 将 GPIO 双边沿中断采样加入按键实例队列。
 * @note 仅供 BUTTON_BACKEND_INTERRUPT 实例在用户拥有的 EXTI 回调中调用。
 */
HAL_StatusTypeDef Button_NotifyInterrupt(Button_t *button, uint32_t now_ms);

/**
 * @brief 对不使用 Button_t 状态机的简单事件执行时间间隔消抖。
 */
uint8_t Button_DebounceAccept(uint32_t now_ms,
                              uint32_t *last_tick,
                              uint16_t debounce_ms);

/**
 * @brief 读取按键实例最近一次更新发布的状态。
 */
const ButtonState_t *Button_GetState(const Button_t *button);

#ifdef __cplusplus
}
#endif

#endif /* BUTTON_H */
