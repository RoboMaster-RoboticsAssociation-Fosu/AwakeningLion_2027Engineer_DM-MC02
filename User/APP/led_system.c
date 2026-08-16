/**
 ******************************************************************************
 * @file    led_system.c
 * @brief   板载状态灯的故障聚合与非阻塞灯效状态机。
 ******************************************************************************
 */

#include "led_system.h"

#include "FreeRTOS.h"
#include "task.h"

#include <limits.h>

#define LED_SYSTEM_SHORT_ON_MS         150U
#define LED_SYSTEM_SHORT_OFF_MS        150U
#define LED_SYSTEM_DOUBLE_FLASH_COUNT  2U
#define LED_SYSTEM_DOUBLE_FLASH_GAP_MS 900U

typedef struct
{
    uint8_t priority;
    uint8_t display_rank;
    LedSystemColor_t color;
    uint16_t on_ms;
    uint16_t off_ms;
    uint16_t gap_ms;
    uint8_t pulse_count;
} LedSystemFaultDescriptor_t;

static const LedSystemFaultDescriptor_t
    g_led_fault_descriptors[LED_SYSTEM_FAULT_COUNT] = {
        [LED_SYSTEM_FAULT_NONE] = {
            .priority = 0U,
            .display_rank = 0U,
            .color = {
                .red = 0U,
                .green = 255U,
                .blue = 0U,
            },
            .on_ms = 0U,
            .off_ms = 0U,
            .gap_ms = 0U,
            .pulse_count = 0U,
        },
        [LED_SYSTEM_FAULT_ARM_MOTOR_ERROR] = {
            .priority = 100U,
            .display_rank = 0U,
            .color = {
                .red = 255U,
                .green = 0U,
                .blue = 0U,
            },
            .on_ms = LED_SYSTEM_SHORT_ON_MS,
            .off_ms = LED_SYSTEM_SHORT_OFF_MS,
            .gap_ms = LED_SYSTEM_DOUBLE_FLASH_GAP_MS,
            .pulse_count = LED_SYSTEM_DOUBLE_FLASH_COUNT,
        },
    };

LedSystem_t g_led_system = {
    .fault = {
        .active_mask = 0U,
        .displayed_fault = LED_SYSTEM_FAULT_NONE,
    },
    .debug = {
        .requested_color = {.red = 0U, .green = 0U, .blue = 0U},
        .phase = LED_SYSTEM_PHASE_SOLID,
        .last_send_status = BSP_WS2812_STATUS_OK,
        .send_failure_count = 0U,
    },
};

static uint32_t g_led_pattern_start_ms = 0U;
static LedSystemColor_t g_led_sent_color = {0U, 0U, 0U};
static bool g_led_sent_color_valid = false;

static uint32_t led_system_fault_bit(LedSystemFault_e fault)
{
    return 1UL << ((uint32_t)fault - 1U);
}

static LedSystemFault_e led_system_select_fault(uint32_t active_mask)
{
    LedSystemFault_e selected = LED_SYSTEM_FAULT_NONE;
    uint8_t selected_priority = 0U;
    uint8_t selected_rank = UINT8_MAX;
    uint32_t fault_value;

    for (fault_value = (uint32_t)LED_SYSTEM_FAULT_NONE + 1U;
         fault_value < (uint32_t)LED_SYSTEM_FAULT_COUNT;
         fault_value++)
    {
        LedSystemFault_e fault = (LedSystemFault_e)fault_value;
        const LedSystemFaultDescriptor_t *descriptor =
            &g_led_fault_descriptors[fault_value];

        if ((active_mask & led_system_fault_bit(fault)) == 0U)
        {
            continue;
        }

        if ((selected == LED_SYSTEM_FAULT_NONE) ||
            (descriptor->priority > selected_priority) ||
            ((descriptor->priority == selected_priority) &&
             (descriptor->display_rank < selected_rank)))
        {
            selected = fault;
            selected_priority = descriptor->priority;
            selected_rank = descriptor->display_rank;
        }
    }

    return selected;
}

static LedSystemColor_t led_system_pattern_color(
    const LedSystemFaultDescriptor_t *descriptor,
    uint32_t elapsed_ms,
    LedSystemPhase_e *phase)
{
    LedSystemColor_t color = {0U, 0U, 0U};
    uint32_t cycle_ms;
    uint32_t position_ms;
    uint8_t pulse;

    cycle_ms = (uint32_t)descriptor->pulse_count * descriptor->on_ms +
               (uint32_t)(descriptor->pulse_count - 1U) * descriptor->off_ms +
               descriptor->gap_ms;
    position_ms = elapsed_ms % cycle_ms;

    for (pulse = 0U; pulse < descriptor->pulse_count; pulse++)
    {
        if (position_ms < descriptor->on_ms)
        {
            *phase = LED_SYSTEM_PHASE_PULSE_ON;
            return descriptor->color;
        }
        position_ms -= descriptor->on_ms;

        if (pulse < (uint8_t)(descriptor->pulse_count - 1U))
        {
            if (position_ms < descriptor->off_ms)
            {
                *phase = LED_SYSTEM_PHASE_PULSE_OFF;
                return color;
            }
            position_ms -= descriptor->off_ms;
        }
    }

    *phase = LED_SYSTEM_PHASE_PULSE_GAP;
    return color;
}

static bool led_system_color_equal(const LedSystemColor_t *left,
                                   const LedSystemColor_t *right)
{
    return (left->red == right->red) &&
           (left->green == right->green) &&
           (left->blue == right->blue);
}

bool LedSystem_SetFault(LedSystemFault_e fault, bool active)
{
    uint32_t fault_bit;

    if ((fault <= LED_SYSTEM_FAULT_NONE) ||
        (fault >= LED_SYSTEM_FAULT_COUNT))
    {
        return false;
    }

    fault_bit = led_system_fault_bit(fault);
    taskENTER_CRITICAL();
    if (active)
    {
        g_led_system.fault.active_mask |= fault_bit;
    }
    else
    {
        g_led_system.fault.active_mask &= ~fault_bit;
    }
    taskEXIT_CRITICAL();

    return true;
}

void LedSystem_Step(uint32_t now_ms)
{
    LedSystemFault_e selected_fault;
    LedSystemColor_t desired_color;
    LedSystemPhase_e phase;
    uint32_t active_mask;
    BspWs2812Status_e send_status;

    taskENTER_CRITICAL();
    active_mask = g_led_system.fault.active_mask;
    taskEXIT_CRITICAL();

    selected_fault = led_system_select_fault(active_mask);
    if (selected_fault != g_led_system.fault.displayed_fault)
    {
        g_led_system.fault.displayed_fault = selected_fault;
        g_led_pattern_start_ms = now_ms;
    }

    if (selected_fault == LED_SYSTEM_FAULT_NONE)
    {
        desired_color =
            g_led_fault_descriptors[LED_SYSTEM_FAULT_NONE].color;
        phase = LED_SYSTEM_PHASE_SOLID;
    }
    else
    {
        desired_color = led_system_pattern_color(
            &g_led_fault_descriptors[selected_fault],
            now_ms - g_led_pattern_start_ms,
            &phase);
    }

    g_led_system.debug.requested_color = desired_color;
    g_led_system.debug.phase = phase;

    if (g_led_sent_color_valid &&
        led_system_color_equal(&desired_color, &g_led_sent_color))
    {
        return;
    }

    send_status = BSP_WS2812_SetColor(desired_color.red,
                                      desired_color.green,
                                      desired_color.blue);
    g_led_system.debug.last_send_status = send_status;
    if (send_status == BSP_WS2812_STATUS_OK)
    {
        g_led_sent_color = desired_color;
        g_led_sent_color_valid = true;
    }
    else
    {
        g_led_sent_color_valid = false;
        g_led_system.debug.send_failure_count++;
    }
}
