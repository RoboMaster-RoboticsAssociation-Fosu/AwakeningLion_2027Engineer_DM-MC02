/**
 ******************************************************************************
 * @file    Button.c
 * @brief   GPIO 按键轮询/中断采样、消抖和长按识别。
 ******************************************************************************
 */

#include "Button.h"

#include <stddef.h>
#include <string.h>

static HAL_StatusTypeDef Button_Validate(const Button_t *button)
{
    if ((button == NULL) || (button->config.port == NULL))
    {
        return HAL_ERROR;
    }

    if ((button->config.backend != BUTTON_BACKEND_POLLING) &&
        (button->config.backend != BUTTON_BACKEND_INTERRUPT))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static uint8_t Button_ReadPressed(const Button_t *button)
{
    return (HAL_GPIO_ReadPin(button->config.port,
                            button->config.pin) ==
            button->config.pressed_level)
               ? 1U
               : 0U;
}

static void Button_ClearEdges(Button_t *button)
{
    button->state.pressed_edge = 0U;
    button->state.released_edge = 0U;
    button->state.long_press_edge = 0U;
}

static void Button_ClearCandidate(Button_t *button)
{
    button->candidate_valid = 0U;
}

static void Button_OnStableStateChanged(Button_t *button,
                                        uint8_t pressed,
                                        uint32_t now_ms)
{
    button->state.pressed = pressed;
    Button_ClearCandidate(button);

    if (pressed != 0U)
    {
        button->state.pressed_edge = 1U;
        button->press_tick = now_ms;
        button->long_press_reported = 0U;
        button->state.long_pressed = 0U;
    }
    else
    {
        button->state.released_edge = 1U;
        button->state.long_pressed = 0U;
        button->press_tick = 0U;
        button->long_press_reported = 0U;
    }
}

static void Button_InterruptQueuePush(Button_t *button,
                                      uint8_t sampled_pressed,
                                      uint32_t sample_tick)
{
    uint8_t tail;

    if (button->interrupt_queue_count >=
        BUTTON_INTERRUPT_QUEUE_LENGTH)
    {
        button->interrupt_queue_head =
            (uint8_t)((button->interrupt_queue_head + 1U) %
                      BUTTON_INTERRUPT_QUEUE_LENGTH);
        button->interrupt_queue_count--;
    }

    tail = button->interrupt_queue_tail;
    button->interrupt_queue_pressed[tail] = sampled_pressed;
    button->interrupt_queue_tick[tail] = sample_tick;
    button->interrupt_queue_tail =
        (uint8_t)((tail + 1U) % BUTTON_INTERRUPT_QUEUE_LENGTH);
    button->interrupt_queue_count++;
}

static uint8_t Button_InterruptQueuePeek(const Button_t *button,
                                         uint8_t *sampled_pressed,
                                         uint32_t *sample_tick)
{
    uint8_t head;

    if ((button == NULL) || (sampled_pressed == NULL) ||
        (sample_tick == NULL) ||
        (button->interrupt_queue_count == 0U))
    {
        return 0U;
    }

    head = button->interrupt_queue_head;
    *sampled_pressed = button->interrupt_queue_pressed[head];
    *sample_tick = button->interrupt_queue_tick[head];
    return 1U;
}

static void Button_InterruptQueuePop(Button_t *button)
{
    if ((button == NULL) ||
        (button->interrupt_queue_count == 0U))
    {
        return;
    }

    button->interrupt_queue_head =
        (uint8_t)((button->interrupt_queue_head + 1U) %
                  BUTTON_INTERRUPT_QUEUE_LENGTH);
    button->interrupt_queue_count--;
}

static void Button_TryCommitCandidate(Button_t *button,
                                      uint32_t reference_tick)
{
    const uint32_t stable_tick =
        button->candidate_tick + button->config.debounce_ms;

    if (button->candidate_valid == 0U)
    {
        return;
    }

    if (button->candidate_pressed == button->state.pressed)
    {
        Button_ClearCandidate(button);
        return;
    }

    if ((uint32_t)(reference_tick - button->candidate_tick) <
        button->config.debounce_ms)
    {
        return;
    }

    Button_OnStableStateChanged(button,
                                button->candidate_pressed,
                                stable_tick);
}

static void Button_ProcessSample(Button_t *button,
                                 uint8_t sampled_pressed,
                                 uint32_t sample_origin_tick)
{
    button->state.raw_pressed = sampled_pressed;

    if (sampled_pressed == button->state.pressed)
    {
        Button_ClearCandidate(button);
        return;
    }

    if ((button->candidate_valid == 0U) ||
        (button->candidate_pressed != sampled_pressed))
    {
        button->candidate_pressed = sampled_pressed;
        button->candidate_tick = sample_origin_tick;
        button->candidate_valid = 1U;
    }
}

static void Button_UpdateLongPress(Button_t *button,
                                   uint32_t now_ms)
{
    if ((button->state.pressed != 0U) &&
        (button->long_press_reported == 0U) &&
        ((uint32_t)(now_ms - button->press_tick) >=
         button->config.long_press_ms))
    {
        button->state.long_pressed = 1U;
        button->state.long_press_edge = 1U;
        button->long_press_reported = 1U;
    }
}

HAL_StatusTypeDef Button_Init(Button_t *button)
{
    if (Button_Validate(button) != HAL_OK)
    {
        return HAL_ERROR;
    }

    button->state.raw_pressed = 0U;
    button->state.pressed = 0U;
    button->state.pressed_edge = 0U;
    button->state.released_edge = 0U;
    button->state.long_pressed = 0U;
    button->state.long_press_edge = 0U;
    button->candidate_pressed = 0U;
    button->candidate_valid = 0U;
    button->long_press_reported = 0U;
    button->candidate_tick = 0U;
    button->press_tick = 0U;
    memset(button->interrupt_queue_pressed,
           0,
           sizeof(button->interrupt_queue_pressed));
    memset(button->interrupt_queue_tick,
           0,
           sizeof(button->interrupt_queue_tick));
    button->interrupt_queue_head = 0U;
    button->interrupt_queue_tail = 0U;
    button->interrupt_queue_count = 0U;

    return HAL_OK;
}

HAL_StatusTypeDef Button_Update(Button_t *button,
                                uint32_t now_ms)
{
    if (Button_Validate(button) != HAL_OK)
    {
        return HAL_ERROR;
    }

    Button_ClearEdges(button);

    if (button->config.backend == BUTTON_BACKEND_POLLING)
    {
        Button_ProcessSample(button,
                             Button_ReadPressed(button),
                             now_ms);
        Button_TryCommitCandidate(button, now_ms);
    }
    else
    {
        uint8_t sampled_pressed = 0U;
        uint32_t sample_tick = 0U;

        while (Button_InterruptQueuePeek(button,
                                         &sampled_pressed,
                                         &sample_tick) != 0U)
        {
            Button_TryCommitCandidate(button, sample_tick);
            Button_ProcessSample(button,
                                 sampled_pressed,
                                 sample_tick);
            Button_InterruptQueuePop(button);
        }

        Button_TryCommitCandidate(button, now_ms);
    }

    Button_UpdateLongPress(button, now_ms);
    return HAL_OK;
}

HAL_StatusTypeDef Button_NotifyInterrupt(Button_t *button,
                                         uint32_t now_ms)
{
    uint8_t previous_raw_pressed;
    uint8_t sampled_pressed;

    if ((Button_Validate(button) != HAL_OK) ||
        (button->config.backend != BUTTON_BACKEND_INTERRUPT))
    {
        return HAL_ERROR;
    }

    previous_raw_pressed = button->state.raw_pressed;
    sampled_pressed = Button_ReadPressed(button);
    button->state.raw_pressed = sampled_pressed;

    if (sampled_pressed == previous_raw_pressed)
    {
        return HAL_OK;
    }

    Button_InterruptQueuePush(button, sampled_pressed, now_ms);
    return HAL_OK;
}

uint8_t Button_DebounceAccept(uint32_t now_ms,
                              uint32_t *last_tick,
                              uint16_t debounce_ms)
{
    if (last_tick == NULL)
    {
        return 0U;
    }

    if ((uint32_t)(now_ms - *last_tick) < debounce_ms)
    {
        return 0U;
    }

    *last_tick = now_ms;
    return 1U;
}

const ButtonState_t *Button_GetState(const Button_t *button)
{
    if (button == NULL)
    {
        return NULL;
    }

    return &button->state;
}
