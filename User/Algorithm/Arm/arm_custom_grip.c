/**
 ******************************************************************************
 * @file    arm_custom_grip.c
 * @brief   自定义控制器夹爪按压方向状态机实现。
 ******************************************************************************
 */

#include "arm_custom_grip.h"

#include <math.h>
#include <stddef.h>

static float arm_custom_grip_clamp(float value,
                                   float minimum,
                                   float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

void ArmCustomGrip_Reset(ArmCustomGripState_t *state)
{
    if (state == NULL)
    {
        return;
    }
    ArmCustomGrip_Disarm(state);
    state->closing_on_next_press = true;
    state->active_direction_closing = true;
}

void ArmCustomGrip_Disarm(ArmCustomGripState_t *state)
{
    if (state == NULL)
    {
        return;
    }
    state->input_online = false;
    state->button_armed = false;
    state->previous_button_pressed = false;
}

bool ArmCustomGrip_Update(ArmCustomGripState_t *state,
                          bool input_online,
                          bool button_pressed,
                          float applied_target_rad,
                          float desired_target_rad,
                          float step_rad,
                          float minimum_target_rad,
                          float maximum_target_rad,
                          float *output_target_rad)
{
    ArmCustomGripState_t next_state;
    float next_target_rad;

    if ((state == NULL) || (output_target_rad == NULL) ||
        !isfinite(applied_target_rad) ||
        !isfinite(desired_target_rad) || !isfinite(step_rad) ||
        !isfinite(minimum_target_rad) ||
        !isfinite(maximum_target_rad) || (step_rad < 0.0f) ||
        (minimum_target_rad > maximum_target_rad))
    {
        return false;
    }

    next_state = *state;
    next_target_rad = desired_target_rad;
    if (!input_online)
    {
        /* 控制器通信短暂掉线只停止运动，保留按钮历史与交替方向。 */
        next_state.input_online = false;
        next_target_rad = applied_target_rad;
    }
    else
    {
        if (!next_state.input_online)
        {
            next_state.input_online = true;
            if (!next_state.button_armed)
            {
                /* 首次进入CUSTOM模式时，带按键进入仍要求先松开。 */
                next_state.button_armed = !button_pressed;
                next_state.previous_button_pressed = button_pressed;
            }
        }

        if (!next_state.button_armed)
        {
            if (!button_pressed)
            {
                next_state.button_armed = true;
            }
            next_state.previous_button_pressed = button_pressed;
        }
        else
        {
            if (button_pressed &&
                !next_state.previous_button_pressed)
            {
                next_state.active_direction_closing =
                    next_state.closing_on_next_press;
                next_state.closing_on_next_press =
                    !next_state.closing_on_next_press;
            }
            next_state.previous_button_pressed = button_pressed;
        }

        if (next_state.button_armed && button_pressed)
        {
            next_target_rad +=
                next_state.active_direction_closing
                    ? -step_rad
                    : step_rad;
        }
    }

    next_target_rad = arm_custom_grip_clamp(
        next_target_rad,
        minimum_target_rad,
        maximum_target_rad);
    *state = next_state;
    *output_target_rad = next_target_rad;
    return true;
}
