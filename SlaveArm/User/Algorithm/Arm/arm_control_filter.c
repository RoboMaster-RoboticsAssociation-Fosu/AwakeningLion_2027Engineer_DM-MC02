/**
 ******************************************************************************
 * @file    arm_control_filter.c
 * @brief   Mechanical-arm target ramp and feedback jump filter.
 ******************************************************************************
 */

#include "arm_control_filter.h"

#include <math.h>
#include <stddef.h>

#define ARM_CONTROL_FILTER_PI_RAD      3.14159265358979323846f
#define ARM_CONTROL_FILTER_TWO_PI_RAD  (2.0f * ARM_CONTROL_FILTER_PI_RAD)

static float arm_control_filter_wrap_to_pi(float angle_rad)
{
    angle_rad = fmodf(
        angle_rad + ARM_CONTROL_FILTER_PI_RAD,
        ARM_CONTROL_FILTER_TWO_PI_RAD);
    if (angle_rad < 0.0f)
    {
        angle_rad += ARM_CONTROL_FILTER_TWO_PI_RAD;
    }
    return angle_rad - ARM_CONTROL_FILTER_PI_RAD;
}

bool ArmControlFilter_SlewAngle(float current_rad,
                                float desired_rad,
                                float maximum_step_rad,
                                bool wrap_to_pi,
                                float *output_rad)
{
    float delta_rad;
    float result_rad;

    if ((output_rad == NULL) || !isfinite(current_rad) ||
        !isfinite(desired_rad) || !isfinite(maximum_step_rad) ||
        (maximum_step_rad < 0.0f))
    {
        return false;
    }

    delta_rad = desired_rad - current_rad;
    if (wrap_to_pi)
    {
        delta_rad = arm_control_filter_wrap_to_pi(delta_rad);
    }
    if (delta_rad > maximum_step_rad)
    {
        delta_rad = maximum_step_rad;
    }
    else if (delta_rad < -maximum_step_rad)
    {
        delta_rad = -maximum_step_rad;
    }

    result_rad = current_rad + delta_rad;
    *output_rad = wrap_to_pi
                      ? arm_control_filter_wrap_to_pi(result_rad)
                      : result_rad;
    return true;
}

bool ArmControlFilter_UpdateFeedback(ArmFeedbackJumpFilter_t *filter,
                                     float candidate_rad,
                                     uint32_t sample_token,
                                     float maximum_jump_rad,
                                     bool wrap_to_pi,
                                     float *output_rad)
{
    float jump_rad;

    if ((filter == NULL) || (output_rad == NULL) ||
        !isfinite(maximum_jump_rad) || (maximum_jump_rad <= 0.0f))
    {
        return false;
    }

    if (filter->initialized &&
        (filter->last_sample_token == sample_token))
    {
        *output_rad = filter->accepted_angle_rad;
        return true;
    }

    filter->last_sample_token = sample_token;
    if (!isfinite(candidate_rad))
    {
        filter->rejected_frame_count++;
        if (!filter->initialized)
        {
            return false;
        }
        *output_rad = filter->accepted_angle_rad;
        return true;
    }

    if (!filter->initialized)
    {
        filter->initialized = true;
        filter->accepted_angle_rad = candidate_rad;
        *output_rad = candidate_rad;
        return true;
    }

    jump_rad = candidate_rad - filter->accepted_angle_rad;
    if (wrap_to_pi)
    {
        jump_rad = arm_control_filter_wrap_to_pi(jump_rad);
    }
    if (fabsf(jump_rad) > maximum_jump_rad)
    {
        filter->rejected_frame_count++;
        *output_rad = filter->accepted_angle_rad;
        return true;
    }

    filter->accepted_angle_rad = candidate_rad;
    *output_rad = candidate_rad;
    return true;
}

void ArmControlFilter_ResetFeedback(ArmFeedbackJumpFilter_t *filter,
                                    uint32_t sample_token)
{
    if (filter == NULL)
    {
        return;
    }
    filter->initialized = false;
    filter->last_sample_token = sample_token;
}
