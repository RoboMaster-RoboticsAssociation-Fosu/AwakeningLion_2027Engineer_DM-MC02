/**
 ******************************************************************************
 * @file    arm_control_filter.h
 * @brief   Mechanical-arm target ramp and feedback jump filter.
 ******************************************************************************
 */

#ifndef ARM_CONTROL_FILTER_H
#define ARM_CONTROL_FILTER_H

#include <stdbool.h>
#include <stdint.h>

/** @brief State of one joint's calibrated-feedback jump filter. */
typedef struct
{
    bool initialized;
    float accepted_angle_rad;
    uint32_t last_sample_token;
    uint32_t rejected_frame_count;
} ArmFeedbackJumpFilter_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Move an angular command toward its target with a bounded rate.
 * @param maximum_step_rad Maximum angle change per update, in rad.
 * @return true on success; false leaves output unchanged.
 */
bool ArmControlFilter_SlewAngle(float current_rad,
                                float desired_rad,
                                float maximum_step_rad,
                                bool wrap_to_pi,
                                float *output_rad);

/**
 * @brief Filter an impossible single-frame angular discontinuity.
 * @note Rejected updates publish the last accepted angle while preserving the
 *       raw driver feedback for diagnostics.
 * @return true when a filtered output is available; false leaves output intact.
 */
bool ArmControlFilter_UpdateFeedback(ArmFeedbackJumpFilter_t *filter,
                                     float candidate_rad,
                                     uint32_t sample_token,
                                     float maximum_jump_rad,
                                     bool wrap_to_pi,
                                     float *output_rad);

/** @brief Invalidate the accepted sample while preserving rejection count. */
void ArmControlFilter_ResetFeedback(ArmFeedbackJumpFilter_t *filter,
                                    uint32_t sample_token);

#ifdef __cplusplus
}
#endif

#endif /* ARM_CONTROL_FILTER_H */
