/**
 ******************************************************************************
 * @file    arm_gravity_compensation.h
 * @brief   Mechanical-arm gravity compensation in calibrated joint space.
 ******************************************************************************
 */

#ifndef ARM_GRAVITY_COMPENSATION_H
#define ARM_GRAVITY_COMPENSATION_H

#include <stdbool.h>

/**
 * @brief Joint-space input used by the gravity model.
 * @note Every angle is after direction, gear-ratio and zero calibration, in rad.
 */
typedef struct
{
    float pitch1_rad;
    float pitch2_rad;
    float roll2_rad;
    float pitch3_rad;
} ArmGravityInput_t;

/** @brief Motor torque feedforward calculated by the gravity model, in N.m. */
typedef struct
{
    float pitch1_torque_nm;
    float pitch2_torque_nm;
    float roll2_torque_nm;
    float pitch3_torque_nm;
} ArmGravityOutput_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calculate gravity feedforward from calibrated mechanism angles.
 * @param input Current joint-space feedback, in rad.
 * @param output Calculated motor torque feedforward, in N.m.
 * @return true when all inputs and outputs are finite; false leaves output intact.
 */
bool ArmGravityCompensation_Calculate(const ArmGravityInput_t *input,
                                      ArmGravityOutput_t *output);

#ifdef __cplusplus
}
#endif

#endif /* ARM_GRAVITY_COMPENSATION_H */
