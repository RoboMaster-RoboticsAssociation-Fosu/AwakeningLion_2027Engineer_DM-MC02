/**
 ******************************************************************************
 * @file    arm_gravity_compensation.c
 * @brief   Gravity torque model adapted to calibrated mechanism angles.
 ******************************************************************************
 */

#include "arm_gravity_compensation.h"

#include <math.h>
#include <stddef.h>

#define ARM_GRAVITY_ACCELERATION_M_S2       9.8f
#define ARM_GRAVITY_PITCH2_GEAR_RATIO       6.33f
#define ARM_GRAVITY_M2_KG                   1.333f
#define ARM_GRAVITY_M4_KG                   0.59967f
#define ARM_GRAVITY_PITCH1_Q2_OFFSET_RAD    0.261799f
#define ARM_GRAVITY_PITCH1_LEVER_M          1.1728f
#define ARM_GRAVITY_PITCH1_RY2_M           (-0.000476f)
#define ARM_GRAVITY_CHAIN_Q2_OFFSET_RAD     0.366519f
#define ARM_GRAVITY_CHAIN_Q3_OFFSET_RAD     (-0.353786f)
#define ARM_GRAVITY_CHAIN_Q6_OFFSET_RAD     0.25f
#define ARM_GRAVITY_CHAIN_LM34_M            0.30228f
#define ARM_GRAVITY_CHAIN_LM56_M            0.13f
#define ARM_GRAVITY_CHAIN_D45_M             0.3f
#define ARM_GRAVITY_CHAIN_M67_KG            1.4327f
#define ARM_GRAVITY_CHAIN_RY4_M             0.003381f
#define ARM_GRAVITY_PITCH3_Q2_OFFSET_RAD    0.20944f
#define ARM_GRAVITY_PITCH3_Q3_OFFSET_RAD    (-0.079799f)
#define ARM_GRAVITY_PITCH3_Q6_OFFSET_RAD    (-0.05f)
#define ARM_GRAVITY_PITCH3_LEVER_M          0.1525f

typedef struct
{
    float q2;
    float q3;
    float q4;
    float q6;
    float sin_q23;
    float cos_q23;
    float sin_q4;
    float cos_q4;
    float sin_q6;
    float cos_q6;
} ArmGravityChainState_t;

static bool arm_gravity_input_is_finite(const ArmGravityInput_t *input)
{
    return (input != NULL) && isfinite(input->pitch1_rad) &&
           isfinite(input->pitch2_rad) && isfinite(input->roll2_rad) &&
           isfinite(input->pitch3_rad);
}

static void arm_gravity_build_chain(const ArmGravityInput_t *input,
                                    ArmGravityChainState_t *state)
{
    float q23;

    state->q2 = ARM_GRAVITY_CHAIN_Q2_OFFSET_RAD - input->pitch1_rad;
    state->q3 = -input->pitch2_rad + ARM_GRAVITY_CHAIN_Q3_OFFSET_RAD;
    state->q4 = -input->roll2_rad;
    state->q6 = input->pitch3_rad + ARM_GRAVITY_CHAIN_Q6_OFFSET_RAD;
    q23 = state->q2 + state->q3;
    state->sin_q23 = sinf(q23);
    state->cos_q23 = cosf(q23);
    state->sin_q4 = sinf(state->q4);
    state->cos_q4 = cosf(state->q4);
    state->sin_q6 = sinf(state->q6);
    state->cos_q6 = cosf(state->q6);
}

static float arm_gravity_pitch2_torque(
    const ArmGravityChainState_t *state)
{
    return ARM_GRAVITY_ACCELERATION_M_S2 /
           -ARM_GRAVITY_PITCH2_GEAR_RATIO *
           (state->cos_q23 *
                (ARM_GRAVITY_CHAIN_LM34_M +
                 ARM_GRAVITY_CHAIN_LM56_M * state->cos_q6 +
                 ARM_GRAVITY_CHAIN_M67_KG * ARM_GRAVITY_CHAIN_D45_M) +
            state->sin_q23 *
                (ARM_GRAVITY_M4_KG * ARM_GRAVITY_CHAIN_RY4_M *
                     state->sin_q4 +
                 ARM_GRAVITY_CHAIN_LM56_M * state->cos_q4 *
                     state->sin_q6));
}

static float arm_gravity_roll2_torque(
    const ArmGravityChainState_t *state)
{
    return ARM_GRAVITY_ACCELERATION_M_S2 * state->cos_q23 *
           state->sin_q4 * ARM_GRAVITY_CHAIN_LM56_M * state->sin_q6;
}

static float arm_gravity_pitch3_torque(const ArmGravityInput_t *input)
{
    float q2 = ARM_GRAVITY_PITCH3_Q2_OFFSET_RAD + input->pitch1_rad;
    float q3 = input->pitch2_rad + ARM_GRAVITY_PITCH3_Q3_OFFSET_RAD;
    float q4 = -input->roll2_rad;
    float q6 = input->pitch3_rad + ARM_GRAVITY_PITCH3_Q6_OFFSET_RAD;
    float q23 = q2 + q3;

    return ARM_GRAVITY_ACCELERATION_M_S2 * ARM_GRAVITY_PITCH3_LEVER_M *
           (cosf(q6) * cosf(q23) * cosf(q4) - sinf(q6) * sinf(q23));
}

static float arm_gravity_pitch1_torque(
    const ArmGravityInput_t *input,
    float pitch2_torque_nm)
{
    float q2 = ARM_GRAVITY_PITCH1_Q2_OFFSET_RAD - input->pitch1_rad;
    float local_torque_nm =
        (ARM_GRAVITY_PITCH1_LEVER_M * cosf(q2) -
         ARM_GRAVITY_M2_KG * ARM_GRAVITY_PITCH1_RY2_M * sinf(q2)) *
        ARM_GRAVITY_ACCELERATION_M_S2;

    return -(pitch2_torque_nm * ARM_GRAVITY_PITCH2_GEAR_RATIO +
             local_torque_nm);
}

bool ArmGravityCompensation_Calculate(const ArmGravityInput_t *input,
                                      ArmGravityOutput_t *output)
{
    ArmGravityChainState_t state = {0};
    ArmGravityOutput_t result;

    if (!arm_gravity_input_is_finite(input) || (output == NULL))
    {
        return false;
    }

    arm_gravity_build_chain(input, &state);
    result.pitch2_torque_nm = arm_gravity_pitch2_torque(&state);
    result.roll2_torque_nm = arm_gravity_roll2_torque(&state);
    result.pitch3_torque_nm = arm_gravity_pitch3_torque(input);
    result.pitch1_torque_nm = arm_gravity_pitch1_torque(
        input,
        result.pitch2_torque_nm);

    if (!isfinite(result.pitch1_torque_nm) ||
        !isfinite(result.pitch2_torque_nm) ||
        !isfinite(result.roll2_torque_nm) ||
        !isfinite(result.pitch3_torque_nm))
    {
        return false;
    }

    *output = result;
    return true;
}
