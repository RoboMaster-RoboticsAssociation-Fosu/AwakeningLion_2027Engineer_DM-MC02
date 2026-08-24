/**
 ******************************************************************************
 * @file    vofa_system.c
 * @brief   集中读取各任务调参量并通过 UART7 上传。
 ******************************************************************************
 */

#include "vofa_system.h"

#include "Remote_Control.h"
#include "arm_task.h"
#include "chassis_task.h"
#include "custom_controller_system.h"
#include "ins_task.h"
#include "usart.h"
#include "vofa.h"

/** 手工修改此宏并重新编译，即可选择唯一启用的VOFA通道布局。 */
#define VOFA_ACTIVE_VIEW VOFA_VIEW_ARM_JOINT_ANGLES
#define VOFA_SYSTEM_MAX_CHANNELS 14U

static float g_vofa_frame[VOFA_SYSTEM_MAX_CHANNELS];

static const ArmJoint_e g_vofa_arm_control_joint_order[] = {
    ARM_JOINT_BIG_YAW,
    ARM_JOINT_PITCH1,
    ARM_JOINT_PITCH2,
    ARM_JOINT_ROLL2,
    ARM_JOINT_PITCH3,
    ARM_JOINT_ROLL3,
    ARM_JOINT_GRIP,
};

static uint16_t vofa_system_fill_arm_joint_angles(void)
{
    uint32_t joint;

    for (joint = 0U; joint < (uint32_t)ARM_JOINT_COUNT; joint++)
    {
        g_vofa_frame[joint] =
            g_arm_task.joint_feedback[joint].angle_rad;
    }
    return (uint16_t)ARM_JOINT_COUNT;
}

static uint16_t vofa_system_fill_chassis_speed_pid(void)
{
    uint32_t wheel;

    for (wheel = 0U; wheel < (uint32_t)CHASSIS_WHEEL_COUNT; wheel++)
    {
        uint32_t channel = wheel * 3U;

        g_vofa_frame[channel] =
            g_chassis.wheel_speed_reference_rpm[wheel];
        g_vofa_frame[channel + 1U] =
            (float)g_chassis.wheel_motor[wheel].Data.Velocity;
        g_vofa_frame[channel + 2U] =
            (float)g_chassis.wheel_motor[wheel].Data.SET_Current;
    }
    return (uint16_t)(CHASSIS_WHEEL_COUNT * 3U);
}

static uint16_t vofa_system_fill_remote_input(void)
{
    uint32_t channel;

    for (channel = 0U; channel < 4U; channel++)
    {
        g_vofa_frame[channel] = (float)remote_ctrl.rc.ch[channel];
    }
    g_vofa_frame[4] = (float)remote_ctrl.rc.iw;
    return 5U;
}

static uint16_t vofa_system_fill_ins_attitude(void)
{
    uint32_t axis;

    g_vofa_frame[0] = INS.Roll;
    g_vofa_frame[1] = INS.Pitch;
    g_vofa_frame[2] = INS.Yaw;
    for (axis = 0U; axis < 3U; axis++)
    {
        g_vofa_frame[3U + axis] = INS.Gyro[axis];
        g_vofa_frame[6U + axis] = INS.Accel[axis];
    }
    return 9U;
}

static uint16_t vofa_system_fill_arm_motor_link_state(void)
{
    uint32_t joint;

    for (joint = 0U; joint < (uint32_t)ARM_JOINT_COUNT; joint++)
    {
        g_vofa_frame[joint] =
            (float)g_arm_task.motor_link_state[joint];
    }
    return (uint16_t)ARM_JOINT_COUNT;
}

static uint16_t vofa_system_fill_arm_control_angles(void)
{
    uint32_t index;

    for (index = 0U;
         index < (sizeof(g_vofa_arm_control_joint_order) /
                  sizeof(g_vofa_arm_control_joint_order[0]));
         index++)
    {
        ArmJoint_e joint = g_vofa_arm_control_joint_order[index];
        uint32_t channel = index * 2U;

        g_vofa_frame[channel] =
            g_arm_task.joint_feedback[joint].angle_rad;
        g_vofa_frame[channel + 1U] =
            g_arm_task.control.joint_target_rad[joint];
    }
    return (uint16_t)(sizeof(g_vofa_arm_control_joint_order) /
                      sizeof(g_vofa_arm_control_joint_order[0]) * 2U);
}

static uint16_t vofa_system_fill_custom_controller(void)
{
    CustomControllerSnapshot_t snapshot = {0};
    uint32_t joint;

    CustomControllerSystem_CopySnapshot(&snapshot);
    g_vofa_frame[0] = snapshot.online ? 1.0f : 0.0f;
    for (joint = 0U;
         joint < CUSTOM_CONTROLLER_JOINT_COUNT;
         joint++)
    {
        g_vofa_frame[1U + joint] = snapshot.joint_target_rad[joint];
    }
    g_vofa_frame[1U + CUSTOM_CONTROLLER_JOINT_COUNT] =
        snapshot.button_pressed ? 1.0f : 0.0f;
    return (uint16_t)(CUSTOM_CONTROLLER_JOINT_COUNT + 2U);
}

void VofaSystem_Step(void)
{
    uint16_t channel_count;

    switch ((VofaView_e)VOFA_ACTIVE_VIEW)
    {
        case VOFA_VIEW_ARM_JOINT_ANGLES:
            channel_count = vofa_system_fill_arm_joint_angles();
            break;
        case VOFA_VIEW_CHASSIS_SPEED_PID:
            channel_count = vofa_system_fill_chassis_speed_pid();
            break;
        case VOFA_VIEW_REMOTE_INPUT:
            channel_count = vofa_system_fill_remote_input();
            break;
        case VOFA_VIEW_INS_ATTITUDE:
            channel_count = vofa_system_fill_ins_attitude();
            break;
        case VOFA_VIEW_ARM_MOTOR_LINK_STATE:
            channel_count = vofa_system_fill_arm_motor_link_state();
            break;
        case VOFA_VIEW_ARM_CONTROL_ANGLES:
            channel_count = vofa_system_fill_arm_control_angles();
            break;
        case VOFA_VIEW_CUSTOM_CONTROLLER:
        default:
            channel_count = vofa_system_fill_custom_controller();
            break;
    }

    (void)vofa_justfloat_send_it(
        &huart7,
        g_vofa_frame,
        channel_count);
}
