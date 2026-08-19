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
#include "ins_task.h"
#include "usart.h"
#include "vofa.h"

#define VOFA_SYSTEM_MAX_CHANNELS 14U

VofaSystem_t g_vofa_system = {
    .requested_view = VOFA_VIEW_ARM_CONTROL_ANGLES,
    .active_view = VOFA_VIEW_ARM_CONTROL_ANGLES,
    .previous_view = VOFA_VIEW_ARM_CONTROL_ANGLES,
};

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

/**
 * @brief 机械臂视图：8 路机构关节角，单位 rad。
 * @note 通道顺序与 ArmJoint_e 完全一致。
 */
static uint16_t vofa_system_fill_arm_joint_angles(void)
{
    uint32_t joint;

    for (joint = 0U; joint < (uint32_t)ARM_JOINT_COUNT; ++joint)
    {
        g_vofa_frame[joint] =
            g_arm_task.joint_feedback[joint].angle_rad;
    }

    return (uint16_t)ARM_JOINT_COUNT;
}

/**
 * @brief 机械臂视图：8 路电机链路状态。
 * @note 通道顺序与 ArmJoint_e 完全一致。数值含义：0 UNKNOWN，
 *       1 ONLINE，2 OFFLINE，3 DM_DISABLED_CONFIRMED，
 *       4 DM_ENABLED_CONFIRMED，5 DM_COMMAND_FAILED。
 */
static uint16_t vofa_system_fill_arm_motor_link_state(void)
{
    uint32_t joint;

    for (joint = 0U; joint < (uint32_t)ARM_JOINT_COUNT; ++joint)
    {
        g_vofa_frame[joint] =
            (float)g_arm_task.motor_link_state[joint];
    }

    return (uint16_t)ARM_JOINT_COUNT;
}

/**
 * @brief Arm control view: calibrated feedback and applied target in rad.
 * @note Channel pairs are big_yaw, pitch1, pitch2, roll2, pitch3, roll3,
 *       grip. The uninstalled roll1 joint is intentionally omitted.
 */
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

/**
 * @brief 底盘视图：每个车轮按目标 rpm、反馈 rpm、C620 电流指令值相邻排列。
 * @note 车轮顺序与 ChassisWheel_e 一致，便于单轮叠图调 PID。
 */
static uint16_t vofa_system_fill_chassis_speed_pid(void)
{
    uint32_t wheel;

    for (wheel = 0U; wheel < (uint32_t)CHASSIS_WHEEL_COUNT; ++wheel)
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

/**
 * @brief 遥控视图：CH0..CH3 与拨轮 iw，均为解析后有符号值。
 */
static uint16_t vofa_system_fill_remote_input(void)
{
    uint32_t channel;

    for (channel = 0U; channel < 4U; ++channel)
    {
        g_vofa_frame[channel] = (float)remote_ctrl.rc.ch[channel];
    }
    g_vofa_frame[4] = (float)remote_ctrl.rc.iw;

    return 5U;
}

/**
 * @brief INS 视图：Roll/Pitch/Yaw、XYZ 角速度与 XYZ 加速度。
 * @note 姿态单位 rad，角速度 rad/s，加速度 m/s^2。
 */
static uint16_t vofa_system_fill_ins_attitude(void)
{
    uint32_t axis;

    g_vofa_frame[0] = INS.Roll;
    g_vofa_frame[1] = INS.Pitch;
    g_vofa_frame[2] = INS.Yaw;
    for (axis = 0U; axis < 3U; ++axis)
    {
        g_vofa_frame[3U + axis] = INS.Gyro[axis];
        g_vofa_frame[6U + axis] = INS.Accel[axis];
    }

    return 9U;
}

bool VofaSystem_SelectView(VofaView_e view)
{
    if ((uint32_t)view >= (uint32_t)VOFA_VIEW_COUNT)
    {
        return false;
    }

    g_vofa_system.requested_view = view;
    return true;
}

void VofaSystem_ReturnView(void)
{
    g_vofa_system.requested_view = g_vofa_system.previous_view;
}

void VofaSystem_Step(void)
{
    HAL_StatusTypeDef send_status;
    VofaView_e requested_view = g_vofa_system.requested_view;
    uint16_t channel_count;

    if ((uint32_t)requested_view >= (uint32_t)VOFA_VIEW_COUNT)
    {
        g_vofa_system.requested_view = g_vofa_system.active_view;
        requested_view = g_vofa_system.active_view;
    }

    if (requested_view != g_vofa_system.active_view)
    {
        g_vofa_system.previous_view = g_vofa_system.active_view;
        g_vofa_system.active_view = requested_view;
    }

    switch (g_vofa_system.active_view)
    {
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
        case VOFA_VIEW_ARM_JOINT_ANGLES:
        default:
            channel_count = vofa_system_fill_arm_joint_angles();
            break;
    }

    send_status = vofa_justfloat_send_it(
        &huart7,
        g_vofa_frame,
        channel_count);
    g_vofa_system.last_send_status = (uint32_t)send_status;
    if (send_status == HAL_BUSY)
    {
        g_vofa_system.busy_drop_count++;
    }
    else if (send_status != HAL_OK)
    {
        g_vofa_system.send_error_count++;
    }
}
