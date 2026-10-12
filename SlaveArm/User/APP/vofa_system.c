/** @file vofa_system.c @brief 读取本板实际启用的控制任务，UART7 只承担 VOFA TX。 */
#include "vofa_system.h"
#include "Remote_Control.h"
#include "ins_types.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "vofa.h"
#include "arm_task.h"
#ifndef VOFA_ACTIVE_VIEW
#define VOFA_ACTIVE_VIEW VOFA_VIEW_ARM_JOINT_ANGLES
#endif

static float g_vofa_frame[16];

void VofaSystem_Step(void)
{
    uint16_t channel_count = 0U;
    uint32_t index;

    /* 与反馈 ISR/INS 快照发布同步；锁内只复制数值，发送不持锁。 */
    taskENTER_CRITICAL();
    switch ((VofaView_e)VOFA_ACTIVE_VIEW)
    {
        case VOFA_VIEW_ARM_JOINT_ANGLES:
            for (index = 0U; index < ARM_JOINT_COUNT; index++)
            {
                g_vofa_frame[index] = g_arm_task.joint_feedback[index].angle_rad;
            }
            channel_count = ARM_JOINT_COUNT;
            break;
        case VOFA_VIEW_ARM_MOTOR_LINK_STATE:
            for (index = 0U; index < ARM_JOINT_COUNT; index++)
            {
                g_vofa_frame[index] = (float)g_arm_task.motor_link_state[index];
            }
            channel_count = ARM_JOINT_COUNT;
            break;
        case VOFA_VIEW_ARM_CONTROL_ANGLES:
            for (index = 0U; index < ARM_JOINT_COUNT; index++)
            {
                g_vofa_frame[index * 2U] = g_arm_task.joint_feedback[index].angle_rad;
                g_vofa_frame[index * 2U + 1U] = g_arm_task.control.applied_target_rad[index];
            }
            channel_count = ARM_JOINT_COUNT * 2U;
            break;
        case VOFA_VIEW_REMOTE_INPUT:
            for (index = 0U; index < 4U; index++)
            {
                g_vofa_frame[index] = (float)remote_ctrl.rc.ch[index];
            }
            g_vofa_frame[4] = (float)remote_ctrl.rc.iw;
            channel_count = 5U;
            break;
        case VOFA_VIEW_INS_ATTITUDE:
            g_vofa_frame[0] = INS.Roll;
            g_vofa_frame[1] = INS.Pitch;
            g_vofa_frame[2] = INS.Yaw;
            for (index = 0U; index < 3U; index++)
            {
                g_vofa_frame[3U + index] = INS.Gyro[index];
                g_vofa_frame[6U + index] = INS.Accel[index];
            }
            channel_count = 9U;
            break;
        default:
            /* 未支持的视图没有数据，禁止发送伪造的零快照。 */
            break;
    }
    taskEXIT_CRITICAL();
    if (channel_count != 0U)
    {
        (void)vofa_justfloat_send_it(&huart7, g_vofa_frame, channel_count);
    }
}
