/**
 ******************************************************************************
 * @file    chassis_task.c
 * @brief   四 M3508 麦轮底盘遥控速度闭环。
 *
 * 控制链路：
 * 遥控器 CH2/CH1/CH3 -> 麦轮转速 -> 速度 PID -> C620 电流指令 -> CAN1。
 ******************************************************************************
 */

#include "chassis_task.h"

#include "FreeRTOS.h"
#include "DJI_Motor.h"
#include "Remote_Control.h"
#include "bsp_dwt.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include "pid.h"
#include "rc_system.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

/* Control timing and safety ------------------------------------------------ */
#define CHASSIS_TASK_PERIOD_S               (0.001F)
#define CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS   (50U)
#define CHASSIS_MAX_LINEAR_SPEED_MPS        (1.0F)
#define CHASSIS_MAX_ROTATION_SPEED_RAD_S    (2.0F)
#define CHASSIS_TASK_NOTIFY_ENABLED         (1UL << 0U)
#define CHASSIS_TASK_NOTIFY_DISABLED        (1UL << 1U)
#define CHASSIS_TASK_NOTIFY_MASK            \
    (CHASSIS_TASK_NOTIFY_ENABLED | CHASSIS_TASK_NOTIFY_DISABLED)

ChassisTask_t g_chassis = {
    .wheel_motor = {
        [CHASSIS_WHEEL_RIGHT_FRONT] = {
            .Motor_Type = DJI_M3508,
            .ID_Set = {
                .RxIdentifier = Chassis_3508_Motor1_RxID,
            },
        },
        [CHASSIS_WHEEL_LEFT_FRONT] = {
            .Motor_Type = DJI_M3508,
            .ID_Set = {
                .RxIdentifier = Chassis_3508_Motor2_RxID,
            },
        },
        [CHASSIS_WHEEL_LEFT_BACK] = {
            .Motor_Type = DJI_M3508,
            .ID_Set = {
                .RxIdentifier = Chassis_3508_Motor3_RxID,
            },
        },
        [CHASSIS_WHEEL_RIGHT_BACK] = {
            .Motor_Type = DJI_M3508,
            .ID_Set = {
                .RxIdentifier = Chassis_3508_Motor4_RxID,
            },
        },
    },
    .wheel_speed_pid = {
        [CHASSIS_WHEEL_RIGHT_FRONT] = {
            .kp = 10.0F,
            .ki = 0.0F,
            .kd = 0.0F,
            .mode = PID_POSITIONAL_MODE,
            .improvement_flags = PID_IMPROVEMENT_OUTPUT_LIMIT,
            .improvement = {
                .output_min = -(float)DJI_C620_CURRENT_COMMAND_MAX,
                .output_max = (float)DJI_C620_CURRENT_COMMAND_MAX,
            },
            .initialized = true,
            .last_status = PID_STATUS_OK,
        },
        [CHASSIS_WHEEL_LEFT_FRONT] = {
            .kp = 10.0F,
            .ki = 0.0F,
            .kd = 0.0F,
            .mode = PID_POSITIONAL_MODE,
            .improvement_flags = PID_IMPROVEMENT_OUTPUT_LIMIT,
            .improvement = {
                .output_min = -(float)DJI_C620_CURRENT_COMMAND_MAX,
                .output_max = (float)DJI_C620_CURRENT_COMMAND_MAX,
            },
            .initialized = true,
            .last_status = PID_STATUS_OK,
        },
        [CHASSIS_WHEEL_LEFT_BACK] = {
            .kp = 10.0F,
            .ki = 0.0F,
            .kd = 0.0F,
            .mode = PID_POSITIONAL_MODE,
            .improvement_flags = PID_IMPROVEMENT_OUTPUT_LIMIT,
            .improvement = {
                .output_min = -(float)DJI_C620_CURRENT_COMMAND_MAX,
                .output_max = (float)DJI_C620_CURRENT_COMMAND_MAX,
            },
            .initialized = true,
            .last_status = PID_STATUS_OK,
        },
        [CHASSIS_WHEEL_RIGHT_BACK] = {
            .kp = 10.0F,
            .ki = 0.0F,
            .kd = 0.0F,
            .mode = PID_POSITIONAL_MODE,
            .improvement_flags = PID_IMPROVEMENT_OUTPUT_LIMIT,
            .improvement = {
                .output_min = -(float)DJI_C620_CURRENT_COMMAND_MAX,
                .output_max = (float)DJI_C620_CURRENT_COMMAND_MAX,
            },
            .initialized = true,
            .last_status = PID_STATUS_OK,
        },
    },
};

static float chassis_clamp(float value, float minimum, float maximum)
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

static int16_t chassis_apply_remote_deadzone(int16_t channel)
{
    if ((channel > -(int16_t)RC_CH_DEADZONE) &&
        (channel < (int16_t)RC_CH_DEADZONE))
    {
        return 0;
    }
    return channel;
}

static void chassis_reset_pid(void)
{
    uint8_t wheel;

    for (wheel = 0U; wheel < (uint8_t)CHASSIS_WHEEL_COUNT; ++wheel)
    {
        (void)pid_reset(&g_chassis.wheel_speed_pid[wheel]);
    }
}

static void chassis_set_zero_current(void)
{
    uint8_t wheel;

    for (wheel = 0U; wheel < (uint8_t)CHASSIS_WHEEL_COUNT; ++wheel)
    {
        g_chassis.wheel_motor[wheel].Data.SET_Current = 0;
        g_chassis.wheel_speed_reference_rpm[wheel] = 0.0F;
    }
}

/**
 * @brief 按 C620 大端格式写入一个电机电流槽位。
 * @note slot 0~3 分别对应当前命令帧覆盖的连续四个电机 ID。
 */
static void chassis_pack_current(uint8_t data[8],
                                 uint8_t slot,
                                 int16_t current)
{
    const uint16_t command = (uint16_t)current;
    const uint8_t offset = (uint8_t)(slot * 2U);

    data[offset] = (uint8_t)(command >> 8U);
    data[offset + 1U] = (uint8_t)command;
}

static void chassis_send_current(void)
{
    uint8_t group_201_to_204[8] = {0U};

    /*
     * 0x200: slot0=右前(0x201)，slot1=左前，slot2=左后，slot3=右后。
     * 本函数是四个底盘 DJI 电机分组帧的唯一发送者。
     */
    chassis_pack_current(
        group_201_to_204,
        0U,
        g_chassis.wheel_motor[CHASSIS_WHEEL_RIGHT_FRONT].Data.SET_Current);
    chassis_pack_current(
        group_201_to_204,
        1U,
        g_chassis.wheel_motor[CHASSIS_WHEEL_LEFT_FRONT].Data.SET_Current);
    chassis_pack_current(
        group_201_to_204,
        2U,
        g_chassis.wheel_motor[CHASSIS_WHEEL_LEFT_BACK].Data.SET_Current);
    chassis_pack_current(
        group_201_to_204,
        3U,
        g_chassis.wheel_motor[CHASSIS_WHEEL_RIGHT_BACK].Data.SET_Current);
    (void)canx_send_data(&hfdcan1,
                         Chassis_3508_MotorA_TxID,
                         group_201_to_204,
                         8U);
}

static bool chassis_read_remote_command(float *vx_mm_s,
                                        float *vy_mm_s,
                                        float *vw_deg_s)
{
    /* DBUS 满量程映射到可配置的底盘最大线速度。 */
    const float remote_speed_scale_mm_s_per_count =
        CHASSIS_MAX_LINEAR_SPEED_MPS * 1000.0F /
        (float)RC_CH_VALUE_MAX;
    /* DBUS 满量程映射到可配置的最大旋转角速度，内部单位为 deg/s。 */
    const float remote_spin_scale_deg_s_per_count =
        CHASSIS_MAX_ROTATION_SPEED_RAD_S * 57.2957795F /
        (float)RC_CH_VALUE_MAX;
    int16_t channel_0;
    int16_t channel_1;
    int16_t channel_2;

    if ((vx_mm_s == NULL) || (vy_mm_s == NULL) ||
        (vw_deg_s == NULL) ||
        (RcSystem_IsReady() == 0U))
    {
        return false;
    }

    taskENTER_CRITICAL();
    channel_0 = remote_ctrl.rc.ch[0];
    channel_1 = remote_ctrl.rc.ch[1];
    channel_2 = remote_ctrl.rc.ch[2];
    taskEXIT_CRITICAL();

    channel_0 = chassis_apply_remote_deadzone(channel_0);
    channel_1 = chassis_apply_remote_deadzone(channel_1);
    channel_2 = chassis_apply_remote_deadzone(channel_2);

    *vx_mm_s =
        chassis_clamp((float)channel_1,
                      -(float)RC_CH_VALUE_MAX,
                      (float)RC_CH_VALUE_MAX) *
        remote_speed_scale_mm_s_per_count;
    *vy_mm_s =
        -chassis_clamp((float)channel_0,
                       -(float)RC_CH_VALUE_MAX,
                       (float)RC_CH_VALUE_MAX) *
        remote_speed_scale_mm_s_per_count;
    *vw_deg_s =
        -chassis_clamp((float)channel_2,
                       -(float)RC_CH_VALUE_MAX,
                       (float)RC_CH_VALUE_MAX) *
        remote_spin_scale_deg_s_per_count;
    return true;
}

static bool chassis_read_wheel_feedback(int16_t wheel_speed_rpm[])
{
    bool feedback_ready = true;
    uint32_t feedback_time_ms[CHASSIS_WHEEL_COUNT];
    uint32_t now_ms;
    uint8_t wheel;

    if (wheel_speed_rpm == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    for (wheel = 0U; wheel < (uint8_t)CHASSIS_WHEEL_COUNT; ++wheel)
    {
        wheel_speed_rpm[wheel] =
            g_chassis.wheel_motor[wheel].Data.Velocity;
        feedback_time_ms[wheel] =
            g_chassis.wheel_motor[wheel].last_feedback_time_ms;
        if (!g_chassis.wheel_motor[wheel].feedback_valid)
        {
            feedback_ready = false;
        }
    }
    taskEXIT_CRITICAL();

    now_ms = DWT_GetTimeMs();
    for (wheel = 0U; wheel < (uint8_t)CHASSIS_WHEEL_COUNT; ++wheel)
    {
        if ((uint32_t)(now_ms - feedback_time_ms[wheel]) >
            CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)
        {
            feedback_ready = false;
        }
    }
    return feedback_ready;
}

static void chassis_mecanum_calculate(float vx_mm_s,
                                      float vy_mm_s,
                                      float vw_deg_s)
{
    /*
     * 旋转速度 deg/s 到轮缘线速度 mm/s：
     * ((轴距 365.0 mm + 轮距 375.0 mm) / 2) * (pi / 180)
     * = 370.0 mm * 0.0174532925 rad/deg
     * = 6.457718 mm/deg。
     */
    const float rotate_ratio = 6.457718F;
    /*
     * 轮缘线速度 mm/s 到减速前电机转速 rpm：
     * 60 s/min * M3508 减速比 19 / 轮周长 478.0 mm
     * = 1140 / 478
     * = 2.384937 rpm/(mm/s)。
     */
    const float wheel_rpm_ratio = 2.384937F;
    float max_wheel_rpm = 0.0F;
    float scale;
    uint8_t wheel;

    g_chassis.wheel_speed_reference_rpm[CHASSIS_WHEEL_RIGHT_FRONT] =
        (-vx_mm_s - vy_mm_s - vw_deg_s * rotate_ratio) *
        wheel_rpm_ratio;
    g_chassis.wheel_speed_reference_rpm[CHASSIS_WHEEL_LEFT_FRONT] =
        (vx_mm_s - vy_mm_s - vw_deg_s * rotate_ratio) *
        wheel_rpm_ratio;
    g_chassis.wheel_speed_reference_rpm[CHASSIS_WHEEL_LEFT_BACK] =
        (vx_mm_s + vy_mm_s - vw_deg_s * rotate_ratio) *
        wheel_rpm_ratio;
    g_chassis.wheel_speed_reference_rpm[CHASSIS_WHEEL_RIGHT_BACK] =
        (-vx_mm_s + vy_mm_s - vw_deg_s * rotate_ratio) *
        wheel_rpm_ratio;

    for (wheel = 0U; wheel < (uint8_t)CHASSIS_WHEEL_COUNT; ++wheel)
    {
        const float wheel_rpm =
            fabsf(g_chassis.wheel_speed_reference_rpm[wheel]);
        if (wheel_rpm > max_wheel_rpm)
        {
            max_wheel_rpm = wheel_rpm;
        }
    }

    if (max_wheel_rpm > (float)MAX_3508_RPM)
    {
        scale = (float)MAX_3508_RPM / max_wheel_rpm;
        for (wheel = 0U; wheel < (uint8_t)CHASSIS_WHEEL_COUNT; ++wheel)
        {
            g_chassis.wheel_speed_reference_rpm[wheel] *= scale;
        }
    }
}

static bool chassis_update_speed_control(const int16_t wheel_speed_rpm[])
{
    float current_command;
    uint8_t wheel;

    if (wheel_speed_rpm == NULL)
    {
        return false;
    }

    for (wheel = 0U; wheel < (uint8_t)CHASSIS_WHEEL_COUNT; ++wheel)
    {
        if (pid_compute(
                &g_chassis.wheel_speed_pid[wheel],
                g_chassis.wheel_speed_reference_rpm[wheel],
                (float)wheel_speed_rpm[wheel],
                CHASSIS_TASK_PERIOD_S,
                &current_command) != PID_STATUS_OK)
        {
            return false;
        }
        g_chassis.wheel_motor[wheel].Data.SET_Current =
            (int16_t)current_command;
    }
    return true;
}

static osThreadId_t g_chassis_task_thread = NULL;

static void chassis_task_step(bool enabled)
{
    float vx_mm_s;
    float vy_mm_s;
    float vw_deg_s;
    int16_t wheel_speed_rpm[CHASSIS_WHEEL_COUNT];

    /* 整车锁定、遥控或任一电机反馈失效时发送安全零电流。 */
    if (!enabled ||
        !chassis_read_remote_command(&vx_mm_s, &vy_mm_s, &vw_deg_s) ||
        !chassis_read_wheel_feedback(wheel_speed_rpm))
    {
        chassis_reset_pid();
        chassis_set_zero_current();
        chassis_send_current();
        return;
    }

    chassis_mecanum_calculate(vx_mm_s, vy_mm_s, vw_deg_s);
    if (!chassis_update_speed_control(wheel_speed_rpm))
    {
        chassis_reset_pid();
        chassis_set_zero_current();
    }
    chassis_send_current();
}

void ChassisTask_OnCan1Rx(uint32_t identifier,
                          uint8_t *data,
                          uint32_t data_len)
{
    ChassisWheel_e wheel;

    if ((data == NULL) || (data_len != FDCAN_DLC_BYTES_8))
    {
        return;
    }

    switch (identifier)
    {
        case Chassis_3508_Motor1_RxID:
            wheel = CHASSIS_WHEEL_RIGHT_FRONT;
            break;
        case Chassis_3508_Motor2_RxID:
            wheel = CHASSIS_WHEEL_LEFT_FRONT;
            break;
        case Chassis_3508_Motor3_RxID:
            wheel = CHASSIS_WHEEL_LEFT_BACK;
            break;
        case Chassis_3508_Motor4_RxID:
            wheel = CHASSIS_WHEEL_RIGHT_BACK;
            break;
        default:
            return;
    }

    DJI_Motor_Info_Update(&g_chassis.wheel_motor[wheel], data, data_len);
}

void ChassisTask_Notify(bool enabled)
{
    if (g_chassis_task_thread == NULL)
    {
        return;
    }

    (void)osThreadFlagsSet(
        g_chassis_task_thread,
        enabled ? CHASSIS_TASK_NOTIFY_ENABLED :
                  CHASSIS_TASK_NOTIFY_DISABLED);
}

void ChassisTask_Run(void)
{
    uint32_t notification;
    bool enabled;

    g_chassis_task_thread = osThreadGetId();
    chassis_set_zero_current();
    chassis_send_current();

    for (;;)
    {
        /* 两个输入周期都未收到通知时按失能处理，禁止沿用旧输出许可。 */
        notification = osThreadFlagsWait(CHASSIS_TASK_NOTIFY_MASK,
                                         osFlagsWaitAny,
                                         2U);
        enabled = ((notification & osFlagsError) == 0U) &&
                  ((notification & CHASSIS_TASK_NOTIFY_DISABLED) == 0U) &&
                  ((notification & CHASSIS_TASK_NOTIFY_ENABLED) != 0U);
        chassis_task_step(enabled);
    }
}
