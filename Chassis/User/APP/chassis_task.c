/**
 ******************************************************************************
 * @file    chassis_task.c
 * @brief   六 M3508 速度闭环、四 J4340 角度 PD 与逐台调试。
 *
 * 控制链路：
 * 当前只接调试器逐台目标，麦轮遥控映射与整车解算暂不接入。
 * 调试器目标 -> 单台 M3508 速度 PID / J4340 角度 PD -> 对应 CAN。
 * CAN1=四麦轮，CAN2=四条腿，CAN3=左右履带；不增加机械行程限位。
 ******************************************************************************
 */

#include "chassis_task.h"


#include "FreeRTOS.h"
#include "DJI_Motor.h"
#include "bsp_dwt.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include "pid.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

/* Control timing and safety ------------------------------------------------ */
#define CHASSIS_TASK_PERIOD_S               (0.001F)
#define CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS   (50U)
#define CHASSIS_LEG_MANAGEMENT_PERIOD_MS    (20U)
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
    .track_motor = {
        [CHASSIS_TRACK_LEFT] = {
            .Motor_Type = DJI_M3508,
            .ID_Set = {.RxIdentifier = Chassis_3508_Motor1_RxID},
        },
        [CHASSIS_TRACK_RIGHT] = {
            .Motor_Type = DJI_M3508,
            .ID_Set = {.RxIdentifier = Chassis_3508_Motor2_RxID},
        },
    },
    .track_speed_pid = {
        [CHASSIS_TRACK_LEFT] = {
            .kp = 10.0F,
            .mode = PID_POSITIONAL_MODE,
            .improvement_flags = PID_IMPROVEMENT_OUTPUT_LIMIT,
            .improvement = {
                .output_min = -(float)DJI_C620_CURRENT_COMMAND_MAX,
                .output_max = (float)DJI_C620_CURRENT_COMMAND_MAX,
            },
            .initialized = true,
            .last_status = PID_STATUS_OK,
        },
        [CHASSIS_TRACK_RIGHT] = {
            .kp = 10.0F,
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
    .leg_motor = {
        [CHASSIS_LEG_RIGHT_FRONT] = DM_MOTOR_CREATE(Mit_mode, DM_J4340, 0x01U, 0x11U),
        [CHASSIS_LEG_LEFT_FRONT] = DM_MOTOR_CREATE(Mit_mode, DM_J4340, 0x02U, 0x12U),
        [CHASSIS_LEG_LEFT_BACK] = DM_MOTOR_CREATE(Mit_mode, DM_J4340, 0x03U, 0x13U),
        [CHASSIS_LEG_RIGHT_BACK] = DM_MOTOR_CREATE(Mit_mode, DM_J4340, 0x04U, 0x14U),
    },
    /* BEGIN 首版调试参数：默认不运行；增益只是待实机调整的悬空测试起点。
     * 正式目标入口接入后，连同调试器命令结构一起移除此参数来源。
     */
    .command = {
        .motor = CHASSIS_TEST_MOTOR_NONE,
        .leg_kp = 5.0F,
        .leg_kd = 0.2F,
    },
    /* END 首版调试参数。 */
};

/** @brief 仅保存跨周期确实需要的选择边沿、使能证据和停止确认。 */
static struct
{
    uint32_t previous_motor;
    bool stopping;
    bool enable_sent;
    bool disable_sent;
    bool target_initialized;
    uint32_t enable_sequence;
    uint32_t disable_sequence;
    bool management_started;
    uint32_t management_time_ms;
} g_chassis_test;

/** @brief 任务上下文的一次腿反馈快照；不是额外的跨周期反馈缓存。 */
typedef struct
{
    float position_rad;
    float velocity_rad_s;
    DM_Motor_State_e state;
    uint32_t sequence;
    bool fresh;
} ChassisLegFeedback_t;

/** @brief 退出测试、切换电机或失能时清除六台速度 PID 的运行状态。 */
static void chassis_reset_pid(void)
{
    uint8_t wheel;

    for (wheel = 0U; wheel < (uint8_t)CHASSIS_WHEEL_COUNT; ++wheel)
    {
        (void)pid_reset(&g_chassis.wheel_speed_pid[wheel]);
    }
    for (wheel = 0U; wheel < (uint8_t)CHASSIS_TRACK_COUNT; ++wheel)
    {
        (void)pid_reset(&g_chassis.track_speed_pid[wheel]);
    }
}

/** @brief 清除两路 DJI 组帧的全部电流及目标，防止未选中槽位沿用旧输出。 */
static void chassis_set_zero_current(void)
{
    uint8_t wheel;

    for (wheel = 0U; wheel < (uint8_t)CHASSIS_WHEEL_COUNT; ++wheel)
    {
        g_chassis.wheel_motor[wheel].Data.SET_Current = 0;
        g_chassis.wheel_speed_reference_rpm[wheel] = 0.0F;
    }
    for (wheel = 0U; wheel < (uint8_t)CHASSIS_TRACK_COUNT; ++wheel)
    {
        g_chassis.track_motor[wheel].Data.SET_Current = 0;
        g_chassis.track_speed_reference_rpm[wheel] = 0.0F;
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

/** @brief 唯一的 DJI 组发送入口；两路即使一路失败也都尝试发送。 */
static bool chassis_send_current(void)
{
    uint8_t group_201_to_204[8] = {0U};
    uint8_t track_group[8] = {0U};
    bool wheel_sent;
    bool track_sent;

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
    chassis_pack_current(track_group, 0U,
        g_chassis.track_motor[CHASSIS_TRACK_LEFT].Data.SET_Current);
    chassis_pack_current(track_group, 1U,
        g_chassis.track_motor[CHASSIS_TRACK_RIGHT].Data.SET_Current);
    wheel_sent = canx_send_data(&hfdcan1, Chassis_3508_MotorA_TxID,
                                group_201_to_204, 8U) == 0U;
    track_sent = canx_send_data(&hfdcan3, Chassis_3508_MotorA_TxID,
                                track_group, 8U) == 0U;
    return wheel_sent && track_sent;
}

static osThreadId_t g_chassis_task_thread = NULL;

/** @brief 同一临界区取时并复制腿反馈；离线判定支持毫秒计数回绕。 */
static ChassisLegFeedback_t chassis_read_leg_feedback(uint32_t leg)
{
    ChassisLegFeedback_t feedback;
    const DM_Motor_Info_t *motor = &g_chassis.leg_motor[leg];
    uint32_t now_ms;

    taskENTER_CRITICAL();
    /* CAN ISR 可能在进入临界区前更新下一毫秒的反馈，不能复用周期开始时间。 */
    now_ms = DWT_GetTimeMs();
    feedback.position_rad = motor->Data.pos;
    feedback.velocity_rad_s = motor->Data.vel;
    feedback.state = motor->Data.state;
    feedback.sequence = motor->feedback_sequence;
    feedback.fresh = motor->feedback_valid &&
        ((uint32_t)(now_ms - motor->last_feedback_time_ms) <=
         CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS);
    taskEXIT_CRITICAL();
    return feedback;
}

/**
 * @brief 撤销测试许可并清除 DJI 输出；腿的停机确认留给本任务后续周期。
 * @note 旧腿未确认失能前保留 active_test_motor，禁止切换时重叠输出。
 */
static void chassis_abort_test(ChassisTestStatus_e status)
{
    g_chassis.command.run = 0U;
    g_chassis.test_status = status;
    chassis_reset_pid();
    chassis_set_zero_current();
    if (g_chassis.active_test_motor >= CHASSIS_TEST_LEG_RIGHT_FRONT)
    {
        if (!g_chassis_test.stopping)
        {
            g_chassis_test.stopping = true;
            g_chassis_test.disable_sent = false;
            g_chassis_test.management_started = false;
        }
    }
    else
    {
        g_chassis.active_test_motor = CHASSIS_TEST_MOTOR_NONE;
    }
}

/**
 * @brief J4340 纯力矩输出边界；MIT 的位置、速度、kp、kd 始终为零。
 * @note 不增加产品力矩/机械限位；仅沿用驱动编码范围，失败不覆盖提交记录。
 */
static bool chassis_send_leg_torque(uint32_t leg, float torque_nm)
{
    DM_Motor_MitCommand_t applied;

    if (DM_Motor_Ctrl(&hfdcan2, &g_chassis.leg_motor[leg],
                     0.0F, 0.0F, 0.0F, 0.0F, torque_nm,
                     0U, &applied) != 0U)
    {
        return false;
    }
    g_chassis.leg_submitted_torque_nm[leg] = applied.torq;
    return true;
}

/**
 * @brief 非阻塞管理腿使能/失能，每 20 ms 请求一次，运动帧仍随原 1 ms 周期。
 * @note 未选中腿只发失能；被选中腿先成功发送零力矩，再发送使能。
 *       收到本次请求之后的新反馈才能确认状态，不以发送成功代替电机确认。
 */
static bool chassis_manage_legs(uint32_t now_ms)
{
    uint32_t leg;
    bool sent = true;

    if (g_chassis_test.management_started &&
        ((uint32_t)(now_ms - g_chassis_test.management_time_ms) <
         CHASSIS_LEG_MANAGEMENT_PERIOD_MS))
    {
        return true;
    }
    g_chassis_test.management_started = true;
    g_chassis_test.management_time_ms = now_ms;
    for (leg = 0U; leg < (uint32_t)CHASSIS_LEG_COUNT; ++leg)
    {
        DM_Motor_Info_t *motor = &g_chassis.leg_motor[leg];
        const bool active = g_chassis.active_test_motor ==
            (uint32_t)CHASSIS_TEST_LEG_RIGHT_FRONT + leg;
        uint32_t sequence;

        taskENTER_CRITICAL();
        sequence = motor->feedback_sequence;
        taskEXIT_CRITICAL();
        if (active && !g_chassis_test.stopping)
        {
            if (g_chassis_test.target_initialized)
            {
                continue;
            }
            if (!chassis_send_leg_torque(leg, 0.0F) ||
                (DM_Enable_Motor(&hfdcan2,
                                 (uint16_t)motor->ID_Set.TxIdentifier,
                                 (uint16_t)Mit_mode, 0U) != 0U))
            {
                sent = false;
                continue;
            }
            if (!g_chassis_test.enable_sent)
            {
                g_chassis_test.enable_sequence = sequence;
                g_chassis_test.enable_sent = true;
            }
        }
        else
        {
            if (active && !chassis_send_leg_torque(leg, 0.0F))
            {
                sent = false;
            }
            /* 即使零力矩帧失败也继续尝试失能，不短路后续电机。 */
            if (DM_Disable_Motor(&hfdcan2,
                                  (uint16_t)motor->ID_Set.TxIdentifier,
                                  (uint16_t)Mit_mode, 0U) != 0U)
            {
                sent = false;
                continue;
            }
            if (active && !g_chassis_test.disable_sent)
            {
                g_chassis_test.disable_sequence = sequence;
                g_chassis_test.disable_sent = true;
            }
        }
    }
    return sent;
}

/* BEGIN 首版逐台调试控制：仅替代目标来源，不替代真实电机驱动。
 * 范围为 command 中选择的一台电机；正式遥控目标接入后移除此调试入口。
 */
/** @brief 只读取并控制所选电机，不要求其他九台电机反馈在线。 */
static void chassis_run_motor_test(const ChassisTestCommand_t *command)
{
    if ((command->motor == CHASSIS_TEST_MOTOR_NONE) ||
        (command->motor >= (uint32_t)CHASSIS_TEST_MOTOR_COUNT) ||
        (command->run != 1U))
    {
        chassis_abort_test(CHASSIS_TEST_INVALID_COMMAND);
        return;
    }

    if (command->motor < CHASSIS_TEST_LEG_RIGHT_FRONT)
    {
        DJI_Motor_Info_t *motor;
        PidController_t *pid;
        float *reference;
        float current;
        int16_t speed_rpm;
        bool fresh;
        uint32_t index;
        uint32_t now_ms;

        if (command->motor <= CHASSIS_TEST_WHEEL_RIGHT_BACK)
        {
            index = command->motor - (uint32_t)CHASSIS_TEST_WHEEL_RIGHT_FRONT;
            motor = &g_chassis.wheel_motor[index];
            pid = &g_chassis.wheel_speed_pid[index];
            reference = &g_chassis.wheel_speed_reference_rpm[index];
        }
        else
        {
            index = command->motor - (uint32_t)CHASSIS_TEST_TRACK_LEFT;
            motor = &g_chassis.track_motor[index];
            pid = &g_chassis.track_speed_pid[index];
            reference = &g_chassis.track_speed_reference_rpm[index];
        }
        if (!isfinite(command->speed_rpm))
        {
            chassis_abort_test(CHASSIS_TEST_INVALID_COMMAND);
            return;
        }
        taskENTER_CRITICAL();
        /* 取时与速度/时间戳快照受同一中断屏蔽保护，避免无符号负时间差。 */
        now_ms = DWT_GetTimeMs();
        speed_rpm = motor->Data.Velocity;
        fresh = motor->feedback_valid &&
            ((uint32_t)(now_ms - motor->last_feedback_time_ms) <=
             CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS);
        taskEXIT_CRITICAL();
        if (!fresh)
        {
            if (g_chassis.active_test_motor == CHASSIS_TEST_MOTOR_NONE)
            {
                g_chassis.test_status = CHASSIS_TEST_WAIT_FEEDBACK;
            }
            else
            {
                chassis_abort_test(CHASSIS_TEST_FEEDBACK_LOST);
            }
            return;
        }
        g_chassis.active_test_motor = command->motor;
        if (pid_compute(pid, command->speed_rpm, (float)speed_rpm,
                        CHASSIS_TASK_PERIOD_S, &current) != PID_STATUS_OK)
        {
            chassis_abort_test(CHASSIS_TEST_INVALID_COMMAND);
            return;
        }
        *reference = command->speed_rpm;
        motor->Data.SET_Current = (int16_t)current;
        g_chassis.test_status = CHASSIS_TEST_RUNNING;
    }
    else
    {
        const uint32_t leg = command->motor - (uint32_t)CHASSIS_TEST_LEG_RIGHT_FRONT;
        const ChassisLegFeedback_t feedback = chassis_read_leg_feedback(leg);
        float target_deg = command->position_deg;
        float torque_nm;

        if (!isfinite(command->leg_kp) || !isfinite(command->leg_kd) ||
            (command->leg_kp < 0.0F) || (command->leg_kd < 0.0F))
        {
            chassis_abort_test(CHASSIS_TEST_INVALID_COMMAND);
            return;
        }
        if (!feedback.fresh)
        {
            if (g_chassis.active_test_motor == CHASSIS_TEST_MOTOR_NONE)
            {
                g_chassis.test_status = CHASSIS_TEST_WAIT_FEEDBACK;
            }
            else
            {
                chassis_abort_test(CHASSIS_TEST_FEEDBACK_LOST);
            }
            return;
        }
        if (((feedback.state != DM_MOTOR_STATE_DISABLED) &&
             (feedback.state != DM_MOTOR_STATE_ENABLED)) ||
            !isfinite(feedback.position_rad) || !isfinite(feedback.velocity_rad_s))
        {
            chassis_abort_test(CHASSIS_TEST_MOTOR_FAULT);
            return;
        }
        if (g_chassis.active_test_motor == CHASSIS_TEST_MOTOR_NONE)
        {
            g_chassis.active_test_motor = command->motor;
            g_chassis_test.enable_sent = false;
            g_chassis_test.target_initialized = false;
            g_chassis_test.management_started = false;
        }
        if (g_chassis_test.target_initialized &&
            (feedback.state != DM_MOTOR_STATE_ENABLED))
        {
            chassis_abort_test(CHASSIS_TEST_MOTOR_FAULT);
            return;
        }
        if (!g_chassis_test.enable_sent ||
            (feedback.sequence == g_chassis_test.enable_sequence) ||
            (feedback.state != DM_MOTOR_STATE_ENABLED))
        {
            g_chassis.test_status = CHASSIS_TEST_ENABLING;
            return;
        }
        if (!g_chassis_test.target_initialized)
        {
            /* 使能新反馈到达后才初始化；旧的绝对目标不会跨越使能沿回放。 */
            target_deg = feedback.position_rad * 57.2957795F;
            g_chassis.command.position_deg = target_deg;
            g_chassis_test.target_initialized = true;
        }
        /* 目标 deg -> rad；直接位置误差，不做角度回绕、行程限位或测试窗口。
         * tau[N.m] = kp[N.m/rad] * error[rad] - kd[N.m.s/rad] * vel[rad/s]。
         */
        torque_nm = command->leg_kp *
                    (target_deg * 0.0174532925F - feedback.position_rad) -
                    command->leg_kd * feedback.velocity_rad_s;
        if (!isfinite(target_deg) || !isfinite(torque_nm))
        {
            chassis_abort_test(CHASSIS_TEST_INVALID_COMMAND);
            return;
        }
        if (!chassis_send_leg_torque(leg, torque_nm))
        {
            chassis_abort_test(CHASSIS_TEST_SEND_FAILED);
            return;
        }
        g_chassis.test_status = CHASSIS_TEST_RUNNING;
    }
}
/* END 首版逐台调试控制。 */

/**
 * @brief 一个控制周期：命令快照 -> 门控/停止确认 -> 所选闭环 -> 总线发送。
 * @note 新增测试不改变原遥控许可来源，不增加软件机械限位或姿态联动。
 */
static void chassis_task_step(bool enabled)
{
    const ChassisTestCommand_t command = g_chassis.command;
    const uint32_t now_ms = DWT_GetTimeMs();
    bool command_changed;

    chassis_set_zero_current();
    command_changed = command.motor != g_chassis_test.previous_motor;
    g_chassis_test.previous_motor = command.motor;
    if (command_changed)
    {
        chassis_abort_test(CHASSIS_TEST_IDLE);
        g_chassis.command.speed_rpm = 0.0F;
    }
    if (!enabled)
    {
        chassis_abort_test(CHASSIS_TEST_REMOTE_LOCKED);
    }
    else if (command.run == 0U)
    {
        ChassisTestStatus_e status = g_chassis.test_status;
        /* 错误保留供调试器读取；新的 run 请求或电机选择会重新评估。 */
        if ((status == CHASSIS_TEST_RUNNING) || (status == CHASSIS_TEST_ENABLING) ||
            (status == CHASSIS_TEST_WAIT_FEEDBACK) ||
            (status == CHASSIS_TEST_REMOTE_LOCKED))
        {
            status = CHASSIS_TEST_IDLE;
        }
        chassis_abort_test(status);
    }

    if (g_chassis_test.stopping)
    {
        const uint32_t leg = g_chassis.active_test_motor -
                             (uint32_t)CHASSIS_TEST_LEG_RIGHT_FRONT;
        const ChassisLegFeedback_t feedback = chassis_read_leg_feedback(leg);

        if (g_chassis_test.disable_sent && feedback.fresh &&
            (feedback.sequence != g_chassis_test.disable_sequence) &&
            (feedback.state == DM_MOTOR_STATE_DISABLED))
        {
            g_chassis.active_test_motor = CHASSIS_TEST_MOTOR_NONE;
            g_chassis_test.stopping = false;
            g_chassis_test.target_initialized = false;
            if (g_chassis.test_status == CHASSIS_TEST_STOPPING)
            {
                g_chassis.test_status = CHASSIS_TEST_IDLE;
            }
        }
        else if ((g_chassis.test_status == CHASSIS_TEST_IDLE) ||
                 (g_chassis.test_status == CHASSIS_TEST_RUNNING))
        {
            g_chassis.test_status = CHASSIS_TEST_STOPPING;
        }
    }

    if (enabled && !command_changed && !g_chassis_test.stopping &&
        (command.run != 0U))
    {
        chassis_run_motor_test(&command);
    }

    if (!chassis_manage_legs(now_ms))
    {
        chassis_abort_test(CHASSIS_TEST_SEND_FAILED);
        /* 发生故障后立即尝试停止旧腿，重试仍由后续周期负责。 */
        (void)chassis_manage_legs(now_ms);
    }
    if (!chassis_send_current())
    {
        chassis_abort_test(CHASSIS_TEST_SEND_FAILED);
        (void)chassis_manage_legs(now_ms);
        (void)chassis_send_current();
    }
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

void ChassisTask_OnCan2Rx(uint32_t identifier,
                          const uint8_t *data,
                          uint32_t data_len)
{
    uint32_t leg;

    if ((data == NULL) || (data_len != FDCAN_DLC_BYTES_8))
    {
        return;
    }
    for (leg = 0U; leg < (uint32_t)CHASSIS_LEG_COUNT; ++leg)
    {
        DM_Motor_Info_t *motor = &g_chassis.leg_motor[leg];
        if ((identifier == motor->ID_Set.RxIdentifier) &&
            ((uint32_t)(data[0] & 0x0FU) == motor->ID_Set.TxIdentifier))
        {
            DM_Motor_Info_Update(motor, data, data_len);
            return;
        }
    }
}

void ChassisTask_OnCan3Rx(uint32_t identifier,
                          uint8_t *data,
                          uint32_t data_len)
{
    uint32_t track;

    if ((data == NULL) || (data_len != FDCAN_DLC_BYTES_8))
    {
        return;
    }
    for (track = 0U; track < (uint32_t)CHASSIS_TRACK_COUNT; ++track)
    {
        if (identifier == g_chassis.track_motor[track].ID_Set.RxIdentifier)
        {
            DJI_Motor_Info_Update(&g_chassis.track_motor[track], data, data_len);
            return;
        }
    }
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
    (void)chassis_send_current();

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

