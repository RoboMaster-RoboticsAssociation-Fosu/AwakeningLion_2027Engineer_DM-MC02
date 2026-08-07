/**
 * @file pid.c
 * @brief 通用 PID 控制器实现。
 */

#include "pid.h"

#include <math.h>
#include <string.h>

/**
 * @brief 记录状态并将其返回，统一所有错误出口。
 */
static PidStatus pid_record_status(PidController *pid, PidStatus status)
{
    if (pid != NULL)
    {
        pid->last_status = status;
    }
    return status;
}

/**
 * @brief 判断浮点数是否可参与控制计算。
 */
static bool pid_is_finite(float value)
{
    return isfinite(value) != 0;
}

/**
 * @brief 判断控制器是否启用了指定优化功能。
 */
static bool pid_has_improvement(const PidController *pid, uint16_t improvement)
{
    return (pid->improvement_flags & improvement) != 0U;
}

/**
 * @brief 检查控制器实例是否可供运行期接口使用。
 */
static PidStatus pid_require_initialized(PidController *pid)
{
    if (pid == NULL)
    {
        return PID_STATUS_NULL_POINTER;
    }
    if (!pid->initialized)
    {
        return pid_record_status(pid, PID_STATUS_NOT_INITIALIZED);
    }
    return PID_STATUS_OK;
}

/**
 * @brief 将数值限制到给定闭区间。
 */
static float pid_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

/**
 * @brief 校验当前已启用优化功能对应的参数。
 *
 * 控制器结构公开用于在线观测，调用方仍可能误写参数，因此计算前也会执行
 * 本校验，避免无效配置继续传播到执行器。
 */
static PidStatus pid_validate_improvement_parameters(const PidController *pid,
                                                     uint16_t improvement_flags)
{
    const PidImprovementParams *params;

    if ((improvement_flags & (uint16_t)(~PID_IMPROVEMENT_ALL_MASK)) != 0U)
    {
        return PID_STATUS_INVALID_PARAMETER;
    }

    params = &pid->improvement;

    if ((improvement_flags & PID_IMPROVEMENT_OUTPUT_LIMIT) != 0U)
    {
        if (!pid_is_finite(params->output_min) ||
            !pid_is_finite(params->output_max) ||
            (params->output_min >= params->output_max))
        {
            return PID_STATUS_INVALID_PARAMETER;
        }
    }

    if ((improvement_flags & PID_IMPROVEMENT_INTEGRAL_LIMIT) != 0U)
    {
        if (!pid_is_finite(params->integral_limit) ||
            (params->integral_limit < 0.0F))
        {
            return PID_STATUS_INVALID_PARAMETER;
        }
    }

    if ((improvement_flags & PID_IMPROVEMENT_DEAD_BAND) != 0U)
    {
        if (!pid_is_finite(params->dead_band) || (params->dead_band < 0.0F))
        {
            return PID_STATUS_INVALID_PARAMETER;
        }
    }

    if ((improvement_flags & PID_IMPROVEMENT_DERIVATIVE_FILTER) != 0U)
    {
        if (!pid_is_finite(params->derivative_filter_time_constant_s) ||
            (params->derivative_filter_time_constant_s <= 0.0F))
        {
            return PID_STATUS_INVALID_PARAMETER;
        }
    }

    if ((improvement_flags & PID_IMPROVEMENT_VARIABLE_INTEGRAL) != 0U)
    {
        if (!pid_is_finite(params->variable_integral_threshold_a) ||
            !pid_is_finite(params->variable_integral_threshold_b) ||
            (params->variable_integral_threshold_a <= 0.0F) ||
            (params->variable_integral_threshold_b < 0.0F))
        {
            return PID_STATUS_INVALID_PARAMETER;
        }
    }

    return PID_STATUS_OK;
}

/**
 * @brief 根据误差计算变速积分系数，返回范围为 [0, 1]。
 */
static float pid_variable_integral_factor(const PidController *pid, float error)
{
    const float absolute_error = fabsf(error);
    const float threshold_a = pid->improvement.variable_integral_threshold_a;
    const float threshold_b = pid->improvement.variable_integral_threshold_b;

    if (!pid_has_improvement(pid, PID_IMPROVEMENT_VARIABLE_INTEGRAL))
    {
        return 1.0F;
    }
    if (absolute_error <= threshold_b)
    {
        return 1.0F;
    }
    if (absolute_error >= (threshold_a + threshold_b))
    {
        return 0.0F;
    }
    return (threshold_a + threshold_b - absolute_error) / threshold_a;
}

/**
 * @brief 对微分项应用与采样周期自适应的一阶低通滤波。
 *
 * alpha = dt / (tau + dt)。相比原实现固定滤波系数，该形式在任务周期轻微
 * 波动时仍保持相同的物理截止特性。
 */
static float pid_filter_derivative(const PidController *pid, float derivative)
{
    float alpha;

    if (!pid_has_improvement(pid, PID_IMPROVEMENT_DERIVATIVE_FILTER))
    {
        return derivative;
    }

    alpha = pid->period_s /
            (pid->improvement.derivative_filter_time_constant_s + pid->period_s);
    return pid->previous_d_output + alpha * (derivative - pid->previous_d_output);
}

/**
 * @brief 计算位置式 PID。
 */
static float pid_compute_positional(PidController *pid,
                                    float control_error,
                                    bool in_dead_band)
{
    const bool external_p = pid_has_improvement(pid, PID_IMPROVEMENT_EXTERNAL_P);
    const bool external_i = pid_has_improvement(pid, PID_IMPROVEMENT_EXTERNAL_I);
    const bool external_d = pid_has_improvement(pid, PID_IMPROVEMENT_EXTERNAL_D);
    const float previous_i_output = pid->i_output;
    float raw_output;
    float derivative;

    /* 死区只抑制内部 P/D，不清空 I，从而保留重力补偿等稳态偏置。 */
    if (!external_p)
    {
        pid->p_output = in_dead_band ? 0.0F : (pid->kp * control_error);
    }

    pid->integral_increment = 0.0F;
    if (!external_i && !in_dead_band)
    {
        pid->integral_increment = pid->ki * control_error *
                                  pid_variable_integral_factor(pid, control_error) *
                                  pid->period_s;
        pid->i_output += pid->integral_increment;

        if (pid_has_improvement(pid, PID_IMPROVEMENT_INTEGRAL_LIMIT))
        {
            pid->i_output = pid_clamp(pid->i_output,
                                      -pid->improvement.integral_limit,
                                      pid->improvement.integral_limit);
            pid->integral_increment = pid->i_output - previous_i_output;
        }
    }

    if (!external_d)
    {
        derivative = 0.0F;
        if (!in_dead_band && (pid->sample_count >= 1U))
        {
            if (pid_has_improvement(pid,
                                    PID_IMPROVEMENT_DERIVATIVE_ON_MEASUREMENT))
            {
                derivative = -pid->kd *
                             (pid->measurement - pid->previous_measurement) /
                             pid->period_s;
            }
            else
            {
                derivative = pid->kd *
                             (control_error - pid->previous_error) /
                             pid->period_s;
            }
        }
        pid->d_output = in_dead_band
                            ? 0.0F
                            : pid_filter_derivative(pid, derivative);
    }

    raw_output = pid->p_output + pid->i_output + pid->d_output;

    if (pid_has_improvement(pid, PID_IMPROVEMENT_OUTPUT_LIMIT))
    {
        /*
         * 条件积分抗饱和：仅当本周期内部积分继续把输出推向饱和方向时，
         * 撤销本周期积分；外部接管的积分不在模块内擅自修改。
         */
        if (!external_i &&
            (((raw_output > pid->improvement.output_max) &&
              (pid->integral_increment > 0.0F)) ||
             ((raw_output < pid->improvement.output_min) &&
              (pid->integral_increment < 0.0F))))
        {
            pid->i_output = previous_i_output;
            pid->integral_increment = 0.0F;
            raw_output = pid->p_output + pid->i_output + pid->d_output;
        }

        raw_output = pid_clamp(raw_output,
                               pid->improvement.output_min,
                               pid->improvement.output_max);
    }

    return raw_output;
}

/**
 * @brief 计算增量式 PID。
 *
 * 增量式控制器的 P/I/D 分量均表示“本周期输出增量”。积分限幅因此限制的是
 * 单周期积分增量；累计输出由输出限幅负责约束。
 */
static float pid_compute_incremental(PidController *pid,
                                     float control_error,
                                     bool in_dead_band)
{
    const bool external_p = pid_has_improvement(pid, PID_IMPROVEMENT_EXTERNAL_P);
    const bool external_i = pid_has_improvement(pid, PID_IMPROVEMENT_EXTERNAL_I);
    const bool external_d = pid_has_improvement(pid, PID_IMPROVEMENT_EXTERNAL_D);
    float raw_output;
    float derivative;

    if (!external_p)
    {
        pid->p_output = in_dead_band
                            ? 0.0F
                            : pid->kp * (control_error - pid->previous_error);
    }

    if (!external_i)
    {
        pid->i_output = in_dead_band
                            ? 0.0F
                            : pid->ki * control_error *
                                  pid_variable_integral_factor(pid, control_error) *
                                  pid->period_s;
        if (pid_has_improvement(pid, PID_IMPROVEMENT_INTEGRAL_LIMIT))
        {
            pid->i_output = pid_clamp(pid->i_output,
                                      -pid->improvement.integral_limit,
                                      pid->improvement.integral_limit);
        }
    }
    pid->integral_increment = external_i ? 0.0F : pid->i_output;

    if (!external_d)
    {
        derivative = 0.0F;
        if (!in_dead_band && (pid->sample_count >= 2U))
        {
            if (pid_has_improvement(pid,
                                    PID_IMPROVEMENT_DERIVATIVE_ON_MEASUREMENT))
            {
                derivative = -pid->kd *
                             (pid->measurement -
                              (2.0F * pid->previous_measurement) +
                              pid->previous_measurement_2) /
                             pid->period_s;
            }
            else
            {
                derivative = pid->kd *
                             (control_error -
                              (2.0F * pid->previous_error) +
                              pid->previous_error_2) /
                             pid->period_s;
            }
        }
        pid->d_output = in_dead_band
                            ? 0.0F
                            : pid_filter_derivative(pid, derivative);
    }

    raw_output = pid->previous_output +
                 pid->p_output + pid->i_output + pid->d_output;

    if (pid_has_improvement(pid, PID_IMPROVEMENT_OUTPUT_LIMIT))
    {
        /* 增量积分继续推向饱和时，丢弃该增量后再执行最终限幅。 */
        if (!external_i &&
            (((raw_output > pid->improvement.output_max) &&
              (pid->i_output > 0.0F)) ||
             ((raw_output < pid->improvement.output_min) &&
              (pid->i_output < 0.0F))))
        {
            pid->i_output = 0.0F;
            pid->integral_increment = 0.0F;
            raw_output = pid->previous_output +
                         pid->p_output + pid->d_output;
        }

        raw_output = pid_clamp(raw_output,
                               pid->improvement.output_min,
                               pid->improvement.output_max);
    }

    return raw_output;
}

/**
 * @brief 数值异常时清空受污染的运行状态并输出安全零值。
 */
static PidStatus pid_handle_numeric_error(PidController *pid, float *output)
{
    (void)pid_reset(pid);
    pid->last_status = PID_STATUS_NUMERIC_ERROR;
    *output = 0.0F;
    return PID_STATUS_NUMERIC_ERROR;
}

PidStatus pid_init(PidController *pid, PidMode mode, float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return PID_STATUS_NULL_POINTER;
    }
    if ((mode != PID_POSITIONAL_MODE) && (mode != PID_INCREMENTAL_MODE))
    {
        return pid_record_status(pid, PID_STATUS_INVALID_MODE);
    }
    if (!pid_is_finite(kp) || !pid_is_finite(ki) || !pid_is_finite(kd))
    {
        return pid_record_status(pid, PID_STATUS_INVALID_PARAMETER);
    }

    (void)memset(pid, 0, sizeof(*pid));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->mode = mode;
    pid->initialized = true;
    pid->last_status = PID_STATUS_OK;
    return PID_STATUS_OK;
}

PidStatus pid_set_params(PidController *pid, float kp, float ki, float kd)
{
    PidStatus status = pid_require_initialized(pid);

    if (status != PID_STATUS_OK)
    {
        return status;
    }
    if (!pid_is_finite(kp) || !pid_is_finite(ki) || !pid_is_finite(kd))
    {
        return pid_record_status(pid, PID_STATUS_INVALID_PARAMETER);
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    return pid_record_status(pid, PID_STATUS_OK);
}

PidStatus pid_set_output_limit(PidController *pid, float min_limit, float max_limit)
{
    PidStatus status = pid_require_initialized(pid);

    if (status != PID_STATUS_OK)
    {
        return status;
    }
    if (!pid_is_finite(min_limit) || !pid_is_finite(max_limit) ||
        (min_limit >= max_limit))
    {
        return pid_record_status(pid, PID_STATUS_INVALID_PARAMETER);
    }

    pid->improvement.output_min = min_limit;
    pid->improvement.output_max = max_limit;
    pid->improvement_flags |= PID_IMPROVEMENT_OUTPUT_LIMIT;
    return pid_record_status(pid, PID_STATUS_OK);
}

PidStatus pid_set_integral_limit(PidController *pid, float limit)
{
    PidStatus status = pid_require_initialized(pid);

    if (status != PID_STATUS_OK)
    {
        return status;
    }
    if (!pid_is_finite(limit) || (limit < 0.0F))
    {
        return pid_record_status(pid, PID_STATUS_INVALID_PARAMETER);
    }

    pid->improvement.integral_limit = limit;
    pid->improvement_flags |= PID_IMPROVEMENT_INTEGRAL_LIMIT;
    return pid_record_status(pid, PID_STATUS_OK);
}

PidStatus pid_set_dead_band(PidController *pid, float dead_band)
{
    PidStatus status = pid_require_initialized(pid);

    if (status != PID_STATUS_OK)
    {
        return status;
    }
    if (!pid_is_finite(dead_band) || (dead_band < 0.0F))
    {
        return pid_record_status(pid, PID_STATUS_INVALID_PARAMETER);
    }

    pid->improvement.dead_band = dead_band;
    pid->improvement_flags |= PID_IMPROVEMENT_DEAD_BAND;
    return pid_record_status(pid, PID_STATUS_OK);
}

PidStatus pid_set_derivative_filter_time_constant(PidController *pid,
                                                  float time_constant_s)
{
    PidStatus status = pid_require_initialized(pid);

    if (status != PID_STATUS_OK)
    {
        return status;
    }
    if (!pid_is_finite(time_constant_s) || (time_constant_s <= 0.0F))
    {
        return pid_record_status(pid, PID_STATUS_INVALID_PARAMETER);
    }

    pid->improvement.derivative_filter_time_constant_s = time_constant_s;
    pid->improvement_flags |= PID_IMPROVEMENT_DERIVATIVE_FILTER;
    return pid_record_status(pid, PID_STATUS_OK);
}

PidStatus pid_set_derivative_first_enable(PidController *pid, bool enable)
{
    return pid_set_improvement_enabled(
        pid, PID_IMPROVEMENT_DERIVATIVE_ON_MEASUREMENT, enable);
}

PidStatus pid_set_variable_integral_thresholds(PidController *pid,
                                               float threshold_a,
                                               float threshold_b)
{
    PidStatus status = pid_require_initialized(pid);

    if (status != PID_STATUS_OK)
    {
        return status;
    }
    if (!pid_is_finite(threshold_a) || !pid_is_finite(threshold_b) ||
        (threshold_a <= 0.0F) || (threshold_b < 0.0F))
    {
        return pid_record_status(pid, PID_STATUS_INVALID_PARAMETER);
    }

    pid->improvement.variable_integral_threshold_a = threshold_a;
    pid->improvement.variable_integral_threshold_b = threshold_b;
    pid->improvement_flags |= PID_IMPROVEMENT_VARIABLE_INTEGRAL;
    return pid_record_status(pid, PID_STATUS_OK);
}

PidStatus pid_set_improvement_enabled(PidController *pid,
                                      uint16_t improvement_mask,
                                      bool enable)
{
    PidStatus status = pid_require_initialized(pid);
    uint16_t candidate_flags;

    if (status != PID_STATUS_OK)
    {
        return status;
    }
    if ((improvement_mask & (uint16_t)(~PID_IMPROVEMENT_ALL_MASK)) != 0U)
    {
        return pid_record_status(pid, PID_STATUS_INVALID_PARAMETER);
    }

    candidate_flags = enable
                          ? (uint16_t)(pid->improvement_flags | improvement_mask)
                          : (uint16_t)(pid->improvement_flags &
                                       (uint16_t)(~improvement_mask));

    status = pid_validate_improvement_parameters(pid, candidate_flags);
    if (status != PID_STATUS_OK)
    {
        return pid_record_status(pid, status);
    }

    pid->improvement_flags = candidate_flags;
    return pid_record_status(pid, PID_STATUS_OK);
}

PidStatus pid_set_external_outputs(PidController *pid,
                                   float p_output,
                                   float i_output,
                                   float d_output)
{
    PidStatus status = pid_require_initialized(pid);
    const bool external_p = (pid != NULL) &&
                            pid_has_improvement(pid, PID_IMPROVEMENT_EXTERNAL_P);
    const bool external_i = (pid != NULL) &&
                            pid_has_improvement(pid, PID_IMPROVEMENT_EXTERNAL_I);
    const bool external_d = (pid != NULL) &&
                            pid_has_improvement(pid, PID_IMPROVEMENT_EXTERNAL_D);

    if (status != PID_STATUS_OK)
    {
        return status;
    }
    if ((!external_p && !external_i && !external_d) ||
        (external_p && !pid_is_finite(p_output)) ||
        (external_i && !pid_is_finite(i_output)) ||
        (external_d && !pid_is_finite(d_output)))
    {
        return pid_record_status(pid, PID_STATUS_INVALID_PARAMETER);
    }

    if (external_p)
    {
        pid->p_output = p_output;
    }
    if (external_i)
    {
        pid->i_output = i_output;
    }
    if (external_d)
    {
        pid->d_output = d_output;
    }
    return pid_record_status(pid, PID_STATUS_OK);
}

PidStatus pid_reset(PidController *pid)
{
    PidStatus status = pid_require_initialized(pid);
    float kp;
    float ki;
    float kd;
    PidMode mode;
    uint16_t improvement_flags;
    PidImprovementParams improvement;

    if (status != PID_STATUS_OK)
    {
        return status;
    }

    kp = pid->kp;
    ki = pid->ki;
    kd = pid->kd;
    mode = pid->mode;
    improvement_flags = pid->improvement_flags;
    improvement = pid->improvement;

    (void)memset(pid, 0, sizeof(*pid));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->mode = mode;
    pid->improvement_flags = improvement_flags;
    pid->improvement = improvement;
    pid->initialized = true;
    pid->last_status = PID_STATUS_OK;
    return PID_STATUS_OK;
}

PidStatus pid_compute(PidController *pid,
                      float setpoint,
                      float measurement,
                      float period_s,
                      float *output)
{
    PidStatus status;
    float control_error;
    bool in_dead_band;

    if (output == NULL)
    {
        return pid_record_status(pid, PID_STATUS_NULL_POINTER);
    }
    *output = 0.0F;

    status = pid_require_initialized(pid);
    if (status != PID_STATUS_OK)
    {
        return status;
    }
    if (!pid_is_finite(setpoint) || !pid_is_finite(measurement))
    {
        return pid_record_status(pid, PID_STATUS_INVALID_PARAMETER);
    }
    if (!pid_is_finite(period_s) || (period_s <= 0.0F))
    {
        return pid_record_status(pid, PID_STATUS_INVALID_PERIOD);
    }
    if ((pid->mode != PID_POSITIONAL_MODE) &&
        (pid->mode != PID_INCREMENTAL_MODE))
    {
        return pid_record_status(pid, PID_STATUS_INVALID_MODE);
    }

    status = pid_validate_improvement_parameters(pid, pid->improvement_flags);
    if (status != PID_STATUS_OK)
    {
        return pid_record_status(pid, status);
    }

    /* 外部接管项属于本次计算输入，使用前必须同样通过数值检查。 */
    if ((pid_has_improvement(pid, PID_IMPROVEMENT_EXTERNAL_P) &&
         !pid_is_finite(pid->p_output)) ||
        (pid_has_improvement(pid, PID_IMPROVEMENT_EXTERNAL_I) &&
         !pid_is_finite(pid->i_output)) ||
        (pid_has_improvement(pid, PID_IMPROVEMENT_EXTERNAL_D) &&
         !pid_is_finite(pid->d_output)))
    {
        return pid_record_status(pid, PID_STATUS_INVALID_PARAMETER);
    }

    control_error = setpoint - measurement;
    if (!pid_is_finite(control_error))
    {
        return pid_handle_numeric_error(pid, output);
    }

    in_dead_band = pid_has_improvement(pid, PID_IMPROVEMENT_DEAD_BAND) &&
                   (fabsf(control_error) <= pid->improvement.dead_band);
    if (in_dead_band)
    {
        control_error = 0.0F;
    }

    pid->setpoint = setpoint;
    pid->measurement = measurement;
    pid->error = control_error;
    pid->period_s = period_s;

    if (pid->mode == PID_POSITIONAL_MODE)
    {
        pid->output = pid_compute_positional(pid, control_error, in_dead_band);
    }
    else
    {
        pid->output = pid_compute_incremental(pid, control_error, in_dead_band);
    }

    if (!pid_is_finite(pid->p_output) ||
        !pid_is_finite(pid->i_output) ||
        !pid_is_finite(pid->d_output) ||
        !pid_is_finite(pid->output))
    {
        return pid_handle_numeric_error(pid, output);
    }

    /* 仅在完整成功后提交历史状态，失败计算不会污染下一周期差分。 */
    pid->previous_error_2 = pid->previous_error;
    pid->previous_error = control_error;
    pid->previous_measurement_2 = pid->previous_measurement;
    pid->previous_measurement = measurement;
    pid->previous_output = pid->output;
    pid->previous_d_output = pid->d_output;
    if (pid->sample_count < 2U)
    {
        ++pid->sample_count;
    }

    *output = pid->output;
    return pid_record_status(pid, PID_STATUS_OK);
}

PidStatus pid_get_component_outputs(const PidController *pid,
                                    float *p_output,
                                    float *i_output,
                                    float *d_output)
{
    if (pid == NULL)
    {
        return PID_STATUS_NULL_POINTER;
    }
    if (!pid->initialized)
    {
        return PID_STATUS_NOT_INITIALIZED;
    }
    if ((p_output == NULL) && (i_output == NULL) && (d_output == NULL))
    {
        return PID_STATUS_NULL_POINTER;
    }

    if (p_output != NULL)
    {
        *p_output = pid->p_output;
    }
    if (i_output != NULL)
    {
        *i_output = pid->i_output;
    }
    if (d_output != NULL)
    {
        *d_output = pid->d_output;
    }
    return PID_STATUS_OK;
}

/* -------------------------------------------------------------------------- */
/* 旧基础 PID 源码兼容层                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief 清除兼容结构中的运行状态，不修改模式和控制参数。
 */
static void pid_compatibility_clear_runtime(PidTypedef *pid)
{
    pid->set = 0.0F;
    pid->fdb = 0.0F;
    pid->out = 0.0F;
    pid->Pout = 0.0F;
    pid->Iout = 0.0F;
    pid->Dout = 0.0F;
    pid->Dbuf[0] = 0.0F;
    pid->Dbuf[1] = 0.0F;
    pid->Dbuf[2] = 0.0F;
    pid->error[0] = 0.0F;
    pid->error[1] = 0.0F;
    pid->error[2] = 0.0F;
}

void PID_init(PidTypedef *pid,
              uint8_t mode,
              const float gains[3],
              float max_out,
              float max_iout)
{
    if ((pid == NULL) || (gains == NULL))
    {
        return;
    }
    if (((mode != (uint8_t)PID_POSITION) &&
         (mode != (uint8_t)PID_DELTA)) ||
        !pid_is_finite(gains[0]) ||
        !pid_is_finite(gains[1]) ||
        !pid_is_finite(gains[2]) ||
        !pid_is_finite(max_out) ||
        !pid_is_finite(max_iout) ||
        (max_out < 0.0F) ||
        (max_iout < 0.0F))
    {
        return;
    }

    /* 保留旧模块 DEBUG 模式下重复调用 PID_init() 可在线更新增益的行为。 */
    if (pid->Initlized)
    {
        pid->Kp = gains[0];
        pid->Ki = gains[1];
        pid->Kd = gains[2];
        return;
    }

    pid->pid_mode = (PID_mode_e)mode;
    pid->Kp = gains[0];
    pid->Ki = gains[1];
    pid->Kd = gains[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid_compatibility_clear_runtime(pid);
    pid->Initlized = true;
}

float PID_Calc(PidTypedef *pid, float fdb, float ref)
{
    PidController controller;
    PidStatus status;
    PidMode mode;
    float output = 0.0F;
    float previous_error;
    float previous_error_2;
    float previous_dbuf;
    float previous_dbuf_2;

    if (pid == NULL)
    {
        return 0.0F;
    }
    if (!pid->Initlized)
    {
        pid_compatibility_clear_runtime(pid);
        return 0.0F;
    }
    if (((pid->pid_mode != PID_POSITION) &&
         (pid->pid_mode != PID_DELTA)) ||
        !pid_is_finite(pid->Kp) ||
        !pid_is_finite(pid->Ki) ||
        !pid_is_finite(pid->Kd) ||
        !pid_is_finite(pid->max_out) ||
        !pid_is_finite(pid->max_iout) ||
        (pid->max_out < 0.0F) ||
        (pid->max_iout < 0.0F))
    {
        PID_clear(pid);
        return 0.0F;
    }

    previous_error = pid->error[0];
    previous_error_2 = pid->error[1];
    previous_dbuf = pid->Dbuf[0];
    previous_dbuf_2 = pid->Dbuf[1];
    mode = (pid->pid_mode == PID_POSITION)
               ? PID_POSITIONAL_MODE
               : PID_INCREMENTAL_MODE;

    status = pid_init(&controller, mode, pid->Kp, pid->Ki, pid->Kd);
    if ((status == PID_STATUS_OK) && (pid->pid_mode == PID_POSITION))
    {
        status = pid_set_integral_limit(&controller, pid->max_iout);
    }
    if ((status == PID_STATUS_OK) && (pid->max_out > 0.0F))
    {
        status = pid_set_output_limit(&controller,
                                      -pid->max_out,
                                      pid->max_out);
    }

    if (status != PID_STATUS_OK)
    {
        PID_clear(pid);
        return 0.0F;
    }

    /*
     * 兼容接口没有周期参数，因此使用归一化周期 1.0。这样旧 Kp/Ki/Kd
     * 仍保持“每调用一次”的离散含义，同时计算逻辑全部复用新内核。
     */
    controller.sample_count = 2U;
    controller.previous_error = previous_error;
    controller.previous_error_2 = previous_error_2;
    controller.previous_output = pid->out;
    controller.i_output = pid->Iout;

    status = pid_compute(&controller, ref, fdb, 1.0F, &output);
    if (status != PID_STATUS_OK)
    {
        PID_clear(pid);
        return 0.0F;
    }

    /* max_out == 0 在旧模块中表示输出被钳制为零。 */
    if (pid->max_out == 0.0F)
    {
        output = 0.0F;
    }

    pid->set = ref;
    pid->fdb = fdb;
    pid->error[2] = previous_error_2;
    pid->error[1] = previous_error;
    pid->error[0] = controller.error;
    pid->Dbuf[2] = previous_dbuf_2;
    pid->Dbuf[1] = previous_dbuf;
    pid->Dbuf[0] = (pid->pid_mode == PID_POSITION)
                       ? (pid->error[0] - pid->error[1])
                       : (pid->error[0] -
                          (2.0F * pid->error[1]) +
                          pid->error[2]);
    pid->Pout = controller.p_output;
    pid->Iout = controller.i_output;
    pid->Dout = controller.d_output;
    pid->out = output;
    return pid->out;
}

void PID_clear(PidTypedef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid_compatibility_clear_runtime(pid);
    pid->Initlized = false;
}
