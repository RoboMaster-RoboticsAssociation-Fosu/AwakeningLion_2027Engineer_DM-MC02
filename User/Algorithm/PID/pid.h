/**
 * @file pid.h
 * @brief 通用 PID 控制器接口。
 *
 * 本模块由 myCode PID 重构移植而来。与原实现相比：
 * - 计算周期由调用方显式传入，单位固定为秒，避免毫秒时间戳误用；
 * - 优化功能使用整型掩码，避免 C 位域布局依赖编译器；
 * - 所有可失败接口返回状态码，不再通过死循环处理空指针；
 * - 位置式与增量式 PID 共用一致的参数校验和状态管理。
 */

#ifndef USER_ALGORITHM_PID_PID_H_
#define USER_ALGORITHM_PID_PID_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID 接口执行状态。
 */
typedef enum
{
    PID_STATUS_OK = 0,              /**< 执行成功。 */
    PID_STATUS_NULL_POINTER,        /**< 必需指针为空。 */
    PID_STATUS_NOT_INITIALIZED,     /**< 控制器尚未初始化。 */
    PID_STATUS_INVALID_MODE,        /**< PID 模式无效。 */
    PID_STATUS_INVALID_PARAMETER,   /**< 参数无效或超出允许范围。 */
    PID_STATUS_INVALID_PERIOD,      /**< 计算周期不是有限正数。 */
    PID_STATUS_NUMERIC_ERROR        /**< 计算结果出现 NaN 或无穷大。 */
} PidStatus_e;

/**
 * @brief PID 离散形式。
 */
typedef enum
{
    PID_POSITIONAL_MODE = 0,        /**< 位置式 PID，输出为各分量之和。 */
    PID_INCREMENTAL_MODE = 1        /**< 增量式 PID，输出在上次结果上累加。 */
} PidMode_e;

/**
 * @brief PID 优化功能掩码。
 *
 * 多个功能可以按位或组合。外部接管某分量后，调用方应在每次计算前
 * 通过 pid_set_external_outputs() 更新对应分量。
 */
typedef enum
{
    PID_IMPROVEMENT_NONE = 0U,
    PID_IMPROVEMENT_OUTPUT_LIMIT = (1U << 0),
    PID_IMPROVEMENT_INTEGRAL_LIMIT = (1U << 1),
    PID_IMPROVEMENT_DEAD_BAND = (1U << 2),
    PID_IMPROVEMENT_DERIVATIVE_ON_MEASUREMENT = (1U << 3),
    PID_IMPROVEMENT_DERIVATIVE_FILTER = (1U << 4),
    PID_IMPROVEMENT_VARIABLE_INTEGRAL = (1U << 5),
    PID_IMPROVEMENT_EXTERNAL_P = (1U << 6),
    PID_IMPROVEMENT_EXTERNAL_I = (1U << 7),
    PID_IMPROVEMENT_EXTERNAL_D = (1U << 8)
} PidImprovement_e;

/** @brief 当前实现支持的全部优化功能。 */
#define PID_IMPROVEMENT_ALL_MASK ((uint16_t)0x01FFU)

/**
 * @brief PID 优化参数。
 */
typedef struct
{
    float output_min;                         /**< 输出下限，必须小于 output_max。 */
    float output_max;                         /**< 输出上限。 */
    float integral_limit;                     /**< 积分分量绝对值上限，必须非负。 */
    float dead_band;                          /**< 误差死区绝对值，必须非负。 */
    float derivative_filter_time_constant_s;  /**< 微分一阶低通时间常数，单位秒且必须为正。 */
    float variable_integral_threshold_a;      /**< 变速积分线性衰减区宽度，必须为正。 */
    float variable_integral_threshold_b;      /**< 变速积分全增益区边界，必须非负。 */
} PidImprovementParams_t;

/**
 * @brief PID 控制器实例。
 *
 * 参数和运行状态公开，便于嵌入式在线观测。除 p_output、i_output、d_output
 * 在启用外部接管时允许由调用方写入外，其余运行状态应由本模块维护。
 */
typedef struct
{
    /* 基本配置。 */
    float kp;
    float ki;
    float kd;
    PidMode_e mode;
    uint16_t improvement_flags;
    PidImprovementParams_t improvement;

    /* 生命周期与诊断状态。 */
    bool initialized;
    uint8_t sample_count;                     /**< 有效历史样本数，最大为 2。 */
    PidStatus_e last_status;

    /* 当前输入、输出与误差。 */
    float setpoint;
    float measurement;
    float output;
    float error;
    float period_s;

    /* 历史状态。 */
    float previous_error;
    float previous_error_2;
    float previous_measurement;
    float previous_measurement_2;
    float previous_output;
    float previous_d_output;

    /* 分量输出与本周期积分增量。 */
    float p_output;
    float i_output;
    float d_output;
    float integral_increment;
} PidController_t;

/**
 * @brief 初始化 PID 控制器并清空全部运行状态。
 * @param pid 控制器实例。
 * @param mode 位置式或增量式 PID。
 * @param kp 比例系数，必须为有限数。
 * @param ki 积分系数，必须为有限数。
 * @param kd 微分系数，必须为有限数。
 * @return PID_STATUS_OK 表示初始化成功，其他值表示参数错误。
 */
PidStatus_e pid_init(PidController_t *pid, PidMode_e mode, float kp, float ki, float kd);

/**
 * @brief 更新 PID 基本系数，不清空运行状态。
 */
PidStatus_e pid_set_params(PidController_t *pid, float kp, float ki, float kd);

/**
 * @brief 设置并启用非对称输出限幅。
 */
PidStatus_e pid_set_output_limit(PidController_t *pid, float min_limit, float max_limit);

/**
 * @brief 设置并启用积分分量限幅。
 */
PidStatus_e pid_set_integral_limit(PidController_t *pid, float limit);

/**
 * @brief 设置并启用误差死区。
 *
 * 位置式 PID 进入死区后保留已有积分分量，避免输出偏置突然丢失；
 * 增量式 PID 在没有外部分量时保持上次输出。
 */
PidStatus_e pid_set_dead_band(PidController_t *pid, float dead_band);

/**
 * @brief 设置并启用微分一阶低通滤波。
 * @param time_constant_s 滤波时间常数，单位秒且必须大于 0。
 */
PidStatus_e pid_set_derivative_filter_time_constant(PidController_t *pid,
                                                  float time_constant_s);

/**
 * @brief 启用或禁用微分先行，即对测量值而不是误差做微分。
 */
PidStatus_e pid_set_derivative_first_enable(PidController_t *pid, bool enable);

/**
 * @brief 设置并启用变速积分阈值。
 *
 * |error| <= B 时使用完整积分；B < |error| < A+B 时线性衰减；
 * |error| >= A+B 时停止积分。
 */
PidStatus_e pid_set_variable_integral_thresholds(PidController_t *pid,
                                               float threshold_a,
                                               float threshold_b);

/**
 * @brief 按掩码统一启用或禁用优化功能。
 *
 * 对需要参数的功能，应优先调用对应 setter 完成参数设置并启用；直接启用时
 * 如果现有参数无效，本函数会拒绝修改。
 */
PidStatus_e pid_set_improvement_enabled(PidController_t *pid,
                                      uint16_t improvement_mask,
                                      bool enable);

/**
 * @brief 写入外部提供的 P、I、D 分量。
 *
 * 只有启用相应 PID_IMPROVEMENT_EXTERNAL_* 标志的分量会被写入；未接管的
 * 内部分量保持不变。三个外部接管标志均未启用时返回参数错误。
 */
PidStatus_e pid_set_external_outputs(PidController_t *pid,
                                   float p_output,
                                   float i_output,
                                   float d_output);

/**
 * @brief 清空全部运行状态，同时保留参数、优化配置和初始化状态。
 */
PidStatus_e pid_reset(PidController_t *pid);

/**
 * @brief 执行一次 PID 计算。
 * @param pid 已初始化的控制器实例。
 * @param setpoint 设定值，必须为有限数。
 * @param measurement 测量值，必须为有限数。
 * @param period_s 本次计算周期，单位秒，必须为有限正数。
 * @param output 输出结果指针；失败时写入 0（指针有效时）。
 * @return PID_STATUS_OK 表示计算成功。
 */
PidStatus_e pid_compute(PidController_t *pid,
                      float setpoint,
                      float measurement,
                      float period_s,
                      float *output);

/**
 * @brief 读取最近一次计算的 P、I、D 分量。
 *
 * 不关心的分量允许传入 NULL，但三个输出指针不能同时为 NULL。
 */
PidStatus_e pid_get_component_outputs(const PidController_t *pid,
                                    float *p_output,
                                    float *i_output,
                                    float *d_output);

#ifdef __cplusplus
}
#endif

#endif /* USER_ALGORITHM_PID_PID_H_ */
