/**
 * @file QD4310_Motor.h
 * @brief QD4310 公共 CAN 驱动：单帧命令、五种控制模式和反馈发布。
 * @note 依据 QD4310 使用手册 V2.6 第 21-25 页；经典 CAN 标准帧，1 Mbit/s。
 *       总线初始化、使能顺序、发送周期、重试、在线判断和机械限位由所属任务负责。
 *       本模块不自动使能，不提供设置零点、清错、重启或 UART 功能。
 */
#ifndef QD4310_MOTOR_H
#define QD4310_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 协议常量和操作类型 ------------------------------------------------------ */
#define QD4310_MOTOR_CONTROL_BYTES 3U
#define QD4310_MOTOR_FEEDBACK_BYTES 8U
#define QD4310_MOTOR_ID_MAX 0x0FU
#define QD4310_MOTOR_CURRENT_MAX_A 10.0f
/* 手册 V2.6 第 23 页：Q 轴电流与轴端力矩按 0.27 N·m/A 换算。
 * ±2.7 N·m 仅对应协议 ±10 A 的给定量程，不是电机可持续或峰值机械力矩。
 */
#define QD4310_MOTOR_TORQUE_CONSTANT_NM_PER_A 0.27f
#define QD4310_MOTOR_TORQUE_MAX_NM \
    (QD4310_MOTOR_TORQUE_CONSTANT_NM_PER_A * QD4310_MOTOR_CURRENT_MAX_A)
/* 纯物理量换算，参数各求值一次；不执行限幅或 CAN 发送。 */
#define QD4310_MOTOR_CURRENT_TO_TORQUE_NM(current_a) \
    ((current_a) * QD4310_MOTOR_TORQUE_CONSTANT_NM_PER_A)
#define QD4310_MOTOR_TORQUE_TO_CURRENT_A(torque_nm) \
    ((torque_nm) / QD4310_MOTOR_TORQUE_CONSTANT_NM_PER_A)
#define QD4310_MOTOR_TURN_RAD 6.2831853071795864769f
#define QD4310_MOTOR_VELOCITY_MAX_RAD_S (1000.0f * QD4310_MOTOR_TURN_RAD / 60.0f)

/** @brief 返回值描述本次处理结果；OK/CLAMPED 发送只证明 HAL 已接受入队。 */
typedef enum
{
    QD4310_MOTOR_OK = 0,
    QD4310_MOTOR_CLAMPED,
    QD4310_MOTOR_INVALID_ARGUMENT,
    QD4310_MOTOR_BAD_FRAME,
    QD4310_MOTOR_SEND_FAILED
} QD4310_Motor_Status_e;

/** @brief 基础管理指令；反馈查询不会修改电机控制目标。 */
typedef enum
{
    QD4310_MOTOR_COMMAND_QUERY = 0x00,
    QD4310_MOTOR_COMMAND_ENABLE = 0x01,
    QD4310_MOTOR_COMMAND_DISABLE = 0x02
} QD4310_Motor_Command_e;

/** @brief 控制量的单位由模式确定；每条控制报文本身选择电机运行模式。 */
typedef enum
{
    QD4310_MOTOR_MODE_CURRENT = 0x03,       /* 电流 A，范围 [-10, 10]。 */
    QD4310_MOTOR_MODE_SPEED = 0x04,         /* 速度 rad/s，对应 [-1000, 1000] rpm。 */
    QD4310_MOTOR_MODE_POSITION = 0x05,      /* 单圈绝对角度 rad，范围 [0, 2*pi]。 */
    QD4310_MOTOR_MODE_LOW_SPEED = 0x06,     /* 低速 rad/s，编码与速度模式相同。 */
    QD4310_MOTOR_MODE_POSITION_STEP = 0x07  /* 增量角度 rad，范围 [-2*pi, 2*pi]。 */
} QD4310_Motor_Mode_e;

/* 固定配置和运行数据 ------------------------------------------------------ */
/** @brief ID 必须与上位机配置一致；收发开始后不得直接改写配置。 */
typedef struct
{
    uint8_t motor_id;                      /* 电机 ID 0x00-0x0F，不是 0x400/0x500 帧 ID。 */
} QD4310_Motor_Config_t;

/** @brief 最近一次合法反馈；手册未定义故障位，保留原始状态供后续核对。 */
typedef struct
{
    float current_a;                       /* 有符号 Q 轴电流 A。 */
    float velocity_rad_s;                  /* 有符号速度 rad/s。 */
    float position_rad;                    /* 单圈绝对角度 rad。 */
    bool enabled;                         /* 状态 Byte0 bit0，不由发送成功推断。 */
    uint8_t raw_status;                    /* 完整 Byte0，bit7-1 不解释。 */
    uint8_t reserved;                      /* 原始 Byte1，不解释。 */
} QD4310_Motor_Feedback_t;

/** @brief valid 表示曾收到合法帧；当前是否在线由任务使用时间戳判断。 */
typedef struct
{
    QD4310_Motor_Feedback_t data;
    bool valid;
    uint32_t last_update_ms;               /* 调用者提供的接收时间 ms。 */
    uint32_t sequence;                     /* 每个合法反馈递增，允许 uint32_t 回绕。 */
} QD4310_Motor_FeedbackState_t;

/** @brief 一台电机的完整实例；中断更新与任务读取的一致性由所属任务保护。 */
typedef struct
{
    QD4310_Motor_Config_t config;
    QD4310_Motor_FeedbackState_t feedback;
} QD4310_Motor_Info_t;

/**
 * @brief 固定 ID 创建实例，参数只求值一次，其余成员由 C 零初始化。
 * @note 不初始化总线、不发命令；运行期收发仍检查 ID。
 */
#define QD4310_MOTOR_CREATE(motor_id_value) \
    { .config = { .motor_id = (motor_id_value) } }

/* 发送和接收接口 ---------------------------------------------------------- */
/**
 * @brief 发送反馈查询、使能或失能，ID=0x400+motor_id，DLC=3。
 * @note 每次只提交一帧，Byte1-2 清零；非法指令拒绝，不延时或重试。
 *       使能需要电机完成校准，是否使能以新的反馈为准。
 */
QD4310_Motor_Status_e QD4310_Motor_SendCommand(FDCAN_HandleTypeDef *hcan,
                                            const QD4310_Motor_Info_t *motor,
                                            QD4310_Motor_Command_e command);

/**
 * @brief 选择一种模式并发送目标值；value 单位见 QD4310_Motor_Mode_e。
 * @param applied 可选输出，限幅后、整数编码前的目标值；NULL 表示不读取。
 * @note 有限越界钳位，NaN/Inf 拒绝；绝对角度不自动取模。
 *       有符号编码使用 +/-32767、绝对角度使用 0-65535，内部量化向零截断。
 *       只有帧成功入队才返回 OK/CLAMPED 并写 applied，失败保持输出不变。
 *       此处限制是协议量程；电机额定 1 A、峰值 2 A，任务须另设运行限制。
 *       电机已有 limit.current 等配置仍会限制实际输出，applied 不等于实际执行值。
 *       绝对角度由电机沿较近方向运动；步进每发一帧都是一次新的增量，任务慎重重试。
 */
QD4310_Motor_Status_e QD4310_Motor_Control(FDCAN_HandleTypeDef *hcan,
                                        const QD4310_Motor_Info_t *motor,
                                        QD4310_Motor_Mode_e mode,
                                        float value, float *applied);

/**
 * @brief 目标轴端力矩 N·m → 协议限幅 → Q 轴电流 A → 一次电流模式 CAN 发送。
 * @param torque_nm 有符号目标力矩，按手册转矩常数 0.27 N·m/A 换算。
 * @param applied_torque_nm 可选输出，限幅后、整数编码前的目标力矩；NULL 可省略。
 * @note 有限越界按协议 ±QD4310_MOTOR_TORQUE_MAX_NM 限幅，NaN/Inf 拒绝且不发送。
 *       只有入队成功返回 OK/CLAMPED 时才写输出；任何失败保留此前的输出值。
 *       采用现有电流模式及向零截断规则，不自动使能、不等待、不重试。
 *       额定/峰值力矩、机械限位和电机 limit.current 由任务及电机端配置约束。
 *       本接口不计算位置闭环，返回目标力矩不代表实测或已执行力矩。
 */
QD4310_Motor_Status_e QD4310_Motor_ControlTorque(FDCAN_HandleTypeDef *hcan,
                                              const QD4310_Motor_Info_t *motor,
                                              float torque_nm,
                                              float *applied_torque_nm);

/**
 * @brief 校验并发布 ID=0x500+motor_id 的 8 字节反馈。
 * @param data_length 实际负载字节数，必须为 8，不是缓冲容量或 HAL DLC 编码。
 * @param now_ms 接收时间 ms，由调用者提供。
 * @note 核对经典 CAN 标准数据帧、ID、HAL DLC 及实际长度，坏帧不改变实例。
 *       不发送命令、不判断在线、不累计多圈角度，可在 CAN 接收中断中调用。
 *       有符号 -32768 按手册的 /32767 比例解析，不静默钳位收到的测量值。
 */
QD4310_Motor_Status_e QD4310_Motor_OnRx(QD4310_Motor_Info_t *motor,
                                     const FDCAN_RxHeaderTypeDef *header,
                                     const uint8_t *data, uint32_t data_length,
                                     uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif /* QD4310_MOTOR_H */
