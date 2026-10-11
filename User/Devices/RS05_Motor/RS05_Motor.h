/**
 * @file RS05_Motor.h
 * @brief RS05 电机驱动公共接口：私有协议与 MIT 协议共用一个实例和模块。
 * @note 依据 RS05使用说明书260713.pdf；经典 CAN 数据帧，C99 接口。
 *
 * 通信能力：
 *   私有协议：经典 CAN 扩展帧；运控、电流、速度、PP/CSP 位置控制。
 *   MIT 协议：经典 CAN 标准帧；运控、位置、速度控制。
 *   两者共用：使能、失能、清错、标零、设置模式、参数读写、主动上报、反馈接收。
 *   实例 config.protocol 选择通信协议，必须与电机现有配置一致，不自动切协议。
 *
 * 调用约束：
 *   总线由所属任务初始化，波特率应与电机一致，出厂默认 1 Mbit/s。
 *   所有发送接口每次只提交一帧，不延时、不重试、不自动使能或切模式。
 *   私有电流/速度/位置目标通过 WriteParameter 写入；限制和加速度单独配置。
 *   编解码与发送细节位于 RS05_Motor.c 内部，调用者无需手工构造 CAN 帧。
 */
#ifndef RS05_MOTOR_H
#define RS05_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 协议常量、枚举与数据类型 ------------------------------------------------ */

#define RS05_MOTOR_FRAME_BYTES 8U
#define RS05_MOTOR_POSITION_MAX 12.57f
#define RS05_MOTOR_VELOCITY_MAX 50.0f
#define RS05_MOTOR_TORQUE_MAX 5.5f
#define RS05_MOTOR_KP_MAX 500.0f
#define RS05_MOTOR_KD_MAX 5.0f

/** @brief 发送结果 OK/CLAMPED 表示 HAL 入队成功；不表示电机已执行或确认。 */
typedef enum
{
    RS05_MOTOR_OK = 0,
    RS05_MOTOR_CLAMPED,
    RS05_MOTOR_INVALID_ARGUMENT,
    RS05_MOTOR_UNSUPPORTED,
    RS05_MOTOR_NOT_FOR_DEVICE,
    RS05_MOTOR_PROTOCOL_ERROR,
    RS05_MOTOR_REMOTE_ERROR,
    RS05_MOTOR_SEND_FAILED
} RS05_Motor_Status_e;

typedef enum
{
    RS05_MOTOR_PROTOCOL_PRIVATE = 0,
    RS05_MOTOR_PROTOCOL_MIT
} RS05_Motor_Protocol_e;

/** @brief 运行模式；MIT 支持运控/速度/CSP，CSP 的协议模式值映射为 1。 */
typedef enum
{
    RS05_MOTOR_MODE_MOTION = 0,
    RS05_MOTOR_MODE_POSITION_PP = 1,
    RS05_MOTOR_MODE_SPEED = 2,
    RS05_MOTOR_MODE_CURRENT = 3,
    RS05_MOTOR_MODE_POSITION_CSP = 5
} RS05_Motor_Mode_e;

typedef enum
{
    RS05_MOTOR_VALUE_U8 = 0,
    RS05_MOTOR_VALUE_U16,
    RS05_MOTOR_VALUE_U32,
    RS05_MOTOR_VALUE_FLOAT
} RS05_Motor_ValueType_e;

/** @brief 手册第 37～38 页参数索引；驱动核对类型、读写权限和协议能力。 */
typedef enum
{
    RS05_MOTOR_PARAM_RUN_MODE = 0x7005,
    RS05_MOTOR_PARAM_IQ_REF = 0x7006,
    RS05_MOTOR_PARAM_SPD_REF = 0x700A,
    RS05_MOTOR_PARAM_LIMIT_TORQUE = 0x700B,
    RS05_MOTOR_PARAM_CUR_KP = 0x7010,
    RS05_MOTOR_PARAM_CUR_KI = 0x7011,
    RS05_MOTOR_PARAM_CUR_FILT_GAIN = 0x7014,
    RS05_MOTOR_PARAM_LOC_REF = 0x7016,
    RS05_MOTOR_PARAM_LIMIT_SPD = 0x7017,
    RS05_MOTOR_PARAM_LIMIT_CUR = 0x7018,
    RS05_MOTOR_PARAM_MECH_POS = 0x7019,
    RS05_MOTOR_PARAM_IQF = 0x701A,
    RS05_MOTOR_PARAM_MECH_VEL = 0x701B,
    RS05_MOTOR_PARAM_VBUS = 0x701C,
    RS05_MOTOR_PARAM_LOC_KP = 0x701E,
    RS05_MOTOR_PARAM_SPD_KP = 0x701F,
    RS05_MOTOR_PARAM_SPD_KI = 0x7020,
    RS05_MOTOR_PARAM_SPD_FILT_GAIN = 0x7021,
    RS05_MOTOR_PARAM_ACC_RAD = 0x7022,
    RS05_MOTOR_PARAM_VEL_MAX = 0x7024,
    RS05_MOTOR_PARAM_ACC_SET = 0x7025,
    RS05_MOTOR_PARAM_REPORT_TIME = 0x7026,
    RS05_MOTOR_PARAM_CAN_TIMEOUT = 0x7028,
    RS05_MOTOR_PARAM_ZERO_STA = 0x7029,
    RS05_MOTOR_PARAM_ADD_OFFSET = 0x702B,
    RS05_MOTOR_PARAM_ALVEOLOUS_OPEN = 0x702C,
    RS05_MOTOR_PARAM_IQ_TEST = 0x702D,
    RS05_MOTOR_PARAM_DCC_SET = 0x702E
} RS05_Motor_Parameter_e;

/** @brief 私有类型 2/24 的六位故障摘要；MIT 只报告是否有故障/预警。 */
typedef enum
{
    RS05_MOTOR_FAULT_UNDERVOLTAGE = 1U << 0,
    RS05_MOTOR_FAULT_PHASE_CURRENT = 1U << 1,
    RS05_MOTOR_FAULT_OVERTEMPERATURE = 1U << 2,
    RS05_MOTOR_FAULT_ENCODER = 1U << 3,
    RS05_MOTOR_FAULT_OVERLOAD = 1U << 4,
    RS05_MOTOR_FAULT_UNCALIBRATED = 1U << 5
} RS05_Motor_Fault_e;

/** @brief 固定通信配置；协议应与电机实际设置一致，不自动切换硬件协议。 */
typedef struct
{
    RS05_Motor_Protocol_e protocol;
    uint8_t motor_id;
    uint8_t host_id;
} RS05_Motor_Config_t;

/** @brief 有类型参数值；type 必须与指定索引的手册类型一致。 */
typedef struct
{
    RS05_Motor_ValueType_e type;
    union
    {
        uint8_t u8;
        uint16_t u16;
        uint32_t u32;
        float real;
    } data;
} RS05_Motor_Value_t;

/** @brief 运动反馈；state 为 0 复位/1 标定/2 运行，不等于运行模式参数。 */
typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;
    uint8_t state;
    uint8_t fault_bits;                         /* 仅私有协议有效；MIT 不伪造详细故障位。 */
    bool fault;
    bool warning;                               /* 仅 MIT 的运动反馈携带预警标志。 */
} RS05_Motor_Feedback_t;

/** @brief 一次参数回复；仅保存参数事实，不混入运动反馈或故障负载。 */
typedef struct
{
    uint16_t index;                             /* 参数索引。 */
    bool write_ack;                             /* true 为 MIT 写回复，false 为读回复。 */
    RS05_Motor_Value_t value;                   /* 按索引类型解码的参数值。 */
} RS05_Motor_ParameterReply_t;

/* 运行数据 ---------------------------------------------------------------- */

/** @brief 运动反馈及其接收证据；valid 只表示曾收到合法反馈，不等于当前在线。 */
typedef struct
{
    RS05_Motor_Feedback_t data;                 /* 最近一次位置、速度、力矩、温度和状态。 */
    bool valid;                                 /* 已收到至少一帧合法运动反馈。 */
    uint32_t last_update_ms;                    /* 最近一次运动反馈的接收时间，单位 ms。 */
    uint32_t sequence;                          /* 每收到一帧合法运动反馈递增，允许回绕。 */
} RS05_Motor_FeedbackState_t;

/** @brief 参数回复及其接收证据；更新本组不刷新运动反馈的时间或序号。 */
typedef struct
{
    RS05_Motor_ParameterReply_t data;           /* 最近一次参数索引、值和回复类型。 */
    bool valid;                                 /* 已收到至少一帧合法参数回复。 */
    uint32_t last_update_ms;                    /* 最近一次参数回复的接收时间，单位 ms。 */
    uint32_t sequence;                          /* 每收到一帧合法参数回复递增，允许回绕。 */
} RS05_Motor_ParameterState_t;

/** @brief 私有类型 21 的原始故障记录；不猜测手册未明确的多字节端序。 */
typedef struct
{
    uint8_t raw[RS05_MOTOR_FRAME_BYTES];        /* 最近一次原始故障负载。 */
    bool valid;                                 /* 已收到至少一帧合法原始故障帧。 */
    uint32_t last_update_ms;                    /* 最近一次原始故障帧的接收时间，单位 ms。 */
} RS05_Motor_FaultState_t;

/* 电机实例 ---------------------------------------------------------------- */

/**
 * @brief 一台 RS05 的完整实例；固定配置与三类接收状态按职责分组。
 * @note 开始收发后不直接修改 config；接收写入与任务读取的一致性由所属任务保护。
 */
typedef struct
{
    RS05_Motor_Config_t config;                 /* 固定通信配置：协议、电机 ID、主机 ID。 */
    RS05_Motor_FeedbackState_t feedback;        /* 运动反馈及新鲜度证据。 */
    RS05_Motor_ParameterState_t parameter;      /* 参数回复及接收证据。 */
    RS05_Motor_FaultState_t fault;              /* 原始故障记录及接收证据。 */
} RS05_Motor_Info_t;

/* 创建辅助 ---------------------------------------------------------------- */

/**
 * @brief 用固定协议、电机 ID 和主机 ID 创建实例，每个宏参数只求值一次。
 * @note 使用 C99 指定初始化，其余数据/有效位/时间/序号由 C 自动清零，无需额外 Init。
 *       本宏不校验配置、不配置总线、不发送命令；首次收发仍须校验通信配置。
 * @code
 * static RS05_Motor_Info_t motor = RS05_MOTOR_CREATE(RS05_MOTOR_PROTOCOL_PRIVATE, 0x7F, 0xFD);
 * @endcode
 */
#define RS05_MOTOR_CREATE(protocol_value, motor_id_value, host_id_value) \
    { \
        .config = { \
            .protocol = (protocol_value), \
            .motor_id = (motor_id_value), \
            .host_id = (host_id_value), \
        }, \
    }

/* 常用管理接口：私有协议和 MIT 均支持 ------------------------------------ */

/**
 * @brief 校验协议及 ID 配置；要求主机 ID 与电机 ID 不同，失败不修改配置。
 * @note 无需 Init；每个收发接口仍检查配置，创建宏不能代替运行期校验。
 */
RS05_Motor_Status_e RS05_Motor_ValidateConfig(const RS05_Motor_Config_t *config);

/**
 * @brief 使能电机，按 config.protocol 发送私有类型 3 或 MIT 的 FF...FC。
 * @note 发送成功不等于已使能；实际状态由新反馈的 state 确认。
 */
RS05_Motor_Status_e RS05_Motor_Enable(FDCAN_HandleTypeDef *hcan,
                                    const RS05_Motor_Info_t *motor);

/**
 * @brief 失能并停止运行，发送私有类型 4 或 MIT 的 FF...FD。
 * @note 本操作不清故障；切换运行模式前由所属任务调用并确认停止。
 */
RS05_Motor_Status_e RS05_Motor_Disable(FDCAN_HandleTypeDef *hcan,
                                     const RS05_Motor_Info_t *motor);

/**
 * @brief 清故障：私有类型 4 的 Byte0=1；MIT 发送 FF FF FF FF FF FF FF FB。
 * @note 私有清错命令同时停止电机；MIT 不保证停止，需停止时先单独调用 Disable。
 */
RS05_Motor_Status_e RS05_Motor_ClearFault(FDCAN_HandleTypeDef *hcan,
                                        const RS05_Motor_Info_t *motor);

/**
 * @brief 将当前位置设为机械零位，发送私有类型 6 或 MIT 的 FF...FE。
 * @note 由任务在停止后调用；私有 PP 模式不支持标零，MIT 位置模式不支持标零。
 *       本地未缓存运行模式，不根据发送成功推断硬件模式或自动切换。
 */
RS05_Motor_Status_e RS05_Motor_SetZero(FDCAN_HandleTypeDef *hcan,
                                     const RS05_Motor_Info_t *motor);

/**
 * @brief 设置运行模式：私有写 run_mode；MIT 使用专用运行模式命令。
 * @param mode 私有支持全部 RS05_Motor_Mode_e；MIT 支持 MOTION/SPEED/POSITION_CSP。
 * @note 必须先停止；非法模式返回 INVALID_ARGUMENT，不支持组合返回 UNSUPPORTED。
 *       每次只发模式设置帧，不自动失能、使能或下发控制目标。
 */
RS05_Motor_Status_e RS05_Motor_SetMode(FDCAN_HandleTypeDef *hcan,
                                     const RS05_Motor_Info_t *motor,
                                     RS05_Motor_Mode_e mode);

/**
 * @brief 开关主动上报：私有类型 24，MIT 的 FF FF FF FF FF FF 00/01 F9。
 * @note MIT 要求固件 >= 0.5.0.13；私有上报周期可通过 REPORT_TIME 参数单独设置。
 */
RS05_Motor_Status_e RS05_Motor_SetActiveReport(FDCAN_HandleTypeDef *hcan,
                                             const RS05_Motor_Info_t *motor,
                                             bool enabled);

/* 运控接口：私有协议和 MIT 均支持 ---------------------------------------- */

/**
 * @brief 下发位置、速度、Kp、Kd、前馈力矩，按实例协议完成编码并发送。
 * @param position_rad 位置 rad，编码量程 [-12.57, 12.57]。
 * @param velocity_rad_s 速度 rad/s，编码量程 [-50, 50]。
 * @param kp 位置增益 [0, 500]。
 * @param kd 速度增益 [0, 5]。
 * @param torque_nm 前馈力矩 N·m，编码量程 [-5.5, 5.5]。
 * @note 要求电机已设为运控并使能；有限越界钳位，NaN/Inf 拒绝且不发送。
 *       整数映射采用截断，MIT 12 位满量程为 4095；管理指令碰撞拒绝发送。
 *       CLAMPED 仅在钳位后的帧成功入队时返回，入队失败优先返回 SEND_FAILED。
 */
RS05_Motor_Status_e RS05_Motor_ControlMotion(FDCAN_HandleTypeDef *hcan,
                                           const RS05_Motor_Info_t *motor,
                                           float position_rad, float velocity_rad_s,
                                           float kp, float kd, float torque_nm);

/* MIT 浮点控制接口：私有协议返回 UNSUPPORTED ----------------------------- */

/**
 * @brief MIT 位置控制，发送小端 float 目标位置及速度限制，不进行运控量程钳位。
 * @param speed_limit_rad_s 速度绝对值 rad/s，必须非负且有限。
 * @note 要求 POSITION_CSP 已设置并使能；拒绝 NaN/Inf，不自动切模式。
 */
RS05_Motor_Status_e RS05_Motor_MitControlPosition(FDCAN_HandleTypeDef *hcan,
                                                const RS05_Motor_Info_t *motor,
                                                float position_rad, float speed_limit_rad_s);

/**
 * @brief MIT 速度控制，发送小端 float 速度和电流限制，不进行运控量程钳位。
 * @param current_limit_a 电流绝对值 A，必须非负且有限。
 * @note 要求 SPEED 已设置并使能；拒绝 NaN/Inf，不自动切模式。
 */
RS05_Motor_Status_e RS05_Motor_MitControlSpeed(FDCAN_HandleTypeDef *hcan,
                                             const RS05_Motor_Info_t *motor,
                                             float velocity_rad_s, float current_limit_a);

/* 参数接口：两套协议均支持，私有其他控制模式通过参数写入 ---------------- */

/**
 * @brief 读取一个手册参数；私有类型 17，MIT 标准 ID 的功能前缀为 3。
 * @note 返回值仅描述请求发送，参数值由 OnRx 写入 motor.parameter。
 *       MIT 参数读写要求固件 >= 0.5.0.13；未知索引返回 UNSUPPORTED。
 */
RS05_Motor_Status_e RS05_Motor_ReadParameter(FDCAN_HandleTypeDef *hcan,
                                           const RS05_Motor_Info_t *motor,
                                           uint16_t index);

/**
 * @brief 写入一个有类型参数；私有类型 18，MIT 标准 ID 的功能前缀为 4。
 * @note 核对索引、类型、写权限、协议能力和浮点有限性；不自动钳位参数值。
 *       业务量程由调用者遵守手册，运行模式统一使用 SetMode，禁止经本接口写 RUN_MODE。
 *       私有电流：CURRENT 模式下写 IQ_REF，单位 A。
 *       私有速度：SPEED 模式下写 SPD_REF，单位 rad/s；LIMIT_CUR、ACC_RAD 单独配置。
 *       私有 CSP：POSITION_CSP 下写 LOC_REF，单位 rad；LIMIT_SPD 单独配置。
 *       私有 PP：POSITION_PP 下写 LOC_REF；VEL_MAX、ACC_SET 单独配置。
 *       参数配置、使能、目标下发的顺序由所属任务按手册安排。
 */
RS05_Motor_Status_e RS05_Motor_WriteParameter(FDCAN_HandleTypeDef *hcan,
                                            const RS05_Motor_Info_t *motor,
                                            uint16_t index, const RS05_Motor_Value_t *value);

/* 接收接口：按实例协议直接解析和发布 ------------------------------------ */

/**
 * @brief 接收一次 HAL 帧，按 config.protocol 解码运动反馈、参数回复或原始故障。
 * @param data_length 本帧实际字节数，必须为 8，不是缓冲区容量。
 * @param now_ms 调用者提供的接收时间，单位 ms。
 * @note 校验配置、经典数据帧、长度和地址后更新相应分组，坏帧保持实例不变。
 *       私有接受类型 2/24 运动反馈；MIT 校验主机 ID 和负载中的电机 ID。
 *       只有运动反馈刷新 feedback 的时间/序号；参数和原始故障单独更新。
 *       不发起 MIT 诊断查询，不以零负载猜测与运动反馈无法可靠区分的查询回复。
 *       可在中断中调用；读取一致快照时由所属任务保护，不在本接口中发送或控制。
 */
RS05_Motor_Status_e RS05_Motor_OnRx(RS05_Motor_Info_t *motor,
                                  const FDCAN_RxHeaderTypeDef *header,
                                  const uint8_t *data, uint32_t data_length, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif /* RS05_MOTOR_H */
