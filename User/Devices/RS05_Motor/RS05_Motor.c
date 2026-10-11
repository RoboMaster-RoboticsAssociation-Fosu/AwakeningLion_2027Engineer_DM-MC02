/**
 * @file RS05_Motor.c
 * @brief RS05 私有/MIT 协议驱动：单帧控制发送与分组反馈更新。
 * @note 总线启动、周期、超时、重试、模式选择和使能顺序由所属任务负责。
 */
#include "RS05_Motor.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/* 协议 float 必须为 binary32；用 C99 编译期约束，不依赖 packed 或位域。 */
typedef char RS05_Motor_FloatFormatCheck[(CHAR_BIT == 8 && sizeof(float) == 4U &&
                                        FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
                                        FLT_MAX_EXP == 128) ? 1 : -1];

/* 私有量程与参数目录 ------------------------------------------------------ */

typedef enum
{
    RS05_SCALE_POSITION = 0,
    RS05_SCALE_VELOCITY,
    RS05_SCALE_KP,
    RS05_SCALE_KD,
    RS05_SCALE_TORQUE,
    RS05_SCALE_COUNT
} RS05_Motor_ScaleIndex_e;

typedef struct
{
    float minimum;
    float maximum;
    float encode_scale;
    float decode_scale;
    uint16_t maximum_code;
} RS05_Motor_Scale_t;

/* 固定比例由编译器计算；控制和反馈热点不重复执行除法。
 * 私有协议各字段为 16 位；MIT 位置为 16 位，其余运控字段为 12 位。
 */
static const RS05_Motor_Scale_t rs05_scales[2][RS05_SCALE_COUNT] = {
    {
        {-RS05_MOTOR_POSITION_MAX, RS05_MOTOR_POSITION_MAX, 65535.0f / (2.0f * RS05_MOTOR_POSITION_MAX), (2.0f * RS05_MOTOR_POSITION_MAX) / 65535.0f, 65535U},
        {-RS05_MOTOR_VELOCITY_MAX, RS05_MOTOR_VELOCITY_MAX, 65535.0f / (2.0f * RS05_MOTOR_VELOCITY_MAX), (2.0f * RS05_MOTOR_VELOCITY_MAX) / 65535.0f, 65535U},
        {0.0f, RS05_MOTOR_KP_MAX, 65535.0f / RS05_MOTOR_KP_MAX, RS05_MOTOR_KP_MAX / 65535.0f, 65535U},
        {0.0f, RS05_MOTOR_KD_MAX, 65535.0f / RS05_MOTOR_KD_MAX, RS05_MOTOR_KD_MAX / 65535.0f, 65535U},
        {-RS05_MOTOR_TORQUE_MAX, RS05_MOTOR_TORQUE_MAX, 65535.0f / (2.0f * RS05_MOTOR_TORQUE_MAX), (2.0f * RS05_MOTOR_TORQUE_MAX) / 65535.0f, 65535U}
    },
    {
        {-RS05_MOTOR_POSITION_MAX, RS05_MOTOR_POSITION_MAX, 65535.0f / (2.0f * RS05_MOTOR_POSITION_MAX), (2.0f * RS05_MOTOR_POSITION_MAX) / 65535.0f, 65535U},
        {-RS05_MOTOR_VELOCITY_MAX, RS05_MOTOR_VELOCITY_MAX, 4095.0f / (2.0f * RS05_MOTOR_VELOCITY_MAX), (2.0f * RS05_MOTOR_VELOCITY_MAX) / 4095.0f, 4095U},
        {0.0f, RS05_MOTOR_KP_MAX, 4095.0f / RS05_MOTOR_KP_MAX, RS05_MOTOR_KP_MAX / 4095.0f, 4095U},
        {0.0f, RS05_MOTOR_KD_MAX, 4095.0f / RS05_MOTOR_KD_MAX, RS05_MOTOR_KD_MAX / 4095.0f, 4095U},
        {-RS05_MOTOR_TORQUE_MAX, RS05_MOTOR_TORQUE_MAX, 4095.0f / (2.0f * RS05_MOTOR_TORQUE_MAX), (2.0f * RS05_MOTOR_TORQUE_MAX) / 4095.0f, 4095U}
    }
};

typedef struct
{
    uint16_t index;
    RS05_Motor_ValueType_e type;
    bool writable;
} RS05_Motor_ParameterDescription_t;

/* 手册第 37～38 页目录；未知索引拒绝，不推测其他型号的参数能力。 */
static const RS05_Motor_ParameterDescription_t rs05_parameters[] = {
    {RS05_MOTOR_PARAM_RUN_MODE, RS05_MOTOR_VALUE_U8, true},
    {RS05_MOTOR_PARAM_IQ_REF, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_SPD_REF, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_LIMIT_TORQUE, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_CUR_KP, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_CUR_KI, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_CUR_FILT_GAIN, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_LOC_REF, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_LIMIT_SPD, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_LIMIT_CUR, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_MECH_POS, RS05_MOTOR_VALUE_FLOAT, false},
    {RS05_MOTOR_PARAM_IQF, RS05_MOTOR_VALUE_FLOAT, false},
    {RS05_MOTOR_PARAM_MECH_VEL, RS05_MOTOR_VALUE_FLOAT, false},
    {RS05_MOTOR_PARAM_VBUS, RS05_MOTOR_VALUE_FLOAT, false},
    {RS05_MOTOR_PARAM_LOC_KP, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_SPD_KP, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_SPD_KI, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_SPD_FILT_GAIN, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_ACC_RAD, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_VEL_MAX, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_ACC_SET, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_REPORT_TIME, RS05_MOTOR_VALUE_U16, true},
    {RS05_MOTOR_PARAM_CAN_TIMEOUT, RS05_MOTOR_VALUE_U32, true},
    {RS05_MOTOR_PARAM_ZERO_STA, RS05_MOTOR_VALUE_U8, true},
    {RS05_MOTOR_PARAM_ADD_OFFSET, RS05_MOTOR_VALUE_FLOAT, true},
    {RS05_MOTOR_PARAM_ALVEOLOUS_OPEN, RS05_MOTOR_VALUE_U8, true},
    {RS05_MOTOR_PARAM_IQ_TEST, RS05_MOTOR_VALUE_U8, true},
    {RS05_MOTOR_PARAM_DCC_SET, RS05_MOTOR_VALUE_FLOAT, true}
};

/* 私有编解码 -------------------------------------------------------------- */

/** @brief 按已知目录查找参数类型和权限，未知索引返回 NULL。 */
static const RS05_Motor_ParameterDescription_t *rs05_find_parameter(uint16_t index)
{
    size_t i;
    for (i = 0U; i < sizeof(rs05_parameters) / sizeof(rs05_parameters[0]); ++i)
    {
        if (rs05_parameters[i].index == index)
        {
            return &rs05_parameters[i];
        }
    }
    return NULL;
}

/** @brief 小端整数编码；width 仅来自已知参数类型，取值为 2 或 4。 */
static void rs05_write_le(uint8_t *data, uint32_t value, uint8_t width)
{
    uint8_t i;
    for (i = 0U; i < width; ++i)
    {
        data[i] = (uint8_t)(value >> ((uint32_t)i * 8U));
    }
}

/** @brief 小端整数解码，不依赖主机端序、对齐或有效类型例外。 */
static uint32_t rs05_read_le(const uint8_t *data, uint8_t width)
{
    uint32_t value = 0U;
    uint8_t i;
    for (i = 0U; i < width; ++i)
    {
        value |= (uint32_t)data[i] << ((uint32_t)i * 8U);
    }
    return value;
}

/** @brief 大端 16 位运控整数编码。 */
static void rs05_write_be16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)value;
}

/** @brief 大端 16 位运控整数解码。 */
static uint16_t rs05_read_be16(const uint8_t *data)
{
    return (uint16_t)(((uint32_t)data[0] << 8U) | data[1]);
}

/** @brief 已校验有限的 float 写为小端 binary32，不把缓冲区强转为 float 指针。 */
static void rs05_write_float(uint8_t *data, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    rs05_write_le(data, bits, 4U);
}

/** @brief 小端 binary32 解码，调用者再检查参数回复的有限性。 */
static float rs05_read_float(const uint8_t *data)
{
    uint32_t bits = rs05_read_le(data, 4U);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

/** @brief 有限输入线性量化：越界钳位，内部截断，端点直接产生完整满量程。 */
static uint16_t rs05_quantize(float value, const RS05_Motor_Scale_t *scale, bool *clamped)
{
    float code;
    if (value <= scale->minimum)
    {
        *clamped = *clamped || value < scale->minimum;
        return 0U;
    }
    if (value >= scale->maximum)
    {
        *clamped = *clamped || value > scale->maximum;
        return scale->maximum_code;
    }
    code = (value - scale->minimum) * scale->encode_scale;
    /* 浮点舍入可能让内部值落到满量程，转整数前保证目标位宽有效。 */
    return code >= (float)scale->maximum_code ? scale->maximum_code : (uint16_t)code;
}

/** @brief 反量化物理值，编码端点精确还原，无热点除法。 */
static float rs05_dequantize(uint16_t value, const RS05_Motor_Scale_t *scale)
{
    if (value == 0U)
    {
        return scale->minimum;
    }
    if (value == scale->maximum_code)
    {
        return scale->maximum;
    }
    return scale->minimum + (float)value * scale->decode_scale;
}

/**
 * @brief 校验发送依赖并构造完整经典 CAN 头；函数不入队、不修改实例。
 * @note private_type 为通信类型，mit_type 为标准 ID 高三位；运控可再覆盖扭矩 ID。
 */
static RS05_Motor_Status_e rs05_prepare_tx(FDCAN_HandleTypeDef *hcan,
                                         const RS05_Motor_Info_t *motor,
                                         uint8_t private_type, uint8_t mit_type,
                                         FDCAN_TxHeaderTypeDef *header)
{
    RS05_Motor_Status_e status;
    FDCAN_TxHeaderTypeDef result = {0};
    if (hcan == NULL || motor == NULL)
    {
        return RS05_MOTOR_INVALID_ARGUMENT;
    }
    status = RS05_Motor_ValidateConfig(&motor->config);
    if (status != RS05_MOTOR_OK)
    {
        return status;
    }
    if (motor->config.protocol == RS05_MOTOR_PROTOCOL_PRIVATE)
    {
        result.Identifier = ((uint32_t)private_type << 24U) |
                            ((uint32_t)motor->config.host_id << 8U) | motor->config.motor_id;
        result.IdType = FDCAN_EXTENDED_ID;
    }
    else
    {
        result.Identifier = ((uint32_t)mit_type << 8U) | motor->config.motor_id;
        result.IdType = FDCAN_STANDARD_ID;
    }
    result.TxFrameType = FDCAN_DATA_FRAME;
    result.DataLength = FDCAN_DLC_BYTES_8;
    result.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    result.BitRateSwitch = FDCAN_BRS_OFF;
    result.FDFormat = FDCAN_CLASSIC_CAN;
    result.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    *header = result;
    return RS05_MOTOR_OK;
}

/** @brief 解码有类型参数回复，失败不修改输出；保留位必须为零。 */
static RS05_Motor_Status_e rs05_decode_parameter(const uint8_t *data, bool write_ack,
                                               RS05_Motor_ParameterReply_t *reply)
{
    RS05_Motor_ParameterReply_t result = {0};
    const RS05_Motor_ParameterDescription_t *description;
    if (data[2] != 0U || data[3] != 0U)
    {
        return RS05_MOTOR_PROTOCOL_ERROR;
    }
    result.index = (uint16_t)rs05_read_le(data, 2U);
    description = rs05_find_parameter(result.index);
    if (description == NULL || (write_ack && !description->writable))
    {
        return RS05_MOTOR_UNSUPPORTED;
    }
    result.write_ack = write_ack;
    result.value.type = description->type;
    switch (description->type)
    {
        case RS05_MOTOR_VALUE_U8: result.value.data.u8 = data[4]; break;
        case RS05_MOTOR_VALUE_U16: result.value.data.u16 = (uint16_t)rs05_read_le(&data[4], 2U); break;
        case RS05_MOTOR_VALUE_U32: result.value.data.u32 = rs05_read_le(&data[4], 4U); break;
        case RS05_MOTOR_VALUE_FLOAT:
            result.value.data.real = rs05_read_float(&data[4]);
            if (!isfinite(result.value.data.real))
            {
                return RS05_MOTOR_PROTOCOL_ERROR;
            }
            break;
        default: return RS05_MOTOR_PROTOCOL_ERROR;
    }
    *reply = result;
    return RS05_MOTOR_OK;
}

/* 公共管理接口 ------------------------------------------------------------ */

/** @brief 校验固定通信协议与地址，主机和电机地址相等时拒绝，避免 MIT 回复歧义。 */
RS05_Motor_Status_e RS05_Motor_ValidateConfig(const RS05_Motor_Config_t *config)
{
    if (config == NULL || config->motor_id == config->host_id)
    {
        return RS05_MOTOR_INVALID_ARGUMENT;
    }
    if (config->protocol != RS05_MOTOR_PROTOCOL_PRIVATE && config->protocol != RS05_MOTOR_PROTOCOL_MIT)
    {
        return RS05_MOTOR_UNSUPPORTED;
    }
    return RS05_MOTOR_OK;
}

/** @brief 私有类型 3/MIT FC 使能；仅入队一次，不把发送成功当硬件状态。 */
RS05_Motor_Status_e RS05_Motor_Enable(FDCAN_HandleTypeDef *hcan, const RS05_Motor_Info_t *motor)
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[RS05_MOTOR_FRAME_BYTES] = {0};
    RS05_Motor_Status_e status = rs05_prepare_tx(hcan, motor, 3U, 0U, &header);
    if (status != RS05_MOTOR_OK)
    {
        return status;
    }
    if (motor->config.protocol == RS05_MOTOR_PROTOCOL_MIT)
    {
        memset(data, 0xFF, sizeof(data));
        data[7] = 0xFCU;
    }
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, data) == HAL_OK ? RS05_MOTOR_OK : RS05_MOTOR_SEND_FAILED;
}

/** @brief 私有类型 4/MIT FD 失能，不附带清错或模式切换。 */
RS05_Motor_Status_e RS05_Motor_Disable(FDCAN_HandleTypeDef *hcan, const RS05_Motor_Info_t *motor)
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[RS05_MOTOR_FRAME_BYTES] = {0};
    RS05_Motor_Status_e status = rs05_prepare_tx(hcan, motor, 4U, 0U, &header);
    if (status != RS05_MOTOR_OK)
    {
        return status;
    }
    if (motor->config.protocol == RS05_MOTOR_PROTOCOL_MIT)
    {
        memset(data, 0xFF, sizeof(data));
        data[7] = 0xFDU;
    }
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, data) == HAL_OK ? RS05_MOTOR_OK : RS05_MOTOR_SEND_FAILED;
}

/** @brief 私有停止帧 Byte0=1/MIT FB 清错；协议差异不通过额外帧掩盖。 */
RS05_Motor_Status_e RS05_Motor_ClearFault(FDCAN_HandleTypeDef *hcan, const RS05_Motor_Info_t *motor)
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[RS05_MOTOR_FRAME_BYTES] = {1U};
    RS05_Motor_Status_e status = rs05_prepare_tx(hcan, motor, 4U, 0U, &header);
    if (status != RS05_MOTOR_OK)
    {
        return status;
    }
    if (motor->config.protocol == RS05_MOTOR_PROTOCOL_MIT)
    {
        memset(data, 0xFF, sizeof(data));
        data[7] = 0xFBU;
    }
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, data) == HAL_OK ? RS05_MOTOR_OK : RS05_MOTOR_SEND_FAILED;
}

/** @brief 私有类型 6 Byte0=1/MIT FE 标零；停止与模式前提由所属任务保证。 */
RS05_Motor_Status_e RS05_Motor_SetZero(FDCAN_HandleTypeDef *hcan, const RS05_Motor_Info_t *motor)
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[RS05_MOTOR_FRAME_BYTES] = {1U};
    RS05_Motor_Status_e status = rs05_prepare_tx(hcan, motor, 6U, 0U, &header);
    if (status != RS05_MOTOR_OK)
    {
        return status;
    }
    if (motor->config.protocol == RS05_MOTOR_PROTOCOL_MIT)
    {
        memset(data, 0xFF, sizeof(data));
        data[7] = 0xFEU;
    }
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, data) == HAL_OK ? RS05_MOTOR_OK : RS05_MOTOR_SEND_FAILED;
}

/** @brief 单帧设置运行模式，MIT 的领域 CSP 映射为线协议值 1，不自动改变使能。 */
RS05_Motor_Status_e RS05_Motor_SetMode(FDCAN_HandleTypeDef *hcan, const RS05_Motor_Info_t *motor,
                                    RS05_Motor_Mode_e mode)
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[RS05_MOTOR_FRAME_BYTES] = {0};
    uint8_t wire_mode;
    RS05_Motor_Status_e status = rs05_prepare_tx(hcan, motor, 18U, 0U, &header);
    if (status != RS05_MOTOR_OK)
    {
        return status;
    }
    switch (mode)
    {
        case RS05_MOTOR_MODE_MOTION: wire_mode = 0U; break;
        case RS05_MOTOR_MODE_SPEED: wire_mode = 2U; break;
        case RS05_MOTOR_MODE_POSITION_CSP:
            wire_mode = motor->config.protocol == RS05_MOTOR_PROTOCOL_MIT ? 1U : 5U;
            break;
        case RS05_MOTOR_MODE_POSITION_PP:
        case RS05_MOTOR_MODE_CURRENT:
            if (motor->config.protocol == RS05_MOTOR_PROTOCOL_MIT)
            {
                return RS05_MOTOR_UNSUPPORTED;
            }
            wire_mode = (uint8_t)mode;
            break;
        default: return RS05_MOTOR_INVALID_ARGUMENT;
    }
    if (motor->config.protocol == RS05_MOTOR_PROTOCOL_PRIVATE)
    {
        rs05_write_le(data, RS05_MOTOR_PARAM_RUN_MODE, 2U);
        data[4] = wire_mode;
    }
    else
    {
        memset(data, 0xFF, sizeof(data));
        data[6] = wire_mode;
        data[7] = 0xFCU;
    }
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, data) == HAL_OK ? RS05_MOTOR_OK : RS05_MOTOR_SEND_FAILED;
}

/** @brief 私有类型 24/MIT F9 上报开关；不额外设置上报周期或保存参数。 */
RS05_Motor_Status_e RS05_Motor_SetActiveReport(FDCAN_HandleTypeDef *hcan,
                                            const RS05_Motor_Info_t *motor, bool enabled)
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[RS05_MOTOR_FRAME_BYTES] = {1U, 2U, 3U, 4U, 5U, 6U, 0U, 0U};
    RS05_Motor_Status_e status = rs05_prepare_tx(hcan, motor, 24U, 0U, &header);
    if (status != RS05_MOTOR_OK)
    {
        return status;
    }
    if (motor->config.protocol == RS05_MOTOR_PROTOCOL_MIT)
    {
        memset(data, 0xFF, sizeof(data));
        data[7] = 0xF9U;
    }
    data[6] = enabled ? 1U : 0U;
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, data) == HAL_OK ? RS05_MOTOR_OK : RS05_MOTOR_SEND_FAILED;
}

/* 控制与参数发送 ---------------------------------------------------------- */

/** @brief 双协议五参数运控；私有力矩位于 ID，MIT 力矩位于负载的低 12 位。 */
RS05_Motor_Status_e RS05_Motor_ControlMotion(FDCAN_HandleTypeDef *hcan, const RS05_Motor_Info_t *motor,
                                          float position_rad, float velocity_rad_s,
                                          float kp, float kd, float torque_nm)
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[RS05_MOTOR_FRAME_BYTES] = {0};
    const RS05_Motor_Scale_t *scales;
    uint16_t p, v, kp_code, kd_code, torque;
    bool clamped = false;
    RS05_Motor_Status_e status = rs05_prepare_tx(hcan, motor, 1U, 0U, &header);
    if (status != RS05_MOTOR_OK)
    {
        return status;
    }
    if (!isfinite(position_rad) || !isfinite(velocity_rad_s) || !isfinite(kp) || !isfinite(kd) || !isfinite(torque_nm))
    {
        return RS05_MOTOR_INVALID_ARGUMENT;
    }
    scales = rs05_scales[motor->config.protocol == RS05_MOTOR_PROTOCOL_PRIVATE ? 0 : 1];
    p = rs05_quantize(position_rad, &scales[RS05_SCALE_POSITION], &clamped);
    v = rs05_quantize(velocity_rad_s, &scales[RS05_SCALE_VELOCITY], &clamped);
    kp_code = rs05_quantize(kp, &scales[RS05_SCALE_KP], &clamped);
    kd_code = rs05_quantize(kd, &scales[RS05_SCALE_KD], &clamped);
    torque = rs05_quantize(torque_nm, &scales[RS05_SCALE_TORQUE], &clamped);
    rs05_write_be16(data, p);
    if (motor->config.protocol == RS05_MOTOR_PROTOCOL_PRIVATE)
    {
        header.Identifier = (1UL << 24U) | ((uint32_t)torque << 8U) | motor->config.motor_id;
        rs05_write_be16(&data[2], v);
        rs05_write_be16(&data[4], kp_code);
        rs05_write_be16(&data[6], kd_code);
    }
    else
    {
        data[2] = (uint8_t)(v >> 4U);
        data[3] = (uint8_t)(((uint32_t)(v & 0x0FU) << 4U) | (kp_code >> 8U));
        data[4] = (uint8_t)kp_code;
        data[5] = (uint8_t)(kd_code >> 4U);
        data[6] = (uint8_t)(((uint32_t)(kd_code & 0x0FU) << 4U) | (torque >> 8U));
        data[7] = (uint8_t)torque;
        /* 只拒绝与手册管理/查询命令完全重合的负载，包括本模块未提供的改 ID/协议/保存命令。
         * FA/FB/01 的 Byte6 可取任意值；其余命令按各自允许的 Byte6 判断。
         */
        if (data[0] == 0xFFU && data[1] == 0xFFU && data[2] == 0xFFU &&
            data[3] == 0xFFU && data[4] == 0xFFU && data[5] == 0xFFU &&
            (data[7] == 0xFAU || data[7] == 0xFBU || data[7] == 0x01U ||
             ((data[7] == 0xFCU || data[7] == 0xFDU) && (data[6] <= 2U || data[6] == 0xFFU)) ||
             ((data[7] == 0xF8U || data[7] == 0xFEU) && data[6] == 0xFFU) ||
             (data[7] == 0xF9U && data[6] <= 1U)))
        {
            return RS05_MOTOR_PROTOCOL_ERROR;
        }
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, data) != HAL_OK)
    {
        return RS05_MOTOR_SEND_FAILED;
    }
    return clamped ? RS05_MOTOR_CLAMPED : RS05_MOTOR_OK;
}

/** @brief MIT 标准 ID 前缀 1 的浮点位置/速度限制帧，保留浮点模式原始数值语义。 */
RS05_Motor_Status_e RS05_Motor_MitControlPosition(FDCAN_HandleTypeDef *hcan, const RS05_Motor_Info_t *motor,
                                               float position_rad, float speed_limit_rad_s)
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[RS05_MOTOR_FRAME_BYTES] = {0};
    RS05_Motor_Status_e status = rs05_prepare_tx(hcan, motor, 0U, 1U, &header);
    if (status != RS05_MOTOR_OK)
    {
        return status;
    }
    if (motor->config.protocol != RS05_MOTOR_PROTOCOL_MIT)
    {
        return RS05_MOTOR_UNSUPPORTED;
    }
    if (!isfinite(position_rad) || !isfinite(speed_limit_rad_s) || speed_limit_rad_s < 0.0f)
    {
        return RS05_MOTOR_INVALID_ARGUMENT;
    }
    rs05_write_float(data, position_rad);
    rs05_write_float(&data[4], speed_limit_rad_s);
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, data) == HAL_OK ? RS05_MOTOR_OK : RS05_MOTOR_SEND_FAILED;
}

/** @brief MIT 标准 ID 前缀 2 的浮点速度/电流限制帧，不套用运控量程。 */
RS05_Motor_Status_e RS05_Motor_MitControlSpeed(FDCAN_HandleTypeDef *hcan, const RS05_Motor_Info_t *motor,
                                            float velocity_rad_s, float current_limit_a)
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[RS05_MOTOR_FRAME_BYTES] = {0};
    RS05_Motor_Status_e status = rs05_prepare_tx(hcan, motor, 0U, 2U, &header);
    if (status != RS05_MOTOR_OK)
    {
        return status;
    }
    if (motor->config.protocol != RS05_MOTOR_PROTOCOL_MIT)
    {
        return RS05_MOTOR_UNSUPPORTED;
    }
    if (!isfinite(velocity_rad_s) || !isfinite(current_limit_a) || current_limit_a < 0.0f)
    {
        return RS05_MOTOR_INVALID_ARGUMENT;
    }
    rs05_write_float(data, velocity_rad_s);
    rs05_write_float(&data[4], current_limit_a);
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, data) == HAL_OK ? RS05_MOTOR_OK : RS05_MOTOR_SEND_FAILED;
}

/** @brief 单参数读请求；负载保留位清零，不等待回复，结果由接收入口更新。 */
RS05_Motor_Status_e RS05_Motor_ReadParameter(FDCAN_HandleTypeDef *hcan, const RS05_Motor_Info_t *motor,
                                          uint16_t index)
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[RS05_MOTOR_FRAME_BYTES] = {0};
    RS05_Motor_Status_e status = rs05_prepare_tx(hcan, motor, 17U, 3U, &header);
    if (status != RS05_MOTOR_OK)
    {
        return status;
    }
    if (rs05_find_parameter(index) == NULL)
    {
        return RS05_MOTOR_UNSUPPORTED;
    }
    rs05_write_le(data, index, 2U);
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, data) == HAL_OK ? RS05_MOTOR_OK : RS05_MOTOR_SEND_FAILED;
}

/** @brief 单参数写入，参数类型与权限先校验；只发送请求，不缓存目标或保存配置。 */
RS05_Motor_Status_e RS05_Motor_WriteParameter(FDCAN_HandleTypeDef *hcan, const RS05_Motor_Info_t *motor,
                                           uint16_t index, const RS05_Motor_Value_t *value)
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[RS05_MOTOR_FRAME_BYTES] = {0};
    const RS05_Motor_ParameterDescription_t *description;
    RS05_Motor_Status_e status = rs05_prepare_tx(hcan, motor, 18U, 4U, &header);
    if (status != RS05_MOTOR_OK)
    {
        return status;
    }
    if (value == NULL)
    {
        return RS05_MOTOR_INVALID_ARGUMENT;
    }
    description = rs05_find_parameter(index);
    if (description == NULL || !description->writable || index == RS05_MOTOR_PARAM_RUN_MODE)
    {
        return RS05_MOTOR_UNSUPPORTED;
    }
    if (description->type != value->type)
    {
        return RS05_MOTOR_INVALID_ARGUMENT;
    }
    rs05_write_le(data, index, 2U);
    switch (value->type)
    {
        case RS05_MOTOR_VALUE_U8: data[4] = value->data.u8; break;
        case RS05_MOTOR_VALUE_U16: rs05_write_le(&data[4], value->data.u16, 2U); break;
        case RS05_MOTOR_VALUE_U32: rs05_write_le(&data[4], value->data.u32, 4U); break;
        case RS05_MOTOR_VALUE_FLOAT:
            if (!isfinite(value->data.real))
            {
                return RS05_MOTOR_INVALID_ARGUMENT;
            }
            rs05_write_float(&data[4], value->data.real);
            break;
        default: return RS05_MOTOR_INVALID_ARGUMENT;
    }
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, data) == HAL_OK ? RS05_MOTOR_OK : RS05_MOTOR_SEND_FAILED;
}

/* 接收与分组发布 ---------------------------------------------------------- */

/**
 * @brief 校验完整 HAL 帧后在局部解码，再发布对应状态；失败不修改电机实例。
 * @note MIT 诊断/设备发现不在本驱动范围；不使用零负载启发式识别反馈。
 */
RS05_Motor_Status_e RS05_Motor_OnRx(RS05_Motor_Info_t *motor, const FDCAN_RxHeaderTypeDef *header,
                                 const uint8_t *data, uint32_t data_length, uint32_t now_ms)
{
    RS05_Motor_Status_e status;
    RS05_Motor_Feedback_t feedback = {0};
    RS05_Motor_ParameterReply_t parameter = {0};
    const RS05_Motor_Scale_t *scales;
    uint32_t function;
    bool parameter_reply = false;
    bool write_ack = false;
    if (motor == NULL || header == NULL || data == NULL)
    {
        return RS05_MOTOR_INVALID_ARGUMENT;
    }
    status = RS05_Motor_ValidateConfig(&motor->config);
    if (status != RS05_MOTOR_OK)
    {
        return status;
    }
    if (data_length != RS05_MOTOR_FRAME_BYTES || header->DataLength != FDCAN_DLC_BYTES_8 ||
        header->RxFrameType != FDCAN_DATA_FRAME || header->FDFormat != FDCAN_CLASSIC_CAN ||
        header->BitRateSwitch != FDCAN_BRS_OFF ||
        (header->IdType != FDCAN_STANDARD_ID && header->IdType != FDCAN_EXTENDED_ID))
    {
        return RS05_MOTOR_PROTOCOL_ERROR;
    }
    if (header->Identifier > (header->IdType == FDCAN_EXTENDED_ID ? 0x1FFFFFFFUL : 0x7FFUL))
    {
        return RS05_MOTOR_PROTOCOL_ERROR;
    }
    if ((header->IdType == FDCAN_EXTENDED_ID) != (motor->config.protocol == RS05_MOTOR_PROTOCOL_PRIVATE))
    {
        return RS05_MOTOR_NOT_FOR_DEVICE;
    }
    if (motor->config.protocol == RS05_MOTOR_PROTOCOL_PRIVATE)
    {
        if ((header->Identifier & 0xFFU) != motor->config.host_id ||
            ((header->Identifier >> 8U) & 0xFFU) != motor->config.motor_id)
        {
            return RS05_MOTOR_NOT_FOR_DEVICE;
        }
        function = header->Identifier >> 24U;
        if (function == 17U)
        {
            uint32_t remote_status = (header->Identifier >> 16U) & 0xFFU;
            if (remote_status != 0U)
            {
                return remote_status == 1U ? RS05_MOTOR_REMOTE_ERROR : RS05_MOTOR_PROTOCOL_ERROR;
            }
            parameter_reply = true;
        }
        else if (function == 21U)
        {
            RS05_Motor_FaultState_t fault = {.raw = {0}};
            /* 类型 21 多字节端序未由手册明确，保留原始负载，不伪造详细故障值。 */
            memcpy(fault.raw, data, sizeof(fault.raw));
            fault.valid = true;
            fault.last_update_ms = now_ms;
            motor->fault = fault;
            return RS05_MOTOR_OK;
        }
        else if (function == 2U || function == 24U)
        {
            scales = rs05_scales[0];
            feedback.state = (uint8_t)((header->Identifier >> 22U) & 3U);
            feedback.fault_bits = (uint8_t)((header->Identifier >> 16U) & 0x3FU);
            feedback.fault = feedback.fault_bits != 0U;
            feedback.position_rad = rs05_dequantize(rs05_read_be16(data), &scales[RS05_SCALE_POSITION]);
            feedback.velocity_rad_s = rs05_dequantize(rs05_read_be16(&data[2]), &scales[RS05_SCALE_VELOCITY]);
            feedback.torque_nm = rs05_dequantize(rs05_read_be16(&data[4]), &scales[RS05_SCALE_TORQUE]);
            feedback.temperature_c = (float)rs05_read_be16(&data[6]) * 0.1f;
        }
        else
        {
            return RS05_MOTOR_UNSUPPORTED;
        }
    }
    else
    {
        function = header->Identifier >> 8U;
        if ((function == 3U || function == 4U) && (header->Identifier & 0xFFU) == motor->config.motor_id)
        {
            parameter_reply = true;
            write_ack = function == 4U;
        }
        else if (header->Identifier == motor->config.host_id && data[0] == motor->config.motor_id)
        {
            scales = rs05_scales[1];
            /* Byte6 高四位为状态/故障/预警，低四位才是温度高位。 */
            feedback.state = (uint8_t)(data[6] >> 6U);
            feedback.fault = (data[6] & 0x20U) != 0U;
            feedback.warning = (data[6] & 0x10U) != 0U;
            feedback.position_rad = rs05_dequantize(rs05_read_be16(&data[1]), &scales[RS05_SCALE_POSITION]);
            feedback.velocity_rad_s = rs05_dequantize((uint16_t)(((uint32_t)data[3] << 4U) | (data[4] >> 4U)), &scales[RS05_SCALE_VELOCITY]);
            feedback.torque_nm = rs05_dequantize((uint16_t)(((uint32_t)(data[4] & 0x0FU) << 8U) | data[5]), &scales[RS05_SCALE_TORQUE]);
            feedback.temperature_c = (float)(((uint32_t)(data[6] & 0x0FU) << 8U) | data[7]) * 0.1f;
        }
        else
        {
            return RS05_MOTOR_NOT_FOR_DEVICE;
        }
    }
    if (parameter_reply)
    {
        status = rs05_decode_parameter(data, write_ack, &parameter);
        if (status != RS05_MOTOR_OK)
        {
            return status;
        }
        motor->parameter.data = parameter;
        motor->parameter.last_update_ms = now_ms;
        motor->parameter.valid = true;
        motor->parameter.sequence++;
    }
    else
    {
        if (feedback.state == 3U)
        {
            return RS05_MOTOR_PROTOCOL_ERROR;
        }
        motor->feedback.data = feedback;
        motor->feedback.last_update_ms = now_ms;
        motor->feedback.valid = true;
        motor->feedback.sequence++;
    }
    return RS05_MOTOR_OK;
}
