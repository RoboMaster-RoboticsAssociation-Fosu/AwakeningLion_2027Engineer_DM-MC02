/**
 * @file QD4310_Motor.c
 * @brief QD4310 小端整数 CAN 协议实现；纯编解码与一次 HAL 提交分开。
 */
#include "QD4310_Motor.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

/* C99 编译期约束；线协议采用 8 位字节和 16 位整数，不依赖结构体布局。 */
typedef char QD4310_Motor_WireWidthCheck[
    (CHAR_BIT == 8 && UINT16_MAX == 65535U && INT16_MAX == 32767) ? 1 : -1];

/* 私有协议比例 ------------------------------------------------------------ */
#define QD4310_CONTROL_ID_BASE 0x400U
#define QD4310_FEEDBACK_ID_BASE 0x500U
#define QD4310_SIGNED_CODE_MAX 32767
#define QD4310_UNSIGNED_CODE_MAX 65535U

typedef struct
{
    float minimum;
    float maximum;
    float encode_scale;
} QD4310_Motor_Scale_t;

/* 比例在编译期计算，接收和周期编码只做乘法。低速复用速度比例。 */
static const QD4310_Motor_Scale_t qd4310_current_scale = {
    -QD4310_MOTOR_CURRENT_MAX_A, QD4310_MOTOR_CURRENT_MAX_A,
    32767.0f / QD4310_MOTOR_CURRENT_MAX_A
};
static const QD4310_Motor_Scale_t qd4310_speed_scale = {
    -QD4310_MOTOR_VELOCITY_MAX_RAD_S, QD4310_MOTOR_VELOCITY_MAX_RAD_S,
    32767.0f / QD4310_MOTOR_VELOCITY_MAX_RAD_S
};
static const QD4310_Motor_Scale_t qd4310_position_scale = {
    0.0f, QD4310_MOTOR_TURN_RAD, 65535.0f / QD4310_MOTOR_TURN_RAD
};
static const QD4310_Motor_Scale_t qd4310_step_scale = {
    -QD4310_MOTOR_TURN_RAD, QD4310_MOTOR_TURN_RAD,
    32767.0f / QD4310_MOTOR_TURN_RAD
};

/** @brief 小端 16 位读取；输入已由接收入口验证长度。 */
static uint16_t qd4310_read_le16(const uint8_t *data)
{
    return (uint16_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8U));
}

/** @brief 在 int32_t 中还原二补码，避免超范围 uint16_t 到 int16_t 转换。 */
static int32_t qd4310_read_signed_le16(const uint8_t *data)
{
    uint16_t raw = qd4310_read_le16(data);
    return raw <= (uint16_t)INT16_MAX ? (int32_t)raw : (int32_t)raw - 65536;
}

/** @brief 纯反馈解码：调用者已校验 8 字节长度，保留未定义的状态和预留字节。 */
static QD4310_Motor_Feedback_t qd4310_decode_feedback(const uint8_t *data)
{
    QD4310_Motor_Feedback_t feedback = {0};
    /* Byte2-3 电流、Byte4-5 速度有符号；Byte6-7 单圈角度无符号。 */
    feedback.current_a = (float)qd4310_read_signed_le16(&data[2]) *
        (QD4310_MOTOR_CURRENT_MAX_A / 32767.0f);
    feedback.velocity_rad_s = (float)qd4310_read_signed_le16(&data[4]) *
        (QD4310_MOTOR_VELOCITY_MAX_RAD_S / 32767.0f);
    feedback.position_rad = (float)qd4310_read_le16(&data[6]) *
        (QD4310_MOTOR_TURN_RAD / 65535.0f);
    feedback.enabled = (data[0] & 0x01U) != 0U;
    feedback.raw_status = data[0];
    feedback.reserved = data[1];
    return feedback;
}

/**
 * @brief 纯控制编码：校验模式和有限性、限幅、量化，再一次提交完整 3 字节负载。
 * @note 失败保持 data/applied 不变；只有发送层成功后才对外发布 applied。
 */
static QD4310_Motor_Status_e qd4310_encode_control(QD4310_Motor_Mode_e mode,
                                                float value, uint8_t *data,
                                                float *applied)
{
    const QD4310_Motor_Scale_t *scale;
    float limited;
    int32_t code;
    uint16_t wire;

    switch (mode)
    {
        case QD4310_MOTOR_MODE_CURRENT: scale = &qd4310_current_scale; break;
        case QD4310_MOTOR_MODE_SPEED:
        case QD4310_MOTOR_MODE_LOW_SPEED: scale = &qd4310_speed_scale; break;
        case QD4310_MOTOR_MODE_POSITION: scale = &qd4310_position_scale; break;
        case QD4310_MOTOR_MODE_POSITION_STEP: scale = &qd4310_step_scale; break;
        default: return QD4310_MOTOR_INVALID_ARGUMENT;
    }
    if (!isfinite(value))
    {
        return QD4310_MOTOR_INVALID_ARGUMENT;
    }

    /* 端点直接给完整码值，避免浮点比例误差丢掉最后一个量化步。 */
    if (value <= scale->minimum)
    {
        limited = scale->minimum;
        code = scale->minimum < 0.0f ? -QD4310_SIGNED_CODE_MAX : 0;
    }
    else if (value >= scale->maximum)
    {
        limited = scale->maximum;
        code = mode == QD4310_MOTOR_MODE_POSITION ?
            (int32_t)QD4310_UNSIGNED_CODE_MAX : QD4310_SIGNED_CODE_MAX;
    }
    else
    {
        limited = value;
        code = (int32_t)(value * scale->encode_scale);
    }
    /* 有符号转无符号按 C 规则取模；不右移负数，不依赖有符号移位行为。 */
    wire = (uint16_t)code;
    data[0] = (uint8_t)mode;
    data[1] = (uint8_t)(wire & 0xFFU);
    data[2] = (uint8_t)(wire >> 8U);
    *applied = limited;
    return limited == value ? QD4310_MOTOR_OK : QD4310_MOTOR_CLAMPED;
}

/** @brief 一次 HAL 发送适配：校验固定 ID、构造完整经典标准帧并报告入队结果。 */
static QD4310_Motor_Status_e qd4310_send(FDCAN_HandleTypeDef *hcan,
                                      const QD4310_Motor_Info_t *motor,
                                      const uint8_t *data)
{
    FDCAN_TxHeaderTypeDef header = {0};
    if (hcan == NULL || motor == NULL || motor->config.motor_id > QD4310_MOTOR_ID_MAX)
    {
        return QD4310_MOTOR_INVALID_ARGUMENT;
    }
    header.Identifier = QD4310_CONTROL_ID_BASE + motor->config.motor_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_3;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, data) == HAL_OK ?
        QD4310_MOTOR_OK : QD4310_MOTOR_SEND_FAILED;
}

/* 公共接口 ---------------------------------------------------------------- */
QD4310_Motor_Status_e QD4310_Motor_SendCommand(FDCAN_HandleTypeDef *hcan,
                                            const QD4310_Motor_Info_t *motor,
                                            QD4310_Motor_Command_e command)
{
    uint8_t data[QD4310_MOTOR_CONTROL_BYTES] = {0U};
    switch (command)
    {
        case QD4310_MOTOR_COMMAND_QUERY:
        case QD4310_MOTOR_COMMAND_ENABLE:
        case QD4310_MOTOR_COMMAND_DISABLE:
            data[0] = (uint8_t)command;
            break;
        default:
            return QD4310_MOTOR_INVALID_ARGUMENT;
    }
    return qd4310_send(hcan, motor, data);
}

QD4310_Motor_Status_e QD4310_Motor_Control(FDCAN_HandleTypeDef *hcan,
                                        const QD4310_Motor_Info_t *motor,
                                        QD4310_Motor_Mode_e mode,
                                        float value, float *applied)
{
    uint8_t data[QD4310_MOTOR_CONTROL_BYTES];
    float limited;
    QD4310_Motor_Status_e status = qd4310_encode_control(mode, value, data, &limited);
    QD4310_Motor_Status_e send_status;
    if (status != QD4310_MOTOR_OK && status != QD4310_MOTOR_CLAMPED)
    {
        return status;
    }
    send_status = qd4310_send(hcan, motor, data);
    if (send_status != QD4310_MOTOR_OK)
    {
        return send_status;
    }
    /* 事务提交点：入队失败不能覆盖调用者上一次成功提交的目标记录。 */
    if (applied != NULL)
    {
        *applied = limited;
    }
    return status;
}

/** @brief 先在力矩单位中处理有限性和量程，再复用电流发送的事务提交边界。 */
QD4310_Motor_Status_e QD4310_Motor_ControlTorque(FDCAN_HandleTypeDef *hcan,
                                              const QD4310_Motor_Info_t *motor,
                                              float torque_nm,
                                              float *applied_torque_nm)
{
    float limited_torque_nm;
    float current_a;
    QD4310_Motor_Status_e status;

    if (!isfinite(torque_nm))
    {
        return QD4310_MOTOR_INVALID_ARGUMENT;
    }
    /* 先限幅再除以转矩常数，避免有限大力矩在换算时溢出为 Inf。
     * 端点直接给 ±10 A，避免浮点换算丢失电流协议的满量程编码。
     */
    if (torque_nm >= QD4310_MOTOR_TORQUE_MAX_NM)
    {
        limited_torque_nm = QD4310_MOTOR_TORQUE_MAX_NM;
        current_a = QD4310_MOTOR_CURRENT_MAX_A;
    }
    else if (torque_nm <= -QD4310_MOTOR_TORQUE_MAX_NM)
    {
        limited_torque_nm = -QD4310_MOTOR_TORQUE_MAX_NM;
        current_a = -QD4310_MOTOR_CURRENT_MAX_A;
    }
    else
    {
        limited_torque_nm = torque_nm;
        current_a = QD4310_MOTOR_TORQUE_TO_CURRENT_A(torque_nm);
    }

    status = QD4310_Motor_Control(hcan, motor, QD4310_MOTOR_MODE_CURRENT,
                                current_a, NULL);
    if (status != QD4310_MOTOR_OK && status != QD4310_MOTOR_CLAMPED)
    {
        return status;
    }
    /* 只有 CAN 已接受本次指令，才更新调用者保存的目标力矩。 */
    if (applied_torque_nm != NULL)
    {
        *applied_torque_nm = limited_torque_nm;
    }
    return limited_torque_nm != torque_nm ? QD4310_MOTOR_CLAMPED : status;
}

QD4310_Motor_Status_e QD4310_Motor_OnRx(QD4310_Motor_Info_t *motor,
                                     const FDCAN_RxHeaderTypeDef *header,
                                     const uint8_t *data, uint32_t data_length,
                                     uint32_t now_ms)
{
    QD4310_Motor_FeedbackState_t next;
    if (motor == NULL || header == NULL || data == NULL ||
        motor->config.motor_id > QD4310_MOTOR_ID_MAX)
    {
        return QD4310_MOTOR_INVALID_ARGUMENT;
    }
    if (data_length != QD4310_MOTOR_FEEDBACK_BYTES ||
        header->DataLength != FDCAN_DLC_BYTES_8 ||
        header->IdType != FDCAN_STANDARD_ID || header->RxFrameType != FDCAN_DATA_FRAME ||
        header->FDFormat != FDCAN_CLASSIC_CAN ||
        header->Identifier != QD4310_FEEDBACK_ID_BASE + motor->config.motor_id)
    {
        return QD4310_MOTOR_BAD_FRAME;
    }

    next.data = qd4310_decode_feedback(data);
    next.valid = true;
    next.last_update_ms = now_ms;
    next.sequence = motor->feedback.sequence + 1U;
    /* 完整校验后发布；结构体赋值不是并发同步，任务读取仍须保护一致性。 */
    motor->feedback = next;
    return QD4310_MOTOR_OK;
}
