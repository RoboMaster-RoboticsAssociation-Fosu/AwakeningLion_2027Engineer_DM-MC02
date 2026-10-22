/**
 * @file test_qd4310.c
 * @brief 主机协议回归：编译真实驱动和真实 HAL 头，仅用捕获器替代硬件发送。
 * @note 测试发送替身仅用于本文件，不进入固件；验证入队报文和失败分支，不能证明实机通信。
 */
#include "QD4310_Motor.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static FDCAN_HandleTypeDef test_bus;
static FDCAN_HandleTypeDef *captured_bus;
static FDCAN_TxHeaderTypeDef captured_header;
static uint8_t captured_data[QD4310_MOTOR_CONTROL_BYTES];
static HAL_StatusTypeDef transport_status = HAL_OK;
static unsigned int send_count;
static unsigned int checks;

/* 测试替身开始：捕获真实驱动的一次 HAL 调用，可注入硬件入队失败。
 * 范围仅主机测试；实机验证必须链接 STM32 HAL 的真实实现，此替身不可进入固件。
 */
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
                                              const FDCAN_TxHeaderTypeDef *header,
                                              const uint8_t *data)
{
    captured_bus = hfdcan;
    captured_header = *header;
    memcpy(captured_data, data, sizeof(captured_data));
    ++send_count;
    return transport_status;
}
/* 测试替身结束。 */

/** @brief 验证完整 HAL 帧属性及手工给定的三个线协议字节。 */
static void check_frame(uint8_t motor_id, uint8_t command, uint8_t low, uint8_t high)
{
    assert(captured_bus == &test_bus);
    assert(captured_header.Identifier == 0x400U + motor_id);
    assert(captured_header.IdType == FDCAN_STANDARD_ID);
    assert(captured_header.TxFrameType == FDCAN_DATA_FRAME);
    assert(captured_header.DataLength == FDCAN_DLC_BYTES_3);
    assert(captured_header.FDFormat == FDCAN_CLASSIC_CAN);
    assert(captured_header.BitRateSwitch == FDCAN_BRS_OFF);
    assert(captured_header.ErrorStateIndicator == FDCAN_ESI_ACTIVE);
    assert(captured_header.TxEventFifoControl == FDCAN_NO_TX_EVENTS);
    assert(captured_header.MessageMarker == 0U);
    assert(captured_data[0] == command && captured_data[1] == low && captured_data[2] == high);
    ++checks;
}

/** @brief 按字段比较实例，避免依赖结构体填充字节。 */
static void check_unchanged(const QD4310_Motor_Info_t *motor, const QD4310_Motor_Info_t *old)
{
    assert(motor->config.motor_id == old->config.motor_id);
    assert(motor->feedback.valid == old->feedback.valid);
    assert(motor->feedback.last_update_ms == old->feedback.last_update_ms);
    assert(motor->feedback.sequence == old->feedback.sequence);
    assert(motor->feedback.data.current_a == old->feedback.data.current_a);
    assert(motor->feedback.data.velocity_rad_s == old->feedback.data.velocity_rad_s);
    assert(motor->feedback.data.position_rad == old->feedback.data.position_rad);
    assert(motor->feedback.data.enabled == old->feedback.data.enabled);
    assert(motor->feedback.data.raw_status == old->feedback.data.raw_status);
    assert(motor->feedback.data.reserved == old->feedback.data.reserved);
    ++checks;
}

/** @brief 覆盖五种模式、手工黄金向量、端点、限幅及每种基础管理命令。 */
static void test_vectors(void)
{
    typedef struct
    {
        QD4310_Motor_Mode_e mode;
        float value;
        uint8_t low;
        uint8_t high;
    } Vector_t;
    static const Vector_t vectors[] = {
        {QD4310_MOTOR_MODE_CURRENT, 0.0f, 0x00U, 0x00U},
        {QD4310_MOTOR_MODE_CURRENT, 0.75f, 0x99U, 0x09U},
        {QD4310_MOTOR_MODE_CURRENT, -0.75f, 0x67U, 0xF6U},
        {QD4310_MOTOR_MODE_CURRENT, 10.0f, 0xFFU, 0x7FU},
        {QD4310_MOTOR_MODE_CURRENT, -10.0f, 0x01U, 0x80U},
        {QD4310_MOTOR_MODE_SPEED, 0.0f, 0x00U, 0x00U},
        {QD4310_MOTOR_MODE_SPEED, QD4310_MOTOR_VELOCITY_MAX_RAD_S / 4.0f, 0xFFU, 0x1FU},
        {QD4310_MOTOR_MODE_SPEED, -QD4310_MOTOR_VELOCITY_MAX_RAD_S / 4.0f, 0x01U, 0xE0U},
        {QD4310_MOTOR_MODE_SPEED, QD4310_MOTOR_VELOCITY_MAX_RAD_S, 0xFFU, 0x7FU},
        {QD4310_MOTOR_MODE_SPEED, -QD4310_MOTOR_VELOCITY_MAX_RAD_S, 0x01U, 0x80U},
        {QD4310_MOTOR_MODE_POSITION, 0.0f, 0x00U, 0x00U},
        {QD4310_MOTOR_MODE_POSITION, 0.5f, 0x5FU, 0x14U},
        {QD4310_MOTOR_MODE_POSITION, QD4310_MOTOR_TURN_RAD / 2.0f, 0xFFU, 0x7FU},
        {QD4310_MOTOR_MODE_POSITION, QD4310_MOTOR_TURN_RAD, 0xFFU, 0xFFU},
        {QD4310_MOTOR_MODE_LOW_SPEED, 0.0f, 0x00U, 0x00U},
        {QD4310_MOTOR_MODE_LOW_SPEED, QD4310_MOTOR_VELOCITY_MAX_RAD_S / 4.0f, 0xFFU, 0x1FU},
        {QD4310_MOTOR_MODE_LOW_SPEED, -QD4310_MOTOR_VELOCITY_MAX_RAD_S / 4.0f, 0x01U, 0xE0U},
        {QD4310_MOTOR_MODE_LOW_SPEED, QD4310_MOTOR_VELOCITY_MAX_RAD_S, 0xFFU, 0x7FU},
        {QD4310_MOTOR_MODE_LOW_SPEED, -QD4310_MOTOR_VELOCITY_MAX_RAD_S, 0x01U, 0x80U},
        {QD4310_MOTOR_MODE_POSITION_STEP, 0.0f, 0x00U, 0x00U},
        {QD4310_MOTOR_MODE_POSITION_STEP, QD4310_MOTOR_TURN_RAD / 4.0f, 0xFFU, 0x1FU},
        {QD4310_MOTOR_MODE_POSITION_STEP, -QD4310_MOTOR_TURN_RAD / 4.0f, 0x01U, 0xE0U},
        {QD4310_MOTOR_MODE_POSITION_STEP, QD4310_MOTOR_TURN_RAD, 0xFFU, 0x7FU},
        {QD4310_MOTOR_MODE_POSITION_STEP, -QD4310_MOTOR_TURN_RAD, 0x01U, 0x80U}
    };
    size_t i;
    float applied;
    uint8_t id = 0U;
    QD4310_Motor_Info_t motor = QD4310_MOTOR_CREATE(id++);
    assert(id == 1U && motor.config.motor_id == 0U);
    assert(!motor.feedback.valid && motor.feedback.sequence == 0U && motor.feedback.last_update_ms == 0U);
    assert(motor.feedback.data.current_a == 0.0f && !motor.feedback.data.enabled);

    for (i = 0U; i < sizeof(vectors) / sizeof(vectors[0]); ++i)
    {
        assert(QD4310_Motor_Control(&test_bus, &motor, vectors[i].mode,
                                   vectors[i].value, &applied) == QD4310_MOTOR_OK);
        assert(applied == vectors[i].value);
        check_frame(0U, (uint8_t)vectors[i].mode, vectors[i].low, vectors[i].high);
    }
    motor.config.motor_id = QD4310_MOTOR_ID_MAX;
    for (i = 0U; i < 3U; ++i)
    {
        assert(QD4310_Motor_SendCommand(&test_bus, &motor,
                                       (QD4310_Motor_Command_e)i) == QD4310_MOTOR_OK);
        check_frame(QD4310_MOTOR_ID_MAX, (uint8_t)i, 0U, 0U);
        assert(!motor.feedback.valid); /* 发送成功不能假造已使能反馈。 */
    }
    assert(QD4310_Motor_Control(&test_bus, &motor, QD4310_MOTOR_MODE_CURRENT,
                               0.0f, NULL) == QD4310_MOTOR_OK);
}

/** @brief 每种模式的正负越界、NaN/Inf、量化步附近和发送失败输出契约。 */
static void test_control_boundaries(void)
{
    static const QD4310_Motor_Mode_e modes[] = {
        QD4310_MOTOR_MODE_CURRENT, QD4310_MOTOR_MODE_SPEED, QD4310_MOTOR_MODE_POSITION,
        QD4310_MOTOR_MODE_LOW_SPEED, QD4310_MOTOR_MODE_POSITION_STEP
    };
    static const float maxima[] = {
        10.0f, QD4310_MOTOR_VELOCITY_MAX_RAD_S, QD4310_MOTOR_TURN_RAD,
        QD4310_MOTOR_VELOCITY_MAX_RAD_S, QD4310_MOTOR_TURN_RAD
    };
    static const float invalid[] = {NAN, INFINITY, -INFINITY};
    static const HAL_StatusTypeDef failures[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
    QD4310_Motor_Info_t motor = QD4310_MOTOR_CREATE(1U);
    size_t i;
    size_t j;
    for (i = 0U; i < sizeof(modes) / sizeof(modes[0]); ++i)
    {
        float minimum = modes[i] == QD4310_MOTOR_MODE_POSITION ? 0.0f : -maxima[i];
        float applied = 123.0f;
        float code_max = modes[i] == QD4310_MOTOR_MODE_POSITION ? 65535.0f : 32767.0f;
        float step = maxima[i] / code_max;
        assert(QD4310_Motor_Control(&test_bus, &motor, modes[i], maxima[i] * 2.0f,
                                   &applied) == QD4310_MOTOR_CLAMPED);
        assert(applied == maxima[i]);
        check_frame(1U, (uint8_t)modes[i], 0xFFU,
                    modes[i] == QD4310_MOTOR_MODE_POSITION ? 0xFFU : 0x7FU);
        assert(QD4310_Motor_Control(&test_bus, &motor, modes[i], -maxima[i] * 2.0f,
                                   &applied) == QD4310_MOTOR_CLAMPED);
        assert(applied == minimum);
        check_frame(1U, (uint8_t)modes[i], (uint8_t)(minimum == 0.0f ? 0U : 1U),
                    (uint8_t)(minimum == 0.0f ? 0U : 0x80U));
        assert(QD4310_Motor_Control(&test_bus, &motor, modes[i], step * 0.99f,
                                   &applied) == QD4310_MOTOR_OK);
        check_frame(1U, (uint8_t)modes[i], 0U, 0U);
        assert(QD4310_Motor_Control(&test_bus, &motor, modes[i], step * 1.01f,
                                   &applied) == QD4310_MOTOR_OK);
        check_frame(1U, (uint8_t)modes[i], 1U, 0U);
        for (j = 0U; j < sizeof(invalid) / sizeof(invalid[0]); ++j)
        {
            unsigned int before = send_count;
            applied = 123.0f;
            assert(QD4310_Motor_Control(&test_bus, &motor, modes[i], invalid[j],
                                       &applied) == QD4310_MOTOR_INVALID_ARGUMENT);
            assert(applied == 123.0f && send_count == before);
            ++checks;
        }
        for (j = 0U; j < sizeof(failures) / sizeof(failures[0]); ++j)
        {
            transport_status = failures[j];
            applied = 123.0f;
            assert(QD4310_Motor_Control(&test_bus, &motor, modes[i], maxima[i] * 2.0f,
                                       &applied) == QD4310_MOTOR_SEND_FAILED);
            assert(applied == 123.0f);
            assert(QD4310_Motor_Control(&test_bus, &motor, modes[i], 0.0f,
                                       &applied) == QD4310_MOTOR_SEND_FAILED);
            assert(applied == 123.0f);
            ++checks;
        }
        transport_status = HAL_OK;
    }
}

/** @brief 验证力矩单位、黄金报文、换算端点、非法输入及发送失败的原子性。 */
static void test_torque_control(void)
{
    static const struct
    {
        float torque_nm;
        uint8_t low;
        uint8_t high;
    } vectors[] = {
        {0.0f, 0x00U, 0x00U},
        {0.027f, 0x47U, 0x01U},
        {-0.027f, 0xB9U, 0xFEU},
        {0.2025f, 0x99U, 0x09U},
        {-0.2025f, 0x67U, 0xF6U},
        {0.27f, 0xCCU, 0x0CU},
        {-0.27f, 0x34U, 0xF3U},
        {QD4310_MOTOR_TORQUE_MAX_NM, 0xFFU, 0x7FU},
        {-QD4310_MOTOR_TORQUE_MAX_NM, 0x01U, 0x80U}
    };
    static const float invalid[] = {NAN, INFINITY, -INFINITY};
    static const HAL_StatusTypeDef failures[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
    QD4310_Motor_Info_t motor = QD4310_MOTOR_CREATE(7U);
    QD4310_Motor_Info_t original = motor;
    float applied;
    float argument = 0.0f;
    float result = QD4310_MOTOR_TORQUE_TO_CURRENT_A(++argument);
    float step = 2.7f / 32767.0f;
    size_t i;
    unsigned int before;
    assert(argument == 1.0f && fabsf(result - 3.7037037f) < 0.000001f);
    result = QD4310_MOTOR_CURRENT_TO_TORQUE_NM(++argument);
    assert(argument == 2.0f && result == 0.54f);

    for (i = 0U; i < sizeof(vectors) / sizeof(vectors[0]); ++i)
    {
        before = send_count;
        assert(QD4310_Motor_ControlTorque(&test_bus, &motor, vectors[i].torque_nm,
                                         &applied) == QD4310_MOTOR_OK);
        assert(applied == vectors[i].torque_nm && send_count == before + 1U);
        check_frame(7U, 0x03U, vectors[i].low, vectors[i].high);
    }
    assert(QD4310_Motor_ControlTorque(&test_bus, &motor, FLT_MAX, &applied) ==
           QD4310_MOTOR_CLAMPED);
    assert(applied == QD4310_MOTOR_TORQUE_MAX_NM);
    check_frame(7U, 0x03U, 0xFFU, 0x7FU);
    assert(QD4310_Motor_ControlTorque(&test_bus, &motor, -FLT_MAX, &applied) ==
           QD4310_MOTOR_CLAMPED);
    assert(applied == -QD4310_MOTOR_TORQUE_MAX_NM);
    check_frame(7U, 0x03U, 0x01U, 0x80U);
    assert(QD4310_Motor_ControlTorque(&test_bus, &motor, step * 0.99f, NULL) == QD4310_MOTOR_OK);
    check_frame(7U, 0x03U, 0U, 0U);
    assert(QD4310_Motor_ControlTorque(&test_bus, &motor, step * 1.01f, NULL) == QD4310_MOTOR_OK);
    check_frame(7U, 0x03U, 1U, 0U);
    assert(QD4310_Motor_ControlTorque(&test_bus, &motor, -step * 1.01f, NULL) == QD4310_MOTOR_OK);
    check_frame(7U, 0x03U, 0xFFU, 0xFFU);

    before = send_count;
    applied = 123.0f;
    for (i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        assert(QD4310_Motor_ControlTorque(&test_bus, &motor, invalid[i], &applied) ==
               QD4310_MOTOR_INVALID_ARGUMENT);
        assert(applied == 123.0f);
        ++checks;
    }
    assert(QD4310_Motor_ControlTorque(NULL, &motor, 0.027f, &applied) ==
           QD4310_MOTOR_INVALID_ARGUMENT);
    assert(QD4310_Motor_ControlTorque(&test_bus, NULL, 0.027f, &applied) ==
           QD4310_MOTOR_INVALID_ARGUMENT);
    motor.config.motor_id = QD4310_MOTOR_ID_MAX + 1U;
    assert(QD4310_Motor_ControlTorque(&test_bus, &motor, 0.027f, &applied) ==
           QD4310_MOTOR_INVALID_ARGUMENT);
    assert(send_count == before && applied == 123.0f);
    motor.config.motor_id = original.config.motor_id;
    for (i = 0U; i < sizeof(failures) / sizeof(failures[0]); ++i)
    {
        transport_status = failures[i];
        assert(QD4310_Motor_ControlTorque(&test_bus, &motor, 0.2025f, &applied) ==
               QD4310_MOTOR_SEND_FAILED);
        assert(applied == 123.0f);
        assert(QD4310_Motor_ControlTorque(&test_bus, &motor, FLT_MAX, &applied) ==
               QD4310_MOTOR_SEND_FAILED);
        assert(applied == 123.0f);
        ++checks;
    }
    transport_status = HAL_OK;
    check_unchanged(&motor, &original);
}

/** @brief 接收黄金向量、-32768 边界、保留位、时间/序号及单圈回绕。 */
static void test_feedback(void)
{
    QD4310_Motor_Info_t motor = QD4310_MOTOR_CREATE(1U);
    FDCAN_RxHeaderTypeDef header = {0};
    const uint8_t positive[8] = {0xA5U, 0xCCU, 0xFFU, 0x7FU, 0xFFU, 0x7FU, 0xFFU, 0xFFU};
    const uint8_t negative[8] = {0xA4U, 0x55U, 0x00U, 0x80U, 0x01U, 0x80U, 0x00U, 0x00U};
    header.Identifier = 0x501U;
    header.IdType = FDCAN_STANDARD_ID;
    header.RxFrameType = FDCAN_DATA_FRAME;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.DataLength = FDCAN_DLC_BYTES_8;
    motor.feedback.sequence = UINT32_MAX;
    assert(QD4310_Motor_OnRx(&motor, &header, positive, 8U, UINT32_MAX) == QD4310_MOTOR_OK);
    assert(motor.feedback.valid && motor.feedback.sequence == 0U);
    assert(motor.feedback.last_update_ms == UINT32_MAX && motor.feedback.data.enabled);
    assert(motor.feedback.data.raw_status == 0xA5U && motor.feedback.data.reserved == 0xCCU);
    assert(fabsf(motor.feedback.data.current_a - 10.0f) < 0.000002f);
    assert(fabsf(motor.feedback.data.velocity_rad_s - QD4310_MOTOR_VELOCITY_MAX_RAD_S) < 0.00002f);
    assert(fabsf(motor.feedback.data.position_rad - QD4310_MOTOR_TURN_RAD) < 0.000001f);
    assert(QD4310_Motor_OnRx(&motor, &header, negative, 8U, 5U) == QD4310_MOTOR_OK);
    assert(motor.feedback.sequence == 1U && motor.feedback.last_update_ms == 5U);
    assert(!motor.feedback.data.enabled && motor.feedback.data.position_rad == 0.0f);
    assert(motor.feedback.data.raw_status == 0xA4U && motor.feedback.data.reserved == 0x55U);
    assert(fabsf(motor.feedback.data.current_a - (-32768.0f * 10.0f / 32767.0f)) < 0.000002f);
    assert(fabsf(motor.feedback.data.velocity_rad_s + QD4310_MOTOR_VELOCITY_MAX_RAD_S) < 0.00002f);
    ++checks;
}

/** @brief 不合法参数和逐项损坏的帧必须不发送、不覆盖旧输出或旧反馈。 */
static void test_rejections(void)
{
    QD4310_Motor_Info_t motor = QD4310_MOTOR_CREATE(1U);
    QD4310_Motor_Info_t old;
    FDCAN_RxHeaderTypeDef header = {0};
    FDCAN_RxHeaderTypeDef bad;
    uint8_t data[9] = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U};
    float applied = 123.0f;
    unsigned int before = send_count;
    unsigned int i;
    header.Identifier = 0x501U;
    header.IdType = FDCAN_STANDARD_ID;
    header.RxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    assert(QD4310_Motor_OnRx(&motor, &header, data, 8U, 2U) == QD4310_MOTOR_OK);
    old = motor;
    assert(QD4310_Motor_Control(NULL, &motor, QD4310_MOTOR_MODE_CURRENT, 1.0f,
                               &applied) == QD4310_MOTOR_INVALID_ARGUMENT);
    assert(QD4310_Motor_Control(&test_bus, NULL, QD4310_MOTOR_MODE_CURRENT, 1.0f,
                               &applied) == QD4310_MOTOR_INVALID_ARGUMENT);
    assert(QD4310_Motor_Control(&test_bus, &motor, (QD4310_Motor_Mode_e)0xFF, 1.0f,
                               &applied) == QD4310_MOTOR_INVALID_ARGUMENT);
    assert(QD4310_Motor_Control(&test_bus, &motor, (QD4310_Motor_Mode_e)-1, 1.0f,
                               &applied) == QD4310_MOTOR_INVALID_ARGUMENT);
    assert(QD4310_Motor_SendCommand(&test_bus, &motor, (QD4310_Motor_Command_e)3) ==
           QD4310_MOTOR_INVALID_ARGUMENT);
    assert(QD4310_Motor_SendCommand(&test_bus, &motor, (QD4310_Motor_Command_e)0xFE) ==
           QD4310_MOTOR_INVALID_ARGUMENT);
    assert(QD4310_Motor_SendCommand(NULL, &motor, QD4310_MOTOR_COMMAND_QUERY) ==
           QD4310_MOTOR_INVALID_ARGUMENT);
    assert(QD4310_Motor_SendCommand(&test_bus, NULL, QD4310_MOTOR_COMMAND_QUERY) ==
           QD4310_MOTOR_INVALID_ARGUMENT);
    assert(send_count == before && applied == 123.0f);
    check_unchanged(&motor, &old);
    for (i = 0U; i < 7U; ++i)
    {
        uint32_t length = 8U;
        bad = header;
        switch (i)
        {
            case 0U: bad.Identifier = 0x502U; break;
            case 1U: bad.IdType = FDCAN_EXTENDED_ID; break;
            case 2U: bad.RxFrameType = FDCAN_REMOTE_FRAME; break;
            case 3U: bad.FDFormat = FDCAN_FD_CAN; break;
            case 4U: bad.DataLength = FDCAN_DLC_BYTES_3; break;
            case 5U: length = 7U; break;
            default: length = 9U; break;
        }
        assert(QD4310_Motor_OnRx(&motor, &bad, data, length, 9U) == QD4310_MOTOR_BAD_FRAME);
        check_unchanged(&motor, &old);
    }
    assert(QD4310_Motor_OnRx(&motor, NULL, data, 8U, 9U) == QD4310_MOTOR_INVALID_ARGUMENT);
    assert(QD4310_Motor_OnRx(&motor, &header, NULL, 8U, 9U) == QD4310_MOTOR_INVALID_ARGUMENT);
    assert(QD4310_Motor_OnRx(NULL, &header, data, 8U, 9U) == QD4310_MOTOR_INVALID_ARGUMENT);
    check_unchanged(&motor, &old);
    motor.config.motor_id = 0x10U;
    old = motor;
    assert(QD4310_Motor_OnRx(&motor, &header, data, 8U, 9U) == QD4310_MOTOR_INVALID_ARGUMENT);
    assert(QD4310_Motor_Control(&test_bus, &motor, QD4310_MOTOR_MODE_CURRENT, 1.0f,
                               &applied) == QD4310_MOTOR_INVALID_ARGUMENT);
    assert(QD4310_Motor_SendCommand(&test_bus, &motor, QD4310_MOTOR_COMMAND_QUERY) ==
           QD4310_MOTOR_INVALID_ARGUMENT);
    assert(send_count == before && applied == 123.0f);
    check_unchanged(&motor, &old);
    transport_status = HAL_BUSY;
    motor.config.motor_id = 1U;
    assert(QD4310_Motor_SendCommand(&test_bus, &motor, QD4310_MOTOR_COMMAND_ENABLE) ==
           QD4310_MOTOR_SEND_FAILED);
    transport_status = HAL_OK;
}

/** @brief 在全量程采样，验证有符号/无符号往返误差不超过一个量化步。 */
static void test_roundtrip(void)
{
    static const QD4310_Motor_Mode_e modes[] = {
        QD4310_MOTOR_MODE_CURRENT, QD4310_MOTOR_MODE_SPEED, QD4310_MOTOR_MODE_POSITION,
        QD4310_MOTOR_MODE_LOW_SPEED, QD4310_MOTOR_MODE_POSITION_STEP
    };
    static const double maxima[] = {10.0, 104.71975511965977, 6.283185307179586,
                                    104.71975511965977, 6.283185307179586};
    QD4310_Motor_Info_t motor = QD4310_MOTOR_CREATE(0U);
    size_t i;
    int sample;
    for (i = 0U; i < sizeof(modes) / sizeof(modes[0]); ++i)
    {
        double minimum = modes[i] == QD4310_MOTOR_MODE_POSITION ? 0.0 : -maxima[i];
        double denominator = modes[i] == QD4310_MOTOR_MODE_POSITION ? 65535.0 : 32767.0;
        for (sample = 0; sample <= 1000; ++sample)
        {
            float value = (float)(minimum + (maxima[i] - minimum) * (double)sample / 1000.0);
            uint16_t raw;
            int32_t signed_code;
            double decoded;
            float applied;
            assert(QD4310_Motor_Control(&test_bus, &motor, modes[i], value, &applied) == QD4310_MOTOR_OK);
            raw = (uint16_t)((uint32_t)captured_data[1] | ((uint32_t)captured_data[2] << 8U));
            signed_code = raw <= 32767U ? (int32_t)raw : (int32_t)raw - 65536;
            decoded = modes[i] == QD4310_MOTOR_MODE_POSITION ?
                (double)raw * maxima[i] / denominator : (double)signed_code * maxima[i] / denominator;
            /* 一个量化步，另允许 binary32 存储满量程导致的浮点表示误差。 */
            assert(fabs(decoded - (double)value) <= maxima[i] / denominator + maxima[i] * 1.0e-7);
            assert(applied == value);
            ++checks;
        }
    }
}

/** @brief 运行全部协议回归，任何断言失败均以非零进程状态终止。 */
int main(void)
{
    test_vectors();
    test_control_boundaries();
    test_torque_control();
    test_feedback();
    test_rejections();
    test_roundtrip();
    printf("QD4310: %u checks passed\n", checks);
    return 0;
}
