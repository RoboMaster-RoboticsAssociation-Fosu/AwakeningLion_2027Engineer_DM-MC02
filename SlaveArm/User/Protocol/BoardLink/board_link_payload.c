/** @file board_link_payload.c @brief 从臂遥控编码与底盘姿态解码。 */
#include "board_link_payload.h"

#include <float.h>
#include <limits.h>
#include <string.h>

/* C99 编译期检查：线协议要求 8 位字节和 IEEE-754 binary32 浮点表示。 */
typedef char BoardLinkFloatFormatCheck[
    (CHAR_BIT == 8 && sizeof(float) == 4U && FLT_RADIX == 2 &&
     FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128 && RC_CHANNEL_COUNT == 4) ? 1 : -1];

/** @brief 写入固定宽度的小端整数，多个业务字段共用。 */
static void board_link_put_u32(uint8_t *payload, uint32_t value)
{
    payload[0] = (uint8_t)value;
    payload[1] = (uint8_t)(value >> 8);
    payload[2] = (uint8_t)(value >> 16);
    payload[3] = (uint8_t)(value >> 24);
}

/** @brief 从四个字节组合整数；每次移位先扩展到 uint32_t。 */
static uint32_t board_link_get_u32(const uint8_t *payload)
{
    return (uint32_t)payload[0] | ((uint32_t)payload[1] << 8) |
           ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);
}

bool BoardLinkPayload_EncodeRemote(const Remote_Info_t *remote,
                                   uint8_t *payload, size_t capacity)
{
    uint32_t channel;
    int16_t analog[8];

    if ((remote == NULL) || (payload == NULL) ||
        (capacity < BOARD_LINK_REMOTE_BYTES))
    {
        return false;
    }
    analog[0] = remote->rc.ch[0];
    analog[1] = remote->rc.ch[1];
    analog[2] = remote->rc.ch[2];
    analog[3] = remote->rc.ch[3];
    analog[4] = remote->rc.iw;
    analog[5] = remote->mouse.x;
    analog[6] = remote->mouse.y;
    analog[7] = remote->mouse.z;
    for (channel = 0U; channel < 8U; channel++)
    {
        uint16_t raw = (uint16_t)analog[channel];
        payload[channel * 2U] = (uint8_t)raw;
        payload[channel * 2U + 1U] = (uint8_t)(raw >> 8);
    }
    payload[16] = remote->rc.s[0];
    payload[17] = remote->rc.s[1];
    payload[18] = remote->mouse.press_l;
    payload[19] = remote->mouse.press_r;
    payload[20] = (uint8_t)remote->key.v;
    payload[21] = (uint8_t)(remote->key.v >> 8);
    payload[22] = remote->rc_lost ? 1U : 0U;
    payload[23] = remote->online_cnt;
    for (channel = 0U; channel < 4U; channel++)
    {
        payload[24U + channel] = remote->rc_active[channel] ? 1U : 0U;
        board_link_put_u32(&payload[28U + channel * 4U],
                           remote->Last_Remote_Active_Time[channel]);
    }
    return true;
}

/**
 * @brief 逐字段列出 INS 中的 36 个浮点量；不跨数组访问或依赖字段邻接。
 * @note 私有宏仅列举真实线协议字段，不隐藏控制流或硬件操作。
 */
#define BOARD_LINK_INS_FLOAT_FIELDS(APPLY) \
    APPLY(q[0]) APPLY(q[1]) APPLY(q[2]) APPLY(q[3]) \
    APPLY(Gyro[0]) APPLY(Gyro[1]) APPLY(Gyro[2]) \
    APPLY(Accel[0]) APPLY(Accel[1]) APPLY(Accel[2]) \
    APPLY(MotionAccel_b[0]) APPLY(MotionAccel_b[1]) APPLY(MotionAccel_b[2]) \
    APPLY(MotionAccel_n[0]) APPLY(MotionAccel_n[1]) APPLY(MotionAccel_n[2]) \
    APPLY(AccelLPF) \
    APPLY(xn[0]) APPLY(xn[1]) APPLY(xn[2]) \
    APPLY(yn[0]) APPLY(yn[1]) APPLY(yn[2]) \
    APPLY(zn[0]) APPLY(zn[1]) APPLY(zn[2]) \
    APPLY(atanxz) APPLY(atanyz) APPLY(Roll) APPLY(Pitch) APPLY(Yaw) \
    APPLY(YawTotalAngle) APPLY(YawAngleLast) APPLY(YawRoundCount) APPLY(v_n) APPLY(x_n)

bool BoardLinkPayload_DecodeIns(const uint8_t *payload, size_t length,
                                INS_t *ins)
{
    INS_t decoded = {.q = {0.0f}};
    float values[36];
    uint32_t index;
    uint32_t bits;

    if ((payload == NULL) || (ins == NULL) || (length != BOARD_LINK_INS_BYTES))
    {
        return false;
    }
    for (index = 0U; index < 36U; index++)
    {
        bits = board_link_get_u32(&payload[index * 4U]);
        memcpy(&values[index], &bits, sizeof(bits));
    }
    index = 0U;
#define BOARD_LINK_INS_STORE(field) decoded.field = values[index++];
    BOARD_LINK_INS_FLOAT_FIELDS(BOARD_LINK_INS_STORE)
#undef BOARD_LINK_INS_STORE
    decoded.ins_flag = payload[144];
    decoded.last_update_time_ms = board_link_get_u32(&payload[145]);
    *ins = decoded;
    return true;
}

#undef BOARD_LINK_INS_FLOAT_FIELDS
