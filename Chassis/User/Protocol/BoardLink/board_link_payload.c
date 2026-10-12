/** @file board_link_payload.c @brief 从臂遥控解码与底盘姿态编码。 */
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

/** @brief 读取线协议有符号 16 位量，避免越界无符号到有符号转换。 */
static int16_t board_link_get_i16(const uint8_t *payload)
{
    uint16_t raw = (uint16_t)((uint16_t)payload[0] |
                             ((uint16_t)payload[1] << 8));
    int32_t value = raw <= INT16_MAX ? (int32_t)raw : (int32_t)raw - 65536;
    return (int16_t)value;
}

bool BoardLinkPayload_DecodeRemote(const uint8_t *payload, size_t length,
                                   Remote_Info_t *remote)
{
    Remote_Info_t decoded = {.rc = {.ch = {0}}};
    uint32_t channel;

    if ((payload == NULL) || (remote == NULL) ||
        (length != BOARD_LINK_REMOTE_BYTES) || (payload[22] > 1U))
    {
        return false;
    }
    for (channel = 0U; channel < 4U; channel++)
    {
        if (payload[24U + channel] > 1U)
        {
            return false;
        }
        decoded.rc.ch[channel] = board_link_get_i16(&payload[channel * 2U]);
        decoded.rc_active[channel] = payload[24U + channel] != 0U;
        decoded.Last_Remote_Active_Time[channel] =
            board_link_get_u32(&payload[28U + channel * 4U]);
    }
    decoded.rc.iw = board_link_get_i16(&payload[8]);
    decoded.mouse.x = board_link_get_i16(&payload[10]);
    decoded.mouse.y = board_link_get_i16(&payload[12]);
    decoded.mouse.z = board_link_get_i16(&payload[14]);
    decoded.rc.s[0] = payload[16];
    decoded.rc.s[1] = payload[17];
    decoded.mouse.press_l = payload[18];
    decoded.mouse.press_r = payload[19];
    decoded.key.v = (uint16_t)((uint16_t)payload[20] | ((uint16_t)payload[21] << 8));
    decoded.rc_lost = payload[22] != 0U;
    decoded.online_cnt = payload[23];
    *remote = decoded;
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

bool BoardLinkPayload_EncodeIns(const INS_t *ins,
                                uint8_t *payload, size_t capacity)
{
    /* 显式字段表只形成 36 个值，不复制 INS_t 的对象表示。 */
    float values[36];
    uint32_t index = 0U;
    uint32_t bits;

    if ((ins == NULL) || (payload == NULL) || (capacity < BOARD_LINK_INS_BYTES))
    {
        return false;
    }
#define BOARD_LINK_INS_LOAD(field) values[index++] = ins->field;
    BOARD_LINK_INS_FLOAT_FIELDS(BOARD_LINK_INS_LOAD)
#undef BOARD_LINK_INS_LOAD
    for (index = 0U; index < 36U; index++)
    {
        memcpy(&bits, &values[index], sizeof(bits));
        board_link_put_u32(&payload[index * 4U], bits);
    }
    payload[144] = ins->ins_flag;
    board_link_put_u32(&payload[145], ins->last_update_time_ms);
    return true;
}

#undef BOARD_LINK_INS_FLOAT_FIELDS
