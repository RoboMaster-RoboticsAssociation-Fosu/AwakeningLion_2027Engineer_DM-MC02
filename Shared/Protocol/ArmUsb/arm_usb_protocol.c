/** @file arm_usb_protocol.c @brief 八轴目标解码和实际反馈的逐字段序列化。 */
#include "arm_usb_protocol.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <string.h>

/* C99 编译期约束；帧中每个 float 必须准确占用四个字节。 */
typedef char ArmUsbFloatFormatCheck[
    (CHAR_BIT == 8 && sizeof(float) == 4U && FLT_RADIX == 2 &&
     FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128) ? 1 : -1];

/** @brief 读取未对齐的小端浮点，不违反有效类型规则。 */
static float arm_usb_read_float(const uint8_t *bytes)
{
    uint32_t bits = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8U) |
                    ((uint32_t)bytes[2] << 16U) | ((uint32_t)bytes[3] << 24U);
    float value;

    memcpy(&value, &bits, sizeof(value));
    return value;
}

/** @brief 浮点对象表示经 memcpy 搬运，再显式按小端排列。 */
static void arm_usb_write_float(uint8_t *bytes, float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    bytes[0] = (uint8_t)bits;
    bytes[1] = (uint8_t)(bits >> 8U);
    bytes[2] = (uint8_t)(bits >> 16U);
    bytes[3] = (uint8_t)(bits >> 24U);
}

bool ArmUsbProtocol_DecodeCommand(const BoardLinkFrame_t *frame, ArmUsbCommand_t *command)
{
    ArmUsbCommand_t decoded;
    uint32_t joint;

    if ((frame == NULL) || (command == NULL) ||
        (frame->type != ARM_USB_MESSAGE_COMMAND) || (frame->length != ARM_USB_COMMAND_BYTES))
    {
        return false;
    }
    for (joint = 0U; joint < ARM_USB_JOINT_COUNT; ++joint)
    {
        decoded.target_angle_rad[joint] = arm_usb_read_float(&frame->payload[joint * 4U]);
        if (!isfinite(decoded.target_angle_rad[joint]))
        {
            return false;
        }
    }
    *command = decoded;
    return true;
}

size_t ArmUsbProtocol_EncodeFeedback(const ArmUsbFeedback_t *feedback,
                                      uint16_t command_sequence,
                                      uint8_t *output, size_t capacity)
{
    uint8_t payload[ARM_USB_FEEDBACK_BYTES];
    uint32_t joint;

    if ((feedback == NULL) || (output == NULL) || (capacity < ARM_USB_FEEDBACK_FRAME_BYTES))
    {
        return 0U;
    }
    for (joint = 0U; joint < ARM_USB_JOINT_COUNT; ++joint)
    {
        if (((feedback->load_valid_mask & (uint8_t)(1U << joint)) != 0U) &&
            !isfinite(feedback->load_torque_nm[joint]))
        {
            return 0U;
        }
        arm_usb_write_float(&payload[joint * 4U], feedback->angle_rad[joint]);
        arm_usb_write_float(&payload[32U + joint * 4U], feedback->velocity_rad_s[joint]);
        arm_usb_write_float(&payload[64U + joint * 4U], feedback->load_torque_nm[joint]);
    }
    payload[96] = feedback->load_valid_mask;
    return BoardLinkProtocol_Encode(ARM_USB_MESSAGE_FEEDBACK, command_sequence,
                                    payload, ARM_USB_FEEDBACK_BYTES, output, capacity);
}
