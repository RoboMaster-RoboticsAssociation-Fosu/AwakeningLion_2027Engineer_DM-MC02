/**
 * @file arm_usb_protocol.h
 * @brief Pi 与从臂的八轴 USB 业务载荷；复用 BoardLink 的流组帧与 CRC。
 * @note 所有 float 均为 IEEE-754 binary32、小端；不直接传输结构体内存。
 *       轴顺序：Yaw、Pitch1、Roll1、Pitch2、Roll2、Pitch3、Roll3、夹爪。
 */
#ifndef ARM_USB_PROTOCOL_H
#define ARM_USB_PROTOCOL_H

#include "board_link_protocol.h"

#define ARM_USB_JOINT_COUNT 8U
#define ARM_USB_COMMAND_BYTES 32U
#define ARM_USB_FEEDBACK_BYTES 97U
#define ARM_USB_FEEDBACK_FRAME_BYTES (BOARD_LINK_HEADER_BYTES + ARM_USB_FEEDBACK_BYTES + 2U)

/** @brief 类型只区分业务；USB 与 USART10 使用独立解析实例。 */
typedef enum
{
    ARM_USB_MESSAGE_COMMAND = 0x10,
    ARM_USB_MESSAGE_FEEDBACK = 0x11
} ArmUsbMessage_e;

/** @brief Pi 下发的关节输出轴目标角度；接收本身不代表电机执行。 */
typedef struct
{
    float target_angle_rad[ARM_USB_JOINT_COUNT]; /* rad */
} ArmUsbCommand_t;

/** @brief 无效观测允许 NaN；只有 load_valid_mask 置位的力矩可用于力反馈。 */
typedef struct
{
    float angle_rad[ARM_USB_JOINT_COUNT];       /* rad */
    float velocity_rad_s[ARM_USB_JOINT_COUNT];  /* rad/s */
    float load_torque_nm[ARM_USB_JOINT_COUNT];  /* 扣除本体/固定工具重力，保留物体重量，N.m */
    uint8_t load_valid_mask;                    /* bit0~7 对应约定轴顺序 */
} ArmUsbFeedback_t;

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 校验类型、长度和八轴有限值；失败时保持 command 不变。 */
bool ArmUsbProtocol_DecodeCommand(const BoardLinkFrame_t *frame, ArmUsbCommand_t *command);

/**
 * @brief 编码 107 字节反馈帧；序号回显最近一次接受的命令序号，启动前为 0。
 * @return 成功返回帧长；空指针、容量不足或有效力矩非有限值时返回 0，不写 output。
 * @note 无效轴的 load_torque_nm 可为 NaN，接收端必须检查有效位。
 */
size_t ArmUsbProtocol_EncodeFeedback(const ArmUsbFeedback_t *feedback,
                                      uint16_t command_sequence,
                                      uint8_t *output, size_t capacity);

#ifdef __cplusplus
}
#endif
#endif /* ARM_USB_PROTOCOL_H */
