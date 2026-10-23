/**
 * @file arm_usb_system.c
 * @brief USB ISR 入队 → ServiceTask 流解析 → 接收快照/实际反馈上传。
 * @note 通信接收与 ArmTask 目标分离；本模块不调用电机驱动，不写运动目标。
 */
#include "arm_usb_system.h"

#include "FreeRTOS.h"
#include "task.h"
#include "arm_task.h"
#include "bsp_usb.h"
#include <math.h>

#define ARM_USB_RX_RING_BYTES 1024U

typedef char ArmUsbJointCountCheck[(ARM_USB_JOINT_COUNT == ARM_JOINT_COUNT) ? 1 : -1];

ArmUsbSystem_t g_arm_usb;

/** @brief ISR 只修改队列与重置证据；解析器和发送帧仅由 ServiceTask 使用。 */
static struct
{
    uint8_t rx_ring[ARM_USB_RX_RING_BYTES];
    volatile uint16_t read;
    volatile uint16_t write;
    volatile bool reset_pending;
    volatile uint32_t overflow_count;
    BoardLinkParser_t parser;
    BoardLinkFrame_t frame;
    uint8_t tx_frame[ARM_USB_FEEDBACK_FRAME_BYTES];
} g_arm_usb_io;

void ArmUsbSystem_OnUsbReset(void)
{
    const UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();

    g_arm_usb_io.read = 0U;
    g_arm_usb_io.write = 0U;
    g_arm_usb_io.reset_pending = true;
    taskEXIT_CRITICAL_FROM_ISR(mask);
}

void ArmUsbSystem_OnUsbRx(const uint8_t *data, uint32_t length)
{
    uint32_t index;
    uint32_t free_bytes;
    UBaseType_t mask;

    if ((data == NULL) || (length == 0U))
    {
        return;
    }
    mask = taskENTER_CRITICAL_FROM_ISR();
    free_bytes = (uint32_t)((g_arm_usb_io.read + ARM_USB_RX_RING_BYTES -
                             g_arm_usb_io.write - 1U) % ARM_USB_RX_RING_BYTES);
    if (length > free_bytes)
    {
        /* 整包丢弃并清理旧半帧；不能把丢字节前后的数据拼成一个命令。 */
        g_arm_usb_io.read = 0U;
        g_arm_usb_io.write = 0U;
        g_arm_usb_io.reset_pending = true;
        g_arm_usb_io.overflow_count++;
        taskEXIT_CRITICAL_FROM_ISR(mask);
        return;
    }
    for (index = 0U; index < length; ++index)
    {
        g_arm_usb_io.rx_ring[g_arm_usb_io.write] = data[index];
        g_arm_usb_io.write =
            (uint16_t)((g_arm_usb_io.write + 1U) % ARM_USB_RX_RING_BYTES);
    }
    taskEXIT_CRITICAL_FROM_ISR(mask);
}

void ArmUsbSystem_Step(void)
{
    ArmUsbFeedback_t feedback;
    ArmUsbCommand_t command;
    uint32_t received;
    uint32_t joint;
    size_t frame_bytes;
    uint8_t byte;
    bool available;

    /* 每次最多消费一个队列容量，持续到达的数据不会使后台任务无限滞留。 */
    for (received = 0U; received < ARM_USB_RX_RING_BYTES; ++received)
    {
        taskENTER_CRITICAL();
        if (g_arm_usb_io.reset_pending)
        {
            g_arm_usb_io.parser.used = 0U;
            g_arm_usb_io.reset_pending = false;
            g_arm_usb.command_sequence = 0U;
        }
        g_arm_usb.rx_overflow_count = g_arm_usb_io.overflow_count;
        available = g_arm_usb_io.read != g_arm_usb_io.write;
        byte = available ? g_arm_usb_io.rx_ring[g_arm_usb_io.read] : 0U;
        if (available)
        {
            g_arm_usb_io.read =
                (uint16_t)((g_arm_usb_io.read + 1U) % ARM_USB_RX_RING_BYTES);
        }
        taskEXIT_CRITICAL();
        if (!available)
        {
            break;
        }
        if (BoardLinkProtocol_PushByte(&g_arm_usb_io.parser, byte, &g_arm_usb_io.frame))
        {
            if (!ArmUsbProtocol_DecodeCommand(&g_arm_usb_io.frame, &command))
            {
                g_arm_usb.rx_rejected_count++;
                continue;
            }
            taskENTER_CRITICAL();
            /* USB 重配置或溢出发生在解析期间时，不发布来自旧流的完整帧。 */
            if (!g_arm_usb_io.reset_pending)
            {
                g_arm_usb.received_command = command;
                g_arm_usb.command_sequence = g_arm_usb_io.frame.sequence;
                g_arm_usb.rx_command_count++;
            }
            taskEXIT_CRITICAL();
        }
    }

    /* 只读取 ArmTask 发布的真实反馈，离线轴使用 NaN，避免伪造零角度。 */
    taskENTER_CRITICAL();
    for (joint = 0U; joint < ARM_USB_JOINT_COUNT; ++joint)
    {
        feedback.angle_rad[joint] = g_arm_task.joint_feedback[joint].valid
                                       ? g_arm_task.joint_feedback[joint].angle_rad : NAN;
        feedback.velocity_rad_s[joint] = g_arm_task.joint_feedback[joint].valid
                                            ? g_arm_task.joint_feedback[joint].velocity_rad_s : NAN;
    }
    taskEXIT_CRITICAL();

    /* BEGIN 通信搭建阶段：重力模型/负载估算尚未接入，不能上传原始电机力矩冒充负载。
     * 范围仅负载字段；真实负载估算发布路径接入后，替换本段并按轴设置有效位。
     */
    feedback.load_valid_mask = 0U;
    for (joint = 0U; joint < ARM_USB_JOINT_COUNT; ++joint)
    {
        feedback.load_torque_nm[joint] = NAN;
    }
    /* END 通信搭建阶段的负载不可用表示。 */

    frame_bytes = ArmUsbProtocol_EncodeFeedback(&feedback, g_arm_usb.command_sequence,
                                                g_arm_usb_io.tx_frame,
                                                sizeof(g_arm_usb_io.tx_frame));
    if ((frame_bytes != 0U) && USB_Transmit_Data(g_arm_usb_io.tx_frame, (uint16_t)frame_bytes))
    {
        g_arm_usb.tx_frame_count++;
    }
    else
    {
        g_arm_usb.tx_not_sent_count++;
    }
}
