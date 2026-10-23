/**
 * @file test_arm_usb.c
 * @brief Native tests of production USB codecs, queue owner and persistent TX buffer.
 * @note Only FreeRTOS critical sections, ArmTask publication and USB hardware are replaced.
 *       Production function bodies are copied verbatim by run_usb_tests.py, excluding includes.
 */
#include "arm_usb_system.h"
#include "bsp_usb.h"
#include "CRC.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define APP_TX_DATA_SIZE 2048U
#define USBD_STATE_CONFIGURED 3U
#define USBD_OK 0U
#define USBD_FAIL 2U

typedef unsigned int UBaseType_t;
typedef struct { uint32_t TxState; } USBD_CDC_HandleTypeDef;
typedef struct { uint8_t dev_state; void *pClassData; } USBD_HandleTypeDef;

static unsigned int critical_depth;
#define taskENTER_CRITICAL() ((void)++critical_depth)
#define taskEXIT_CRITICAL() ((void)--critical_depth)
#define taskENTER_CRITICAL_FROM_ISR() (critical_depth++)
#define taskEXIT_CRITICAL_FROM_ISR(saved) ((void)(critical_depth = (saved)))

static struct
{
    struct { float angle_rad; float velocity_rad_s; bool valid; } joint_feedback[8];
} g_arm_task;
#define ARM_JOINT_COUNT 8U

USBD_HandleTypeDef hUsbDeviceHS;
uint8_t UserTxBufferHS[APP_TX_DATA_SIZE];
static USBD_CDC_HandleTypeDef test_cdc;
static uint8_t *in_flight;
static uint16_t in_flight_length;

/** @brief Hardware replacement retains the actual pointer until completion, like USB CDC. */
static uint8_t CDC_Transmit_HS(uint8_t *data, uint16_t length)
{
    assert(critical_depth != 0U);
    assert(test_cdc.TxState == 0U);
    in_flight = data;
    in_flight_length = length;
    test_cdc.TxState = 1U;
    return USBD_OK;
}

#include "bsp_usb_production.inc"
#include "arm_usb_system_production.inc"

static const float golden_angles[8] = {0.0F, 1.0F, -1.0F, 2.0F, -2.0F, 0.5F, -0.5F, 12.5F};
static const uint8_t golden_command[] = {
    0xA5,0x5A,0x01,0x10,0x20,0x00,0xCD,0xAB,
    0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x3F,
    0x00,0x00,0x80,0xBF,0x00,0x00,0x00,0x40,
    0x00,0x00,0x00,0xC0,0x00,0x00,0x00,0x3F,
    0x00,0x00,0x00,0xBF,0x00,0x00,0x48,0x41,0xF2,0x67
};

/** @brief Recover a complete received frame without depending on USB packet boundaries. */
static BoardLinkFrame_t parse_frame(const uint8_t *data, size_t length)
{
    BoardLinkParser_t parser = {{0}, 0U};
    BoardLinkFrame_t frame = {0};
    size_t index;
    unsigned int count = 0U;

    for (index = 0U; index < length; ++index)
    {
        if (BoardLinkProtocol_PushByte(&parser, data[index], &frame))
        {
            count++;
        }
    }
    assert(count == 1U);
    return frame;
}

/** @brief Reject malformed types/lengths/nonfinite targets without partially replacing a command. */
static void test_codecs(void)
{
    BoardLinkFrame_t frame = parse_frame(golden_command, sizeof(golden_command));
    ArmUsbCommand_t command;
    ArmUsbFeedback_t feedback = {{0}, {0}, {0}, 0xFFU};
    uint8_t output[ARM_USB_FEEDBACK_FRAME_BYTES];
    uint8_t untouched[ARM_USB_FEEDBACK_FRAME_BYTES];
    uint32_t joint;

    assert(Get_CRC16_Check_Sum((uint8_t *)"123456789", 9U, 0xFFFFU) == 0x6F91U);
    assert(frame.sequence == 0xABCDU);
    assert(ArmUsbProtocol_DecodeCommand(&frame, &command));
    for (joint = 0U; joint < 8U; ++joint)
    {
        assert(command.target_angle_rad[joint] == golden_angles[joint]);
        feedback.angle_rad[joint] = golden_angles[joint];
        feedback.load_torque_nm[joint] = golden_angles[joint];
    }
    frame.type = ARM_USB_MESSAGE_FEEDBACK;
    assert(!ArmUsbProtocol_DecodeCommand(&frame, &command));
    frame.type = ARM_USB_MESSAGE_COMMAND;
    frame.length--;
    assert(!ArmUsbProtocol_DecodeCommand(&frame, &command));
    frame.length++;
    frame.payload[30] = 0xC0U;
    frame.payload[31] = 0x7FU; /* NaN at the last joint must not commit earlier fields. */
    assert(!ArmUsbProtocol_DecodeCommand(&frame, &command));
    for (joint = 0U; joint < 8U; ++joint)
    {
        assert(command.target_angle_rad[joint] == golden_angles[joint]);
    }
    assert(!ArmUsbProtocol_DecodeCommand(NULL, &command));
    assert(!ArmUsbProtocol_DecodeCommand(&frame, NULL));
    assert(ArmUsbProtocol_EncodeFeedback(&feedback, 0x1234U, output, sizeof(output)) == 107U);
    assert(output[0] == 0xA5U && output[1] == 0x5AU && output[3] == 0x11U);
    assert(output[4] == 97U && output[6] == 0x34U && output[7] == 0x12U);
    assert(output[104] == 0xFFU && output[105] == 0x47U && output[106] == 0x58U);
    frame = parse_frame(output, sizeof(output));
    assert(memcmp(frame.payload, &golden_command[8], 32U) == 0);
    assert(memcmp(&frame.payload[64], &golden_command[8], 32U) == 0);
    memset(output, 0xCC, sizeof(output));
    memcpy(untouched, output, sizeof(output));
    feedback.load_torque_nm[7] = NAN;
    assert(ArmUsbProtocol_EncodeFeedback(&feedback, 0U, output, sizeof(output)) == 0U);
    assert(memcmp(output, untouched, sizeof(output)) == 0);
    feedback.load_valid_mask &= (uint8_t)~(1U << 7U);
    assert(ArmUsbProtocol_EncodeFeedback(&feedback, 0U, output, sizeof(output)) == 107U);
    assert(ArmUsbProtocol_EncodeFeedback(&feedback, 0U, output, sizeof(output) - 1U) == 0U);
    assert(ArmUsbProtocol_EncodeFeedback(NULL, 0U, output, sizeof(output)) == 0U);
    assert(ArmUsbProtocol_EncodeFeedback(&feedback, 0U, NULL, sizeof(output)) == 0U);
}

/** @brief Verify busy/disconnected sends never corrupt the buffer still owned by USB. */
static void test_transport(void)
{
    uint8_t first[] = {0xA5U, 0x5AU, 0xAAU};
    uint8_t second[] = {0x11U, 0x22U, 0x33U};

    hUsbDeviceHS.dev_state = 0U;
    hUsbDeviceHS.pClassData = NULL;
    assert(!USB_Transmit_Data(first, sizeof(first)));
    hUsbDeviceHS.dev_state = USBD_STATE_CONFIGURED;
    assert(!USB_Transmit_Data(first, sizeof(first)));
    hUsbDeviceHS.pClassData = &test_cdc;
    assert(!USB_Transmit_Data(NULL, 1U));
    assert(!USB_Transmit_Data(first, 0U));
    assert(!USB_Transmit_Data(first, APP_TX_DATA_SIZE + 1U));
    assert(USB_Transmit_Data(first, sizeof(first)));
    assert(in_flight == UserTxBufferHS);
    assert(in_flight_length == sizeof(first));
    first[0] = 0U;
    assert(in_flight[0] == 0xA5U);
    assert(!USB_Transmit_Data(second, sizeof(second)));
    assert(in_flight[0] == 0xA5U);
    test_cdc.TxState = 0U;
}

/** @brief Exercise actual owner functions with arbitrary packet fragmentation and stream resets. */
static void test_owner(void)
{
    BoardLinkFrame_t feedback;
    uint8_t corrupted[sizeof(golden_command)];
    uint8_t overflow[1024] = {0};
    uint32_t accepted;
    uint32_t joint;
    float unavailable_load;

    for (joint = 0U; joint < 8U; ++joint)
    {
        g_arm_task.joint_feedback[joint].valid = true;
        g_arm_task.joint_feedback[joint].angle_rad = golden_angles[joint];
        g_arm_task.joint_feedback[joint].velocity_rad_s = -golden_angles[joint];
    }
    ArmUsbSystem_OnUsbReset();
    for (joint = 0U; joint < sizeof(golden_command); ++joint)
    {
        ArmUsbSystem_OnUsbRx(&golden_command[joint], 1U);
        test_cdc.TxState = 0U;
        ArmUsbSystem_Step();
    }
    assert(g_arm_usb.rx_command_count == 1U);
    assert(g_arm_usb.command_sequence == 0xABCDU);
    for (joint = 0U; joint < 8U; ++joint)
    {
        assert(g_arm_usb.received_command.target_angle_rad[joint] == golden_angles[joint]);
    }
    feedback = parse_frame(in_flight, in_flight_length);
    assert(feedback.sequence == 0xABCDU && feedback.length == 97U);
    assert(memcmp(feedback.payload, &golden_command[8], 32U) == 0);
    assert(feedback.payload[96] == 0U); /* No gravity model: load estimate is explicitly unavailable. */
    memcpy(&unavailable_load, &feedback.payload[64], sizeof(unavailable_load));
    assert(isnan(unavailable_load));
    accepted = g_arm_usb.rx_command_count;
    memcpy(corrupted, golden_command, sizeof(corrupted));
    corrupted[10] ^= 1U;
    ArmUsbSystem_OnUsbRx(corrupted, sizeof(corrupted));
    ArmUsbSystem_OnUsbRx(golden_command, sizeof(golden_command));
    test_cdc.TxState = 0U;
    ArmUsbSystem_Step();
    assert(g_arm_usb.rx_command_count == accepted + 1U);
    accepted = g_arm_usb.rx_command_count;
    ArmUsbSystem_OnUsbRx(golden_command, 20U);
    test_cdc.TxState = 0U;
    ArmUsbSystem_Step();
    ArmUsbSystem_OnUsbReset();
    ArmUsbSystem_OnUsbRx(&golden_command[20], (uint32_t)sizeof(golden_command) - 20U);
    test_cdc.TxState = 0U;
    ArmUsbSystem_Step();
    assert(g_arm_usb.rx_command_count == accepted);
    assert(g_arm_usb.command_sequence == 0U);
    ArmUsbSystem_OnUsbRx(overflow, sizeof(overflow));
    ArmUsbSystem_OnUsbRx(golden_command, sizeof(golden_command));
    test_cdc.TxState = 0U;
    ArmUsbSystem_Step();
    assert(g_arm_usb.rx_overflow_count == 1U);
    assert(g_arm_usb.rx_command_count == accepted + 1U);
    assert(critical_depth == 0U);
}

int main(void)
{
    test_codecs();
    test_transport();
    test_owner();
    puts("USB: golden vectors, bad commands, persistent TX, fragmentation, CRC recovery, reset and overflow passed");
    return 0;
}
