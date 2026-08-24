/**
 ******************************************************************************
 * @file    custom_controller_protocol.c
 * @brief   自定义控制器39字节串口协议的校验和解码。
 ******************************************************************************
 */

#include "custom_controller_protocol.h"

#include "CRC.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define CUSTOM_CONTROLLER_DATA_OFFSET \
    (CUSTOM_CONTROLLER_HEADER_LENGTH + CUSTOM_CONTROLLER_COMMAND_LENGTH)
#define CUSTOM_CONTROLLER_BUTTON_OFFSET \
    (CUSTOM_CONTROLLER_DATA_OFFSET + 1U + \
     CUSTOM_CONTROLLER_JOINT_COUNT * sizeof(float))

static uint16_t custom_controller_read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] |
                      ((uint16_t)data[1] << 8U));
}

static float custom_controller_read_float_le(const uint8_t *data)
{
    uint32_t bits = (uint32_t)data[0] |
                    ((uint32_t)data[1] << 8U) |
                    ((uint32_t)data[2] << 16U) |
                    ((uint32_t)data[3] << 24U);
    float value;

    memcpy(&value, &bits, sizeof(value));
    return value;
}

void CustomControllerProtocol_Init(CustomControllerParser_t *parser)
{
    if (parser == NULL)
    {
        return;
    }
    parser->index = 0U;
}

bool CustomControllerProtocol_DecodeFrame(
    const uint8_t frame[CUSTOM_CONTROLLER_FRAME_LENGTH],
    uint16_t frame_length,
    CustomControllerPayload_t *payload)
{
    CustomControllerPayload_t decoded = {0};
    uint16_t command_id;
    uint32_t joint;
    uint8_t button_value;

    if ((frame == NULL) || (payload == NULL) ||
        (frame_length != CUSTOM_CONTROLLER_FRAME_LENGTH) ||
        (frame[0] != CUSTOM_CONTROLLER_FRAME_SOF) ||
        (custom_controller_read_u16_le(&frame[1]) !=
         CUSTOM_CONTROLLER_PAYLOAD_LENGTH) ||
        !Verify_CRC8_Check_Sum((uint8_t *)frame,
                               CUSTOM_CONTROLLER_HEADER_LENGTH) ||
        !Verify_CRC16_Check_Sum((uint8_t *)frame,
                                CUSTOM_CONTROLLER_FRAME_LENGTH))
    {
        return false;
    }

    command_id = custom_controller_read_u16_le(
        &frame[CUSTOM_CONTROLLER_HEADER_LENGTH]);
    if (command_id != CUSTOM_CONTROLLER_COMMAND_ID)
    {
        return false;
    }

    decoded.work_mode = frame[CUSTOM_CONTROLLER_DATA_OFFSET];
    if (decoded.work_mode != CUSTOM_CONTROLLER_WORK_MODE_ENCODER)
    {
        return false;
    }

    for (joint = 0U; joint < CUSTOM_CONTROLLER_JOINT_COUNT; joint++)
    {
        decoded.joint_target_rad[joint] = custom_controller_read_float_le(
            &frame[CUSTOM_CONTROLLER_DATA_OFFSET + 1U +
                   joint * sizeof(float)]);
        if (!isfinite(decoded.joint_target_rad[joint]))
        {
            return false;
        }
    }

    button_value = frame[CUSTOM_CONTROLLER_BUTTON_OFFSET];
    if (button_value > 1U)
    {
        return false;
    }
    decoded.button_pressed = button_value != 0U;
    *payload = decoded;
    return true;
}

bool CustomControllerProtocol_AcceptByte(
    CustomControllerParser_t *parser,
    uint8_t byte,
    CustomControllerPayload_t *payload)
{
    bool frame_valid;

    if ((parser == NULL) || (payload == NULL))
    {
        return false;
    }

    if (parser->index == 0U)
    {
        if (byte != CUSTOM_CONTROLLER_FRAME_SOF)
        {
            return false;
        }
        parser->frame[parser->index++] = byte;
        return false;
    }

    parser->frame[parser->index++] = byte;
    if (parser->index == CUSTOM_CONTROLLER_HEADER_LENGTH)
    {
        if ((custom_controller_read_u16_le(&parser->frame[1]) !=
             CUSTOM_CONTROLLER_PAYLOAD_LENGTH) ||
            !Verify_CRC8_Check_Sum(parser->frame,
                                   CUSTOM_CONTROLLER_HEADER_LENGTH))
        {
            parser->index = 0U;
        }
        return false;
    }

    if (parser->index < CUSTOM_CONTROLLER_FRAME_LENGTH)
    {
        return false;
    }

    frame_valid = CustomControllerProtocol_DecodeFrame(
        parser->frame,
        CUSTOM_CONTROLLER_FRAME_LENGTH,
        payload);
    parser->index = 0U;
    return frame_valid;
}
