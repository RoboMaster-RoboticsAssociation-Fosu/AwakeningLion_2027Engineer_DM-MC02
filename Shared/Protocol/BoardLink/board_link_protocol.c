/** @file board_link_protocol.c @brief 有界串口流组帧及 CRC 重同步。 */
#include "board_link_protocol.h"

#include "CRC.h"
#include <string.h>

/** @brief 将帧头中的两字节字段按小端读取，不依赖对齐。 */
static uint16_t board_link_read_u16(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

size_t BoardLinkProtocol_Encode(uint8_t type, uint16_t sequence,
                               const uint8_t *payload, uint16_t length,
                               uint8_t *output, size_t capacity)
{
    size_t total = BOARD_LINK_HEADER_BYTES + (size_t)length + 2U;
    uint16_t crc;

    if ((output == NULL) || (length > BOARD_LINK_PAYLOAD_MAX) ||
        ((length != 0U) && (payload == NULL)) || (capacity < total))
    {
        return 0U;
    }
    output[0] = 0xA5U;
    output[1] = 0x5AU;
    output[2] = BOARD_LINK_PROTOCOL_VERSION;
    output[3] = type;
    output[4] = (uint8_t)length;
    output[5] = (uint8_t)(length >> 8);
    output[6] = (uint8_t)sequence;
    output[7] = (uint8_t)(sequence >> 8);
    if (length != 0U)
    {
        memcpy(&output[BOARD_LINK_HEADER_BYTES], payload, length);
    }
    crc = Get_CRC16_Check_Sum(output, (uint32_t)(total - 2U), 0xFFFFU);
    output[total - 2U] = (uint8_t)crc;
    output[total - 1U] = (uint8_t)(crc >> 8);
    return total;
}

bool BoardLinkProtocol_PushByte(BoardLinkParser_t *parser, uint8_t byte,
                                BoardLinkFrame_t *frame)
{
    uint16_t length;
    uint16_t total;

    if ((parser == NULL) || (frame == NULL))
    {
        return false;
    }
    /* used 只在本函数内增加；完整最大帧必然在本次校验后被消费或重同步。 */
    if (parser->used >= BOARD_LINK_FRAME_MAX)
    {
        parser->used = 0U;
    }
    parser->bytes[parser->used++] = byte;

    while (parser->used != 0U)
    {
        if ((parser->bytes[0] != 0xA5U) ||
            ((parser->used >= 2U) && (parser->bytes[1] != 0x5AU)) ||
            ((parser->used >= 3U) &&
             (parser->bytes[2] != BOARD_LINK_PROTOCOL_VERSION)))
        {
            parser->used--;
            memmove(parser->bytes, &parser->bytes[1], parser->used);
            continue;
        }
        if (parser->used < BOARD_LINK_HEADER_BYTES)
        {
            return false;
        }
        length = board_link_read_u16(&parser->bytes[4]);
        if (length > BOARD_LINK_PAYLOAD_MAX)
        {
            parser->used--;
            memmove(parser->bytes, &parser->bytes[1], parser->used);
            continue;
        }
        total = (uint16_t)(BOARD_LINK_HEADER_BYTES + length + 2U);
        if (parser->used < total)
        {
            return false;
        }
        if (Get_CRC16_Check_Sum(parser->bytes, (uint32_t)total - 2U, 0xFFFFU) !=
            board_link_read_u16(&parser->bytes[total - 2U]))
        {
            parser->used--;
            memmove(parser->bytes, &parser->bytes[1], parser->used);
            continue;
        }
        frame->type = parser->bytes[3];
        frame->sequence = board_link_read_u16(&parser->bytes[6]);
        frame->length = length;
        memcpy(frame->payload, &parser->bytes[BOARD_LINK_HEADER_BYTES], length);
        parser->used = (uint16_t)(parser->used - total);
        memmove(parser->bytes, &parser->bytes[total], parser->used);
        return true;
    }
    return false;
}
