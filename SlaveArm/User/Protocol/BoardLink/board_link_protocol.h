/**
 * @file board_link_protocol.h
 * @brief 双主板串口帧的纯编解码；不依赖 HAL、任务或业务结构体布局。
 * @note 帧：A5 5A | version:u8 | type:u8 | length:u16LE | sequence:u16LE |
 *       payload:length | CRC16:u16LE。CRC 覆盖帧头和负载，初值 0xFFFF。
 */
#ifndef BOARD_LINK_PROTOCOL_H
#define BOARD_LINK_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BOARD_LINK_PROTOCOL_VERSION 1U
#define BOARD_LINK_HEADER_BYTES 8U
#define BOARD_LINK_PAYLOAD_MAX 256U
#define BOARD_LINK_FRAME_MAX (BOARD_LINK_HEADER_BYTES + BOARD_LINK_PAYLOAD_MAX + 2U)

/** @brief 消息类型只选择负载格式，不改变串口传输语义。 */
typedef enum
{
    BOARD_LINK_MESSAGE_REMOTE = 1,
    BOARD_LINK_MESSAGE_INS = 2
} BoardLinkMessage_e;

/** @brief 一个通过 CRC 校验的帧；未知 type 仍可由传输层接收。 */
typedef struct
{
    uint8_t type;
    uint16_t sequence;
    uint16_t length;
    uint8_t payload[BOARD_LINK_PAYLOAD_MAX];
} BoardLinkFrame_t;

/** @brief 单一接收 owner 持有的流解析状态；零初始化即为空流。 */
typedef struct
{
    uint8_t bytes[BOARD_LINK_FRAME_MAX];
    uint16_t used;
} BoardLinkParser_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 编码一帧；成功返回实际字节数，参数/容量错误返回 0 且不写 output。
 * @note payload 不得与 output 重叠；允许 length=0、payload=NULL。
 */
size_t BoardLinkProtocol_Encode(uint8_t type, uint16_t sequence,
                               const uint8_t *payload, uint16_t length,
                               uint8_t *output, size_t capacity);

/**
 * @brief 输入一字节；产出完整有效帧时返回 true，否则保持 frame 不变。
 * @note 支持分片、连续帧及错位重同步；长度/版本/CRC 错误逐字节寻找下一帧。
 *       parser 与 frame 不得重叠，均由一个调用者独占。
 */
bool BoardLinkProtocol_PushByte(BoardLinkParser_t *parser, uint8_t byte,
                                BoardLinkFrame_t *frame);

#ifdef __cplusplus
}
#endif
#endif /* BOARD_LINK_PROTOCOL_H */
