/**
 * @file board_link_payload.h
 * @brief Remote_Info_t 与 INS_t 的完整字段快照，不传结构体填充或位域布局。
 * @note 整数和 IEEE-754 binary32 均使用小端；源时间戳原样保留。
 */
#ifndef BOARD_LINK_PAYLOAD_H
#define BOARD_LINK_PAYLOAD_H

#include "Remote_Control.h"
#include "ins_types.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BOARD_LINK_REMOTE_BYTES 44U
#define BOARD_LINK_INS_BYTES 149U

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 完整编码快照；参数/容量错误返回 false，且不修改输出。 */
bool BoardLinkPayload_EncodeRemote(const Remote_Info_t *remote,
                                   uint8_t *payload, size_t capacity);
bool BoardLinkPayload_EncodeIns(const INS_t *ins,
                                uint8_t *payload, size_t capacity);
/** @brief 严格检查长度和布尔编码；失败不修改业务快照。 */
bool BoardLinkPayload_DecodeRemote(const uint8_t *payload, size_t length,
                                   Remote_Info_t *remote);
bool BoardLinkPayload_DecodeIns(const uint8_t *payload, size_t length,
                                INS_t *ins);

#ifdef __cplusplus
}
#endif
#endif /* BOARD_LINK_PAYLOAD_H */
