/**
 ******************************************************************************
 * @file    custom_controller_protocol.h
 * @brief   自定义控制器39字节串口协议的纯解析接口。
 ******************************************************************************
 */

#ifndef CUSTOM_CONTROLLER_PROTOCOL_H
#define CUSTOM_CONTROLLER_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define CUSTOM_CONTROLLER_FRAME_SOF          0xA5U
#define CUSTOM_CONTROLLER_COMMAND_ID         0x0302U
#define CUSTOM_CONTROLLER_PAYLOAD_LENGTH     30U
#define CUSTOM_CONTROLLER_HEADER_LENGTH       5U
#define CUSTOM_CONTROLLER_COMMAND_LENGTH      2U
#define CUSTOM_CONTROLLER_CRC16_LENGTH         2U
#define CUSTOM_CONTROLLER_FRAME_LENGTH        39U
#define CUSTOM_CONTROLLER_JOINT_COUNT           6U
#define CUSTOM_CONTROLLER_WORK_MODE_ENCODER    0U

/** @brief 线上六轴最终关节目标的固定顺序。 */
typedef enum
{
    CUSTOM_CONTROLLER_JOINT_BIG_YAW = 0,
    CUSTOM_CONTROLLER_JOINT_PITCH1,
    CUSTOM_CONTROLLER_JOINT_PITCH2,
    CUSTOM_CONTROLLER_JOINT_ROLL2,
    CUSTOM_CONTROLLER_JOINT_PITCH3,
    CUSTOM_CONTROLLER_JOINT_ROLL3,
    CUSTOM_CONTROLLER_JOINT_COUNT_VALUE
} CustomControllerJoint_e;

/** @brief 通过完整协议校验后的30字节业务载荷。 */
typedef struct
{
    uint8_t work_mode;
    float joint_target_rad[CUSTOM_CONTROLLER_JOINT_COUNT];
    bool button_pressed;
} CustomControllerPayload_t;

/** @brief 可跨任意串口分块边界连续喂字节的协议解析状态。 */
typedef struct
{
    uint8_t frame[CUSTOM_CONTROLLER_FRAME_LENGTH];
    uint16_t index;
} CustomControllerParser_t;

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 清空解析状态并重新等待SOF。 */
void CustomControllerProtocol_Init(CustomControllerParser_t *parser);

/**
 * @brief 校验并解码一帧，失败时保持payload不变。
 * @return true表示SOF、长度、命令、CRC和载荷字段全部有效。
 */
bool CustomControllerProtocol_DecodeFrame(
    const uint8_t frame[CUSTOM_CONTROLLER_FRAME_LENGTH],
    uint16_t frame_length,
    CustomControllerPayload_t *payload);

/**
 * @brief 向解析器喂入一个字节。
 * @return true表示本字节完成了一帧合法数据并写入payload。
 */
bool CustomControllerProtocol_AcceptByte(
    CustomControllerParser_t *parser,
    uint8_t byte,
    CustomControllerPayload_t *payload);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_CONTROLLER_PROTOCOL_H */
