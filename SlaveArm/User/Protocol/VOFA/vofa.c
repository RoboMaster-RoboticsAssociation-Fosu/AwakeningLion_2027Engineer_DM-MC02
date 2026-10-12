/**
 ******************************************************************************
 * @file    vofa.c
 * @brief   VOFA+ JustFloat 有界帧打包与 UART 发送实现
 ******************************************************************************
 */

#include "vofa.h"

#include <stddef.h>
#include <string.h>

/* Private defines ---------------------------------------------------------- */
#define VOFA_JUSTFLOAT_FLOAT_BYTES (4U)
#define VOFA_JUSTFLOAT_TAIL_BYTES  (4U)
#define VOFA_JUSTFLOAT_FRAME_BYTES                                         \
    ((VOFA_JUSTFLOAT_MAX_FLOATS * VOFA_JUSTFLOAT_FLOAT_BYTES) +           \
     VOFA_JUSTFLOAT_TAIL_BYTES)

/* Private variables -------------------------------------------------------- */
/**
 * @brief 中断发送使用的持久缓冲区。
 * @note 只允许单任务 owner 调用 vofa_justfloat_send_it()；HAL UART 状态用于
 *       阻止上一帧完成前覆盖该缓冲区。
 */
static uint8_t vofa_it_frame[VOFA_JUSTFLOAT_FRAME_BYTES];

static const uint8_t vofa_justfloat_tail[VOFA_JUSTFLOAT_TAIL_BYTES] = {
    0x00U,
    0x00U,
    0x80U,
    0x7FU
};

/* Private functions -------------------------------------------------------- */
/**
 * @brief 将单精度浮点数按小端字节序写入 JustFloat 帧。
 * @note 先复制位模式到 uint32_t，避免违反 C 严格别名规则。
 */
static void vofa_float_to_le_bytes(
    float value,
    uint8_t bytes[VOFA_JUSTFLOAT_FLOAT_BYTES])
{
    uint32_t raw = 0U;

    (void)memcpy(&raw, &value, sizeof(raw));
    bytes[0] = (uint8_t)(raw & 0xFFU);
    bytes[1] = (uint8_t)((raw >> 8U) & 0xFFU);
    bytes[2] = (uint8_t)((raw >> 16U) & 0xFFU);
    bytes[3] = (uint8_t)((raw >> 24U) & 0xFFU);
}

/**
 * @brief 把浮点通道和固定帧尾打包到目标缓冲区。
 * @return 实际帧长；参数非法时返回 0。
 */
static uint16_t vofa_pack_justfloat_frame(
    const float *fdata,
    uint16_t fdata_num,
    uint8_t frame[VOFA_JUSTFLOAT_FRAME_BYTES])
{
    uint16_t index;
    uint16_t offset = 0U;

    if ((fdata == NULL) || (frame == NULL) ||
        (fdata_num == 0U) || (fdata_num > VOFA_JUSTFLOAT_MAX_FLOATS))
    {
        return 0U;
    }

    for (index = 0U; index < fdata_num; ++index)
    {
        vofa_float_to_le_bytes(fdata[index], &frame[offset]);
        offset = (uint16_t)(offset + VOFA_JUSTFLOAT_FLOAT_BYTES);
    }

    (void)memcpy(&frame[offset], vofa_justfloat_tail,
                 sizeof(vofa_justfloat_tail));
    return (uint16_t)(offset + sizeof(vofa_justfloat_tail));
}

/* Exported functions ------------------------------------------------------- */
HAL_StatusTypeDef vofa_justfloat_send(
    UART_HandleTypeDef *huart,
    const float *fdata,
    uint16_t fdata_num,
    uint32_t timeout_ms)
{
    uint8_t frame[VOFA_JUSTFLOAT_FRAME_BYTES];
    uint16_t frame_size;

    if ((huart == NULL) || ((fdata == NULL) && (fdata_num != 0U)) ||
        (fdata_num > VOFA_JUSTFLOAT_MAX_FLOATS))
    {
        return HAL_ERROR;
    }
    if (fdata_num == 0U)
    {
        return HAL_OK;
    }

    frame_size = vofa_pack_justfloat_frame(fdata, fdata_num, frame);
    if (frame_size == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(huart, frame, frame_size, timeout_ms);
}

HAL_StatusTypeDef vofa_justfloat_send_it(
    UART_HandleTypeDef *huart,
    const float *fdata,
    uint16_t fdata_num)
{
    uint16_t frame_size;

    if ((huart == NULL) || ((fdata == NULL) && (fdata_num != 0U)) ||
        (fdata_num > VOFA_JUSTFLOAT_MAX_FLOATS))
    {
        return HAL_ERROR;
    }
    if (fdata_num == 0U)
    {
        return HAL_OK;
    }

    /* 不得覆盖仍被 HAL 中断发送流程读取的模块内缓冲区。 */
    if (HAL_UART_GetState(huart) != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    frame_size = vofa_pack_justfloat_frame(fdata, fdata_num, vofa_it_frame);
    if (frame_size == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit_IT(huart, vofa_it_frame, frame_size);
}
