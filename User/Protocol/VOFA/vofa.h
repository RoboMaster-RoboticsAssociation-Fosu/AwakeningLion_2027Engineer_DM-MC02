/**
 ******************************************************************************
 * @file    vofa.h
 * @version V1.0.0
 * @date    2026.07.19
 * @brief   VOFA+ JustFloat 串口上传接口
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 数据帧由若干 IEEE-754 单精度浮点数和固定帧尾 00 00 80 7F 组成。
 * * vofa_justfloat_send_it() 使用模块内静态发送缓冲区，同一时刻只允许一帧
 *   异步发送，调用方必须保证该接口只有一个任务 owner。
 ******************************************************************************
 */

#ifndef VOFA_H
#define VOFA_H

/* Includes ----------------------------------------------------------------- */
#include <stdint.h>
#include "stm32h7xx_hal.h"

/* Defines ------------------------------------------------------------------ */
#define VOFA_JUSTFLOAT_MAX_FLOATS (32U)

/* Functions ---------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 通过指定 UART 阻塞发送一帧 JustFloat 数据。
 * @param huart 已初始化的 UART 句柄
 * @param fdata 按通道顺序排列的浮点数据
 * @param fdata_num 浮点数据个数，不得超过 VOFA_JUSTFLOAT_MAX_FLOATS
 * @param timeout_ms HAL 阻塞发送超时时间，单位 ms
 * @return HAL_OK 表示发送成功或数据个数为 0；参数非法返回 HAL_ERROR；
 *         其他情况返回 HAL_UART_Transmit() 的状态。
 */
HAL_StatusTypeDef vofa_justfloat_send(
    UART_HandleTypeDef *huart,
    const float *fdata,
    uint16_t fdata_num,
    uint32_t timeout_ms);

/**
 * @brief 通过指定 UART 中断发送一帧 JustFloat 数据。
 * @param huart 已初始化且已开启全局中断的 UART 句柄
 * @param fdata 按通道顺序排列的浮点数据
 * @param fdata_num 浮点数据个数，不得超过 VOFA_JUSTFLOAT_MAX_FLOATS
 * @return HAL_OK 表示已启动发送或数据个数为 0；上一帧尚未完成返回 HAL_BUSY；
 *         参数非法返回 HAL_ERROR；其他情况返回 HAL_UART_Transmit_IT() 的状态。
 * @note 发送缓冲区由本模块持有，发送完成前调用方可以释放或改写 fdata。
 */
HAL_StatusTypeDef vofa_justfloat_send_it(
    UART_HandleTypeDef *huart,
    const float *fdata,
    uint16_t fdata_num);

#ifdef __cplusplus
}
#endif

#endif /* VOFA_H */
