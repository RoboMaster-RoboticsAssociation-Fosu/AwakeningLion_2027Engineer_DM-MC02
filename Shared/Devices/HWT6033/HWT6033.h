/**
 ******************************************************************************
 * @file    HWT6033.h
 * @version V1.0.0
 * @date    2026.07.14
 * @brief   HWT6033-TTL 姿态传感器的 STM32H7 适配接口
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 本模块只绑定串口发送和解析入口，不接管串口初始化、DMA 或中断。
 * * 接收端必须把 HWT6033 的原始字节流交给 HWT6033_FeedData。
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef HWT6033_H
#define HWT6033_H

/* Includes ----------------------------------------------------------------- */
#include "stm32h7xx_hal.h"
#include <stdint.h>

/* Defines ------------------------------------------------------------------ */
#define HWT6033_DEFAULT_BAUD_RATE       9600U
#define HWT6033_UART_TX_TIMEOUT_MS      20U

#define HWT6033_UPDATE_ACCELERATION     (1UL << 0)
#define HWT6033_UPDATE_ANGULAR_VELOCITY (1UL << 1)
#define HWT6033_UPDATE_ANGLE            (1UL << 2)
#define HWT6033_UPDATE_TEMPERATURE      (1UL << 3)

/* Enums -------------------------------------------------------------------- */
/**
 * @brief HWT6033 适配层返回状态
 */
typedef enum
{
    HWT6033_STATUS_OK = 0,
    HWT6033_STATUS_ERROR = -1,
    HWT6033_STATUS_INVALID_ARGUMENT = -2,
    HWT6033_STATUS_NOT_INITIALIZED = -3,
} HWT6033_Status_e;

/* Structs ------------------------------------------------------------------ */
/**
 * @brief HWT6033 已换算的测量数据
 */
typedef struct
{
    float acceleration_g[3];       /*!< X/Y/Z 轴加速度，单位 g */
    float angular_velocity_dps[3]; /*!< X/Y/Z 轴角速度，单位 deg/s */
    float angle_deg[3];            /*!< Roll/Pitch/Yaw，单位 deg */
    float temperature_c;           /*!< 传感器温度，单位 degC */
    uint32_t update_flags;         /*!< 自上次清除以来的数据更新标志 */
    uint32_t last_update_ms;       /*!< 最近一次有效数据更新的 HAL tick */
} HWT6033_Data_t;

/* Functions ---------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 HWT6033 标准串口协议适配层
 * @param huart 已由 CubeMX/HAL 初始化的 UART 句柄
 * @return HWT6033_Status_e
 * @note UART 应配置为传感器当前波特率、8 数据位、无校验、1 停止位。
 */
HWT6033_Status_e HWT6033_Init(UART_HandleTypeDef *huart);

/**
 * @brief 解除当前 UART 绑定并复位官方 SDK 的回调状态
 * @return 无
 * @note 调用前应先停止向本模块投递 UART 接收数据。
 */
void HWT6033_DeInit(void);

/**
 * @brief 向官方协议解析器投递一个字节
 * @param data HWT6033 原始串口字节
 * @return HWT6033_Status_e
 */
HWT6033_Status_e HWT6033_FeedByte(uint8_t data);

/**
 * @brief 向官方协议解析器投递一段连续字节流
 * @param data 数据缓冲区
 * @param length 缓冲区有效字节数
 * @return HWT6033_Status_e
 * @note 可从 UART 空闲中断、DMA 接收回调或任务中调用。
 */
HWT6033_Status_e HWT6033_FeedData(const uint8_t *data, uint32_t length);

/**
 * @brief 原子复制最近一次 HWT6033 测量结果
 * @param data 接收测量结果的目标结构体
 * @return HWT6033_Status_e
 */
HWT6033_Status_e HWT6033_CopyData(HWT6033_Data_t *data);

/**
 * @brief 读取并清除累计更新标志
 * @return HWT6033_UPDATE_* 标志组合
 */
uint32_t HWT6033_TakeUpdateFlags(void);

/**
 * @brief 获取最近一次官方 SDK 串口发送的 HAL 状态
 * @return HAL_StatusTypeDef
 * @note 配置/校准指令通过官方 SDK 发送时可用此接口检查底层发送结果。
 */
HAL_StatusTypeDef HWT6033_GetLastTransmitStatus(void);

#ifdef __cplusplus
}
#endif

/* -------------------------------------------------------------------------- */
#endif /* HWT6033_H */
