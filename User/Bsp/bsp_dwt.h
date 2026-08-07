/**
 ******************************************************************************
 * @file    bsp_dwt.h
 * @version V2.0.0
 * @date    2026.07.20
 * @brief   Cortex-M DWT 单调时间与精确延时接口
 * @encoding UTF-8
 ******************************************************************************
 */

#ifndef BSP_DWT_H
#define BSP_DWT_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化 DWT 周期计数器和软件扩展时间。
 * @param cpu_frequency_hz 当前内核时钟频率，单位 Hz
 * @return true 表示周期计数器已成功启动
 * @note 必须在系统时钟配置完成后调用；重复调用会把 DWT 时间重新归零。
 */
bool DWT_Init(uint32_t cpu_frequency_hz);

/**
 * @brief 周期性维护 32 位 CYCCNT 的 64 位软件扩展。
 * @note 当前工程由 1 ms TIM2 HAL 时基中断调用，调用间隔必须小于一次
 *       CYCCNT 回绕时间。本函数也可在任务或其他中断上下文调用。
 */
void DWT_Update(void);

/** @brief 获取从 DWT_Init() 开始累计的 CPU 周期数。 */
uint64_t DWT_GetCycleCount64(void);

/** @brief 获取从 DWT_Init() 开始的单调微秒时间。 */
uint64_t DWT_GetTimeUs(void);

/**
 * @brief 获取从 DWT_Init() 开始的单调毫秒时间低 32 位。
 * @note 适合用无符号减法进行不超过 2^31 ms 的超时判断。
 */
uint32_t DWT_GetTimeMs(void);

/**
 * @brief 获取相邻两次调用间隔，单位秒。
 * @param last_cycle 调用方独占的上次 CYCCNT 快照；函数返回前自动更新
 * @note 适用于小于一次 CYCCNT 回绕时间的高频控制周期。
 */
float DWT_GetDeltaT(uint32_t *last_cycle);

/** @brief DWT_GetDeltaT() 的 double 返回版本。 */
double DWT_GetDeltaT64(uint32_t *last_cycle);

/** @brief 忙等待指定秒数，仅用于外设时序和启动阶段短延时。 */
void DWT_Delay(float delay_s);

/** @brief 忙等待指定微秒数，仅用于外设时序。 */
void DWT_DelayUs(uint32_t delay_us);

/** @brief 兼容原有调用命名。 */
#define DWT_Delay_us(delay_us_) DWT_DelayUs((uint32_t)(delay_us_))

#endif /* BSP_DWT_H */
