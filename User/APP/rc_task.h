/**
 ******************************************************************************
 * @file    rc_task.h
 * @version V1.0.0
 * @date    2026.07.19
 * @brief   UART5 DBUS 双缓冲 DMA 接收与调试信息发送接口
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * UART5 RX 由本模块独占并用于 18 字节 DBUS 帧接收。
 * * UART5 TX 由 DbgDisp 使用普通模式 DMA 回传遥控状态，不占用 RX DMA。
 * * 任务接口每 50 ms 调用一次；DMA 完成回调仍按每帧实时运行。
 ******************************************************************************
 */

#ifndef RC_TASK_H
#define RC_TASK_H

/* Includes ----------------------------------------------------------------- */
#include "DbgDisp.h"
#include "Remote_Control.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

/* Functions ---------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 UART5 DBUS 硬件双缓冲 DMA 接收
 * @return HAL_OK 表示接收已启动，否则返回 HAL_ERROR
 */
HAL_StatusTypeDef RC_Task_Init(void);

/**
 * @brief 执行 50 ms 周期的离线检测、错误恢复和 DbgDisp 状态回传
 * @return 无
 */
void RC_Task_Process(void);

/**
 * @brief 使用 rcTask 内部的 UART5 DbgDisp 实例发送名称/数值对
 * @param name 第一个变量名
 * @param value 第一个变量值
 * @param ... 后续名称/数值对；由 RC_Task_DebugSend() 自动追加结束标记
 * @return DbgDisp 发送结果；上一帧 DMA 未结束时返回 DBG_BUSY
 * @note 不可在中断服务函数中调用。
 */
DbgRet RC_Task_DebugSendArgs(const char *name, double value, ...);

#define RC_Task_DebugSend(...) \
    RC_Task_DebugSendArgs(__VA_ARGS__, (const char *)NULL)

/**
 * @brief 查询是否已接收到有效且未超时的 DBUS 帧
 * @return 非零表示遥控数据可用
 */
uint8_t RC_Task_IsReady(void);

/**
 * @brief 获取累计有效 DBUS 帧数
 */
uint32_t RC_Task_GetValidFrameCount(void);

/**
 * @brief 获取累计拒绝的错误/错位 DBUS 帧数
 */
uint32_t RC_Task_GetRejectedFrameCount(void);

/**
 * @brief 转发 UART5 Receive-to-Idle 事件以完成错位重同步
 */
void RC_Task_UartRxEvent(UART_HandleTypeDef *huart, uint16_t size);

/**
 * @brief 转发 UART5 错误事件以释放 TX 并请求 RX 重启
 */
void RC_Task_UartError(UART_HandleTypeDef *huart);

/**
 * @brief 转发 UART5 发送完成事件以释放 DbgDisp 发送缓冲区
 */
void RC_Task_UartTxComplete(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

/* -------------------------------------------------------------------------- */
#endif /* RC_TASK_H */
