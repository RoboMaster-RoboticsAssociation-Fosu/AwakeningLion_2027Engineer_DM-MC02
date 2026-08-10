/**
 ******************************************************************************
 * @file    rc_system.h
 * @version V1.0.0
 * @date    2026.07.19
 * @brief   UART5 DBUS 遥控接收系统接口
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * UART5 RX 由本模块独占并用于 18 字节 DBUS 帧接收。
 * * UART5 TX 由 DbgDisp 使用普通模式 DMA 回传遥控状态，不占用 RX DMA。
 * * RcSystem_Process() 由 InputTask 周期调用；DMA 完成回调仍按每帧实时运行。
 ******************************************************************************
 */

#ifndef RC_SYSTEM_H
#define RC_SYSTEM_H

/* Includes ----------------------------------------------------------------- */
#include "DbgDisp.h"
#include "Remote_Control.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

/* Structs ------------------------------------------------------------------ */
/**
 * @brief RcSystem DBUS 接收与恢复状态。
 * @note rc_system.c 实现专用类型，不作为模块间数据交换接口。
 */
typedef struct
{
    uint8_t ready;
    uint8_t restart_pending;
    uint8_t has_valid_frame;
    uint32_t last_valid_time_ms;
} RcSystemReceiveState_t;

/**
 * @brief RcSystem 拨杆和拨轮边沿历史。
 * @note 由 DMA 完成中断更新；离线时清空，重连首帧只建立比较基准。
 */
typedef struct
{
    uint8_t valid;
    uint8_t previous_switch[REMOTE_SWITCH_COUNT];
    int16_t previous_iw;
} RcSystemEdgeState_t;

/**
 * @brief RcSystem DBUS 帧诊断统计。
 */
typedef struct
{
    uint32_t valid_frame_count;
    uint32_t rejected_frame_count;
} RcSystemDiagnostics_t;

/**
 * @brief RcSystem 调试显示 TX 资源和同步状态。
 * @note rc_system.c 实现专用类型；UART5 RX 由 DBUS 独占，因此这里只保留
 *       DbgDisp 发送实际需要的缓冲区和对象表。
 */
typedef struct
{
    uint8_t tx_buffer[DBG_FRAME_MAX];
    DbgObj_t objects[DBG_MAX_OBJ];
    Dbg_t display;
} RcSystemDebug_t;

/**
 * @brief RcSystem 单例运行状态。
 * @note rc_system.c 是唯一 owner。debug 必须保持为首成员，使其中 TX DMA
 *       缓冲区随单例整体获得 32 字节对齐。
 */
typedef struct
{
    RcSystemDebug_t debug;
    volatile RcSystemReceiveState_t receive;
    volatile RcSystemEdgeState_t edge;
    volatile RcSystemDiagnostics_t diagnostics;
} RcSystem_t;

/* Functions ---------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 执行 50 ms 周期的 DMA 启动/恢复和离线检测
 * @return 无
 */
void RcSystem_Process(void);

/**
 * @brief 使用 RcSystem 内部的 UART5 DbgDisp 实例发送名称/数值对
 * @param name 第一个变量名
 * @param value 第一个变量值
 * @param ... 后续名称/数值对；由 RcSystem_DebugSend() 自动追加结束标记
 * @return DbgDisp 发送结果；上一帧 DMA 未结束时返回 DBG_BUSY
 * @note 仅允许 InputTask owner 线程调用，不可在中断或其他任务中调用。
 */
DbgRet RcSystem_DebugSendArgs(const char *name, double value, ...);

#define RcSystem_DebugSend(...) \
    RcSystem_DebugSendArgs(__VA_ARGS__, (const char *)NULL)

/**
 * @brief 查询是否已接收到有效且未超时的 DBUS 帧
 * @return 非零表示遥控数据可用
 */
uint8_t RcSystem_IsReady(void);

/**
 * @brief 获取累计有效 DBUS 帧数
 */
uint32_t RcSystem_GetValidFrameCount(void);

/**
 * @brief 获取累计拒绝的错误/错位 DBUS 帧数
 */
uint32_t RcSystem_GetRejectedFrameCount(void);

/**
 * @brief 转发 UART5 Receive-to-Idle 事件以完成错位重同步
 */
void RcSystem_UartRxEvent(UART_HandleTypeDef *huart, uint16_t size);

/**
 * @brief 转发 UART5 错误事件以释放 TX 并请求 RX 重启
 */
void RcSystem_UartError(UART_HandleTypeDef *huart);

/**
 * @brief 转发 UART5 发送完成事件以释放 DbgDisp 发送缓冲区
 */
void RcSystem_UartTxComplete(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

/* -------------------------------------------------------------------------- */
#endif /* RC_SYSTEM_H */
