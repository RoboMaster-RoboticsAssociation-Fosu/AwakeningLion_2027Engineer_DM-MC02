/**
 ******************************************************************************
 * @file    rc_system.c
 * @version V1.0.0
 * @date    2026.07.19
 * @brief   UART5 DBUS 双缓冲 DMA 接收与调试信息发送实现
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * STM32H723 DMA1 无法访问 DTCM，链接后需确认 DMA 缓冲位于 AXI SRAM。
 * * DMA/中断侧只做定长帧校验和快照发布；50 ms 任务侧负责离线检测、
 *   错误恢复和调试显示发送，不降低底层接收实时性。
 ******************************************************************************
 */

/* Includes ----------------------------------------------------------------- */
#include "rc_system.h"
#include "DbgDisp.h"
#include "FreeRTOS.h"
#include "Remote_Control.h"
#include "bsp_dwt.h"
#include "task.h"
#include "usart.h"
#include <string.h>

/* Defines ------------------------------------------------------------------ */
#define RC_DMA_BUFFER_COUNT       2U
#define RC_DBUS_CHANNEL_MIN       364U
#define RC_DBUS_CHANNEL_MAX       1684U
#define RC_OFFLINE_TIMEOUT_MS     200U
#define RC_IW_UP_THRESHOLD        (-330)
#define RC_IW_DOWN_THRESHOLD      330

#define RC_DMA_BUFFER_ALIGN __attribute__((aligned(32)))

/* Global variable --------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */
/* 单例整体对齐使首成员 debug.tx_buffer 满足 UART TX DMA 对齐要求。 */
static RcSystem_t g_rc_system RC_DMA_BUFFER_ALIGN = {
    .debug = {
        .display = {
            .config = {
                .huart = &huart5,
                .tx_mode = DBG_TX_DMA,
                .timeout_ms = DBG_TIMEOUT_MS,
                .tx_buf = g_rc_system.debug.tx_buffer,
                .tx_cap = (uint16_t)sizeof(g_rc_system.debug.tx_buffer),
                .obj_buf = g_rc_system.debug.objects,
                .obj_cap =
                    (uint8_t)(sizeof(g_rc_system.debug.objects) /
                              sizeof(g_rc_system.debug.objects[0])),
            },
        },
    },
    .receive = {
        .restart_pending = 1U,
    },
};

static uint8_t g_rc_dma_rx_buffer[RC_DMA_BUFFER_COUNT]
                                  [SBUS_RX_BUF_LEN]
    RC_DMA_BUFFER_ALIGN;

/* Private function prototypes --------------------------------------------- */
static HAL_StatusTypeDef RC_StartDmaDoubleBuffer(void);
static void RC_ProcessDmaFrame(const uint8_t frame[SBUS_RX_BUF_LEN]);
static uint8_t RC_IsDbusFrameValid(
    const uint8_t frame[SBUS_RX_BUF_LEN]);
static void RC_DmaMemory0Complete(DMA_HandleTypeDef *hdma);
static void RC_DmaMemory1Complete(DMA_HandleTypeDef *hdma);
static void RC_DmaError(DMA_HandleTypeDef *hdma);
static void RC_StopAndRequestRestart(UART_HandleTypeDef *huart);
static void RC_UpdateOfflineState(uint32_t now_ms);
static void RC_ResetEdgeState(void);
static void RC_UpdateInputEdges(const Remote_Info_t *remote);
static uint32_t RC_GetSwitchEdge(uint8_t previous, uint8_t current);
static uint32_t RC_GetIwEdges(int16_t previous, int16_t current);

/* Functions --------------------------------------------------------------- */
void RcSystem_Process(void)
{
    const uint32_t now_ms = DWT_GetTimeMs();

    if (g_rc_system.receive.restart_pending != 0U)
    {
        (void)RC_StartDmaDoubleBuffer();
    }

    RC_UpdateOfflineState(now_ms);
}

DbgRet RcSystem_DebugSendArgs(const char *name, double value, ...)
{
    DbgRet result = DBG_OK;
    va_list arguments;

    va_start(arguments, value);
    result = Dbg_SendVArgs(&g_rc_system.debug.display,
                           name,
                           value,
                           arguments);
    va_end(arguments);

    return result;
}

uint8_t RcSystem_IsReady(void)
{
    return ((g_rc_system.receive.ready != 0U) &&
            (remote_ctrl.rc_lost == false))
               ? 1U
               : 0U;
}

uint32_t RcSystem_GetValidFrameCount(void)
{
    return g_rc_system.diagnostics.valid_frame_count;
}

uint32_t RcSystem_GetRejectedFrameCount(void)
{
    return g_rc_system.diagnostics.rejected_frame_count;
}

void RcSystem_UartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart5)
    {
        return;
    }

    /*
     * 满 18 字节的块由 DMA M0/M1 完成回调处理。IDLE 时只有不足一帧，说明
     * DMA 从帧中间开始，停止并在下一个 50 ms 任务周期从内存 0 重新对齐。
     */
    if ((HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE) &&
        (size > 0U) && (size < SBUS_RX_BUF_LEN))
    {
        RC_StopAndRequestRestart(huart);
    }
}

void RcSystem_UartError(UART_HandleTypeDef *huart)
{
    if (huart != &huart5)
    {
        return;
    }

    Dbg_TxDone(&g_rc_system.debug.display, huart);
    RC_StopAndRequestRestart(huart);
}

void RcSystem_UartTxComplete(UART_HandleTypeDef *huart)
{
    Dbg_TxDone(&g_rc_system.debug.display, huart);
}

/* Private functions ------------------------------------------------------- */
static HAL_StatusTypeDef RC_StartDmaDoubleBuffer(void)
{
    UART_HandleTypeDef *huart = &huart5;
    DMA_HandleTypeDef *hdma = huart->hdmarx;
    HAL_StatusTypeDef status = HAL_ERROR;

    if (hdma == NULL)
    {
        return HAL_ERROR;
    }

    if ((huart->RxState != HAL_UART_STATE_READY) ||
        HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR))
    {
        if (HAL_UART_AbortReceive(huart) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    memset(g_rc_dma_rx_buffer, 0, sizeof(g_rc_dma_rx_buffer));
    __HAL_UART_CLEAR_FLAG(huart,
                          UART_CLEAR_OREF | UART_CLEAR_NEF |
                              UART_CLEAR_PEF | UART_CLEAR_FEF |
                              UART_CLEAR_IDLEF);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

    taskENTER_CRITICAL();

    huart->pRxBuffPtr = g_rc_dma_rx_buffer[0];
    huart->RxXferSize = SBUS_RX_BUF_LEN;
    huart->RxXferCount = SBUS_RX_BUF_LEN;
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_BUSY_RX;
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart->RxEventType = HAL_UART_RXEVENT_TC;

    hdma->XferCpltCallback = RC_DmaMemory0Complete;
    hdma->XferHalfCpltCallback = NULL;
    hdma->XferM1CpltCallback = RC_DmaMemory1Complete;
    hdma->XferM1HalfCpltCallback = NULL;
    hdma->XferErrorCallback = RC_DmaError;
    hdma->XferAbortCallback = NULL;

    /* 每次错误恢复都从 M0 开始，防止旧 CT 状态把首帧交给错误缓冲区。 */
    CLEAR_BIT(((DMA_Stream_TypeDef *)hdma->Instance)->CR,
              DMA_SxCR_DBM | DMA_SxCR_CT);
    status = HAL_DMAEx_MultiBufferStart_IT(
        hdma,
        (uint32_t)&huart->Instance->RDR,
        (uint32_t)g_rc_dma_rx_buffer[0],
        (uint32_t)g_rc_dma_rx_buffer[1],
        SBUS_RX_BUF_LEN);

    if (status == HAL_OK)
    {
        if (huart->Init.Parity != UART_PARITY_NONE)
        {
            ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);
        }
        ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_EIE);
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);
        ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
        __DMB();
        ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
        g_rc_system.receive.restart_pending = 0U;
    }
    else
    {
        huart->ErrorCode |= HAL_UART_ERROR_DMA;
        huart->RxState = HAL_UART_STATE_READY;
        huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
        g_rc_system.receive.restart_pending = 1U;
    }

    taskEXIT_CRITICAL();

    return status;
}

static void RC_ProcessDmaFrame(const uint8_t frame[SBUS_RX_BUF_LEN])
{
    __DMB();
    if (RC_IsDbusFrameValid(frame) == 0U)
    {
        g_rc_system.diagnostics.rejected_frame_count++;
        return;
    }

    SBUS_TO_RC(frame, &remote_ctrl);
    RC_UpdateInputEdges(&remote_ctrl);
    g_rc_system.receive.last_valid_time_ms = DWT_GetTimeMs();
    g_rc_system.receive.has_valid_frame = 1U;
    g_rc_system.receive.ready = 1U;
    g_rc_system.diagnostics.valid_frame_count++;
    __DMB();
}

static uint8_t RC_IsDbusFrameValid(
    const uint8_t frame[SBUS_RX_BUF_LEN])
{
    uint16_t channel[5] = {0U};
    uint8_t switch_left = 0U;
    uint8_t switch_right = 0U;

    if (frame == NULL)
    {
        return 0U;
    }

    channel[0] = ((uint16_t)frame[0] |
                  ((uint16_t)frame[1] << 8U)) &
                 0x07FFU;
    channel[1] = (((uint16_t)frame[1] >> 3U) |
                  ((uint16_t)frame[2] << 5U)) &
                 0x07FFU;
    channel[2] = (((uint16_t)frame[2] >> 6U) |
                  ((uint16_t)frame[3] << 2U) |
                  ((uint16_t)frame[4] << 10U)) &
                 0x07FFU;
    channel[3] = (((uint16_t)frame[4] >> 1U) |
                  ((uint16_t)frame[5] << 7U)) &
                 0x07FFU;
    channel[4] = ((uint16_t)frame[16] |
                  ((uint16_t)frame[17] << 8U)) &
                 0x07FFU;
    switch_left = (frame[5] >> 6U) & 0x03U;
    switch_right = (frame[5] >> 4U) & 0x03U;

    if ((channel[0] < RC_DBUS_CHANNEL_MIN) ||
        (channel[0] > RC_DBUS_CHANNEL_MAX) ||
        (channel[1] < RC_DBUS_CHANNEL_MIN) ||
        (channel[1] > RC_DBUS_CHANNEL_MAX) ||
        (channel[2] < RC_DBUS_CHANNEL_MIN) ||
        (channel[2] > RC_DBUS_CHANNEL_MAX) ||
        (channel[3] < RC_DBUS_CHANNEL_MIN) ||
        (channel[3] > RC_DBUS_CHANNEL_MAX) ||
        (channel[4] < RC_DBUS_CHANNEL_MIN) ||
        (channel[4] > RC_DBUS_CHANNEL_MAX) ||
        (switch_left < RC_SW_UP) || (switch_left > RC_SW_MID) ||
        (switch_right < RC_SW_UP) || (switch_right > RC_SW_MID))
    {
        return 0U;
    }

    return 1U;
}

static void RC_DmaMemory0Complete(DMA_HandleTypeDef *hdma)
{
    if (hdma == huart5.hdmarx)
    {
        RC_ProcessDmaFrame(g_rc_dma_rx_buffer[0]);
    }
}

static void RC_DmaMemory1Complete(DMA_HandleTypeDef *hdma)
{
    if (hdma == huart5.hdmarx)
    {
        RC_ProcessDmaFrame(g_rc_dma_rx_buffer[1]);
    }
}

static void RC_DmaError(DMA_HandleTypeDef *hdma)
{
    UART_HandleTypeDef *huart = NULL;

    if (hdma != huart5.hdmarx)
    {
        return;
    }

    huart = (UART_HandleTypeDef *)hdma->Parent;
    ATOMIC_CLEAR_BIT(huart->Instance->CR1,
                     USART_CR1_PEIE | USART_CR1_IDLEIE);
    ATOMIC_CLEAR_BIT(huart->Instance->CR3,
                     USART_CR3_DMAR | USART_CR3_EIE);
    huart->ErrorCode |= HAL_UART_ERROR_DMA;
    huart->RxState = HAL_UART_STATE_READY;
    huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
    g_rc_system.receive.ready = 0U;
    g_rc_system.receive.restart_pending = 1U;
}

static void RC_StopAndRequestRestart(UART_HandleTypeDef *huart)
{
    g_rc_system.receive.ready = 0U;
    g_rc_system.receive.restart_pending = 1U;
    (void)HAL_UART_AbortReceive(huart);
}

static void RC_UpdateOfflineState(uint32_t now_ms)
{
    uint8_t timed_out = 0U;

    taskENTER_CRITICAL();
    timed_out =
        ((g_rc_system.receive.has_valid_frame == 0U) ||
         ((uint32_t)(now_ms - g_rc_system.receive.last_valid_time_ms) >
          RC_OFFLINE_TIMEOUT_MS))
            ? 1U
            : 0U;
    if (timed_out != 0U)
    {
        memset(&remote_ctrl, 0, sizeof(remote_ctrl));
        remote_ctrl.rc_lost = true;
        RC_ResetEdgeState();
        g_rc_system.receive.ready = 0U;
    }
    else
    {
        Remote_Active_Detect(&remote_ctrl);
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief 清除输入位置历史并废弃所有任务尚未读取的旧边沿。
 * @note 初始化和遥控器离线时调用；重连后的第一帧只建立基准，不生成伪边沿。
 */
static void RC_ResetEdgeState(void)
{
    uint8_t switch_index;

    g_rc_system.edge.valid = 0U;
    g_rc_system.edge.previous_iw = 0U;
    for (switch_index = 0U;
         switch_index < (uint8_t)REMOTE_SWITCH_COUNT;
         ++switch_index)
    {
        g_rc_system.edge.previous_switch[switch_index] = 0U;
    }
    Remote_Edge_Reset();
}

/**
 * @brief 对每个有效 DBUS 帧解析 SW 和 iw 边沿并发布一个原子批次。
 * @note 在 DMA 完成中断路径调用，避免 50 ms rcTask 周期漏掉快速切换。
 */
static void RC_UpdateInputEdges(const Remote_Info_t *remote)
{
    uint32_t switch_edges[REMOTE_SWITCH_COUNT] = {
        RC_SWITCH_EDGE_NONE,
        RC_SWITCH_EDGE_NONE,
    };
    uint32_t iw_edges;
    uint8_t switch_index;

    if (remote == NULL)
    {
        return;
    }

    if (g_rc_system.edge.valid == 0U)
    {
        for (switch_index = 0U;
             switch_index < (uint8_t)REMOTE_SWITCH_COUNT;
             ++switch_index)
        {
            g_rc_system.edge.previous_switch[switch_index] =
                remote->rc.s[switch_index];
        }
        g_rc_system.edge.previous_iw = remote->rc.iw;
        g_rc_system.edge.valid = 1U;
        return;
    }

    for (switch_index = 0U;
         switch_index < (uint8_t)REMOTE_SWITCH_COUNT;
         ++switch_index)
    {
        const uint8_t current_switch =
            remote->rc.s[switch_index];

        switch_edges[switch_index] =
            RC_GetSwitchEdge(
                g_rc_system.edge.previous_switch[switch_index],
                current_switch);
        g_rc_system.edge.previous_switch[switch_index] = current_switch;
    }

    iw_edges = RC_GetIwEdges(g_rc_system.edge.previous_iw,
                             remote->rc.iw);
    g_rc_system.edge.previous_iw = remote->rc.iw;
    Remote_Edge_PublishFromISR(
        switch_edges[REMOTE_SWITCH_SW1],
        switch_edges[REMOTE_SWITCH_SW2],
        iw_edges);
}

/**
 * @brief 把 SW 位置变化映射为进入当前目标位置的边沿位。
 */
static uint32_t RC_GetSwitchEdge(uint8_t previous, uint8_t current)
{
    if (previous == current)
    {
        return RC_SWITCH_EDGE_NONE;
    }

    if (current == RC_SW_UP)
    {
        return RC_SWITCH_EDGE_TO_UP;
    }
    if (current == RC_SW_MID)
    {
        return RC_SWITCH_EDGE_TO_MID;
    }
    if (current == RC_SW_DOWN)
    {
        return RC_SWITCH_EDGE_TO_DOWN;
    }

    return RC_SWITCH_EDGE_NONE;
}

/**
 * @brief 比较连续两帧 iw 有符号值并返回跨越 -330/+330 的全部边沿位。
 * @note 单帧从上区直接跳到下区时会同时产生 LEAVE_UP 和 ENTER_DOWN。
 */
static uint32_t RC_GetIwEdges(int16_t previous, int16_t current)
{
    uint32_t edges = RC_IW_EDGE_NONE;

    if ((previous > RC_IW_UP_THRESHOLD) &&
        (current <= RC_IW_UP_THRESHOLD))
    {
        edges |= RC_IW_EDGE_ENTER_UP;
    }
    else if ((previous <= RC_IW_UP_THRESHOLD) &&
             (current > RC_IW_UP_THRESHOLD))
    {
        edges |= RC_IW_EDGE_LEAVE_UP;
    }

    if ((previous < RC_IW_DOWN_THRESHOLD) &&
        (current >= RC_IW_DOWN_THRESHOLD))
    {
        edges |= RC_IW_EDGE_ENTER_DOWN;
    }
    else if ((previous >= RC_IW_DOWN_THRESHOLD) &&
             (current < RC_IW_DOWN_THRESHOLD))
    {
        edges |= RC_IW_EDGE_LEAVE_DOWN;
    }

    return edges;
}

/* -------------------------------------------------------------------------- */
