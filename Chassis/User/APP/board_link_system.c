/**
 * @file board_link_system.c
 * @brief USART10 DMA → 字节环形缓冲 → 流帧解析 → 类型快照发布。
 * @note 保留快照中的 rc_lost；本轮不加入底盘的板间超时/重连策略。
 */
#include "board_link_system.h"

#include "board_link_payload.h"
#include "board_link_protocol.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include <string.h>

#define BOARD_LINK_SYSTEM_DMA_RX_BYTES 256U
#define BOARD_LINK_SYSTEM_RX_RING_BYTES 1024U
#define BOARD_LINK_SYSTEM_TX_PERIOD_MS 5U
#define BOARD_LINK_SYSTEM_DMA_ALIGN __attribute__((aligned(32)))

/* 底盘遥控快照仅由 BoardLink 解码发布，上电保持离线初值。 */
Remote_Info_t remote_ctrl = {.online_cnt = 0xFAU, .rc_lost = true};

BoardLinkSystem_t g_board_link_system;

/** @brief ISR 维护接收及 DMA 状态，InputTask 管理流解析和发送调度。 */
static struct
{
    /* DMA1 缓冲分别保持 32 字节对齐；链接后核对位于 AXI SRAM，不使用 DTCM。 */
    uint8_t dma_rx[BOARD_LINK_SYSTEM_DMA_RX_BYTES] BOARD_LINK_SYSTEM_DMA_ALIGN;
    uint8_t dma_tx[BOARD_LINK_FRAME_MAX] BOARD_LINK_SYSTEM_DMA_ALIGN;

    /* 字节环形缓冲及任务/中断共享状态；同步仍由临界区和内存屏障保证。 */
    uint8_t rx_ring[BOARD_LINK_SYSTEM_RX_RING_BYTES];
    volatile uint16_t rx_write;
    volatile uint16_t rx_read;
    volatile uint32_t rx_overrun;
    volatile bool rx_restart;
    volatile bool tx_busy;

    /* DMA 接收位置、发送调度及流解析状态。 */
    uint16_t dma_position;
    uint16_t tx_sequence;
    uint32_t last_tx_ms;
    BoardLinkParser_t parser;
} g_board_link_io = {
    /* 首次 Process 启动接收；其余成员由静态初始化清零。 */
    .rx_restart = true,
};

void BoardLinkSystem_Start(void)
{
    HAL_StatusTypeDef status;

    if (HAL_UART_AbortReceive(&huart10) != HAL_OK)
    {
        g_board_link_io.rx_restart = true;
        return;
    }
    g_board_link_io.dma_position = 0U;
    /* 消费者不丢弃已排队数据；错误后的 parser 在 owner 上下文重置。 */
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart10, g_board_link_io.dma_rx,
                                         BOARD_LINK_SYSTEM_DMA_RX_BYTES);
    if (status == HAL_OK)
    {
        /* 保留 HT/TC 回调，持续字节流无 IDLE 时也能在绕回前搬运数据。 */
        g_board_link_io.rx_restart = false;
    }
    else
    {
        g_board_link_io.rx_restart = true;
    }
}

void BoardLinkSystem_OnRxEvent(uint16_t size)
{
    uint16_t position;
    uint16_t next;

    if ((size > BOARD_LINK_SYSTEM_DMA_RX_BYTES) || (size == g_board_link_io.dma_position))
    {
        return;
    }
    position = g_board_link_io.dma_position;
    /* TC 给出 size=256，随后 IDLE/HT 从缓冲起点继续；重复位置不再搬运。 */
    while (position != size)
    {
        if (position == BOARD_LINK_SYSTEM_DMA_RX_BYTES)
        {
            position = 0U;
        }
        if (position == size)
        {
            break;
        }
        next = (uint16_t)((g_board_link_io.rx_write + 1U) % BOARD_LINK_SYSTEM_RX_RING_BYTES);
        if (next == g_board_link_io.rx_read)
        {
            g_board_link_io.rx_overrun++;
            break;
        }
        g_board_link_io.rx_ring[g_board_link_io.rx_write] = g_board_link_io.dma_rx[position++];
        __DMB();
        g_board_link_io.rx_write = next;
    }
    g_board_link_io.dma_position = size;
}

void BoardLinkSystem_OnTxComplete(void)
{
    g_board_link_io.tx_busy = false;
}

void BoardLinkSystem_OnError(void)
{
    g_board_link_io.rx_restart = true;
    /* 不复用仍被 TX DMA 使用的缓冲；由 HAL 完成回调或 owner Abort 释放。 */
}

void BoardLinkSystem_Process(uint32_t now_ms)
{
    BoardLinkFrame_t frame;
    uint8_t payload[BOARD_LINK_INS_BYTES];
    size_t tx_bytes;
    uint16_t payload_bytes;
    uint8_t type;
    uint8_t byte;
    bool encoded;
    uint16_t receive_end;

    if (g_board_link_io.rx_restart)
    {
        /* UART/DMA 错误后从新流边界开始；RX 和 TX 的释放由各自 HAL 状态确认。 */
        taskENTER_CRITICAL();
        g_board_link_io.rx_read = g_board_link_io.rx_write;
        g_board_link_io.parser.used = 0U;
        taskEXIT_CRITICAL();
        BoardLinkSystem_Start();
    }
    if (g_board_link_system.rx_overrun_count != g_board_link_io.rx_overrun)
    {
        taskENTER_CRITICAL();
        g_board_link_system.rx_overrun_count = g_board_link_io.rx_overrun;
        g_board_link_io.rx_read = g_board_link_io.rx_write;
        g_board_link_io.parser.used = 0U;
        taskEXIT_CRITICAL();
    }
    /* 固定本次消费终点，ISR 新入队的数据留下一周期，避免持续流拖长 owner。 */
    receive_end = g_board_link_io.rx_write;
    while (g_board_link_io.rx_read != receive_end)
    {
        __DMB();
        byte = g_board_link_io.rx_ring[g_board_link_io.rx_read];
        g_board_link_io.rx_read =
            (uint16_t)((g_board_link_io.rx_read + 1U) % BOARD_LINK_SYSTEM_RX_RING_BYTES);
        if (!BoardLinkProtocol_PushByte(&g_board_link_io.parser, byte, &frame))
        {
            continue;
        }
        Remote_Info_t received_remote;
        if ((frame.type == BOARD_LINK_MESSAGE_REMOTE) &&
            BoardLinkPayload_DecodeRemote(frame.payload, frame.length, &received_remote))
        {
            taskENTER_CRITICAL();
            remote_ctrl = received_remote;
            taskEXIT_CRITICAL();
            g_board_link_system.remote_valid = true;
            g_board_link_system.remote_sequence = frame.sequence;
            g_board_link_system.last_remote_rx_ms = now_ms;
        }
    }
    if ((uint32_t)(now_ms - g_board_link_io.last_tx_ms) < BOARD_LINK_SYSTEM_TX_PERIOD_MS)
    {
        return;
    }
    g_board_link_io.last_tx_ms = now_ms;
    if (g_board_link_io.tx_busy)
    {
        if (huart10.gState != HAL_UART_STATE_READY)
        {
            /* 忙时丢弃本次快照，下一周期发送最新值，不覆盖 DMA 负载。 */
            g_board_link_system.tx_failure_count++;
            return;
        }
        g_board_link_io.tx_busy = false;
    }
    taskENTER_CRITICAL();
    encoded = BoardLinkPayload_EncodeIns(&INS, payload, sizeof(payload));
    payload_bytes = BOARD_LINK_INS_BYTES;
    type = BOARD_LINK_MESSAGE_INS;
    taskEXIT_CRITICAL();
    if (!encoded)
    {
        g_board_link_system.tx_failure_count++;
        return;
    }
    tx_bytes = BoardLinkProtocol_Encode(type, g_board_link_io.tx_sequence,
                                       payload, payload_bytes,
                                       g_board_link_io.dma_tx, sizeof(g_board_link_io.dma_tx));
    g_board_link_io.tx_busy = true;
    if ((tx_bytes == 0U) ||
        (HAL_UART_Transmit_DMA(&huart10, g_board_link_io.dma_tx, (uint16_t)tx_bytes) != HAL_OK))
    {
        g_board_link_io.tx_busy = false;
        g_board_link_system.tx_failure_count++;
    }
    else
    {
        g_board_link_io.tx_sequence++;
    }
}
