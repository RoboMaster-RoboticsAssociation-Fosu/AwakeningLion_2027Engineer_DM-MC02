/**
 * @file board_link.h
 * @brief USART10 双向快照转发，InputTask 是唯一解析和发送 owner。
 * @note ISR 只搬运接收字节和维护 DMA 状态。源时间戳不与本板时钟直接比较。
 */
#ifndef BOARD_LINK_H
#define BOARD_LINK_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief 板间链路的接收证据；时间为本板 HAL tick，仅记录已解码消息。 */
typedef struct
{
    bool remote_valid;
    bool ins_valid;
    uint16_t remote_sequence;
    uint16_t ins_sequence;
    uint32_t last_remote_rx_ms;
    uint32_t last_ins_rx_ms;
    uint32_t rx_overrun_count;
    uint32_t tx_failure_count;
} BoardLink_t;

extern BoardLink_t g_board_link;

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 开启 USART10 循环 Receive-to-Idle DMA；启动失败由 Process 重试。 */
void BoardLink_Start(void);
/** @brief InputTask 每 1 ms 调用；解析收到的数据，每 5 ms 发本板快照。 */
void BoardLink_Process(uint32_t now_ms);
/** @brief USART10 RX 事件入口，size 为 DMA 缓冲中的累计写入位置。 */
void BoardLink_OnRxEvent(uint16_t size);
/** @brief USART10 完成/错误回调；不得用于其他 UART。 */
void BoardLink_OnTxComplete(void);
void BoardLink_OnError(void);

#ifdef __cplusplus
}
#endif
#endif /* BOARD_LINK_H */
