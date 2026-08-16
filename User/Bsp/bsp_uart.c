/**
 ******************************************************************************
 * @file    bsp_uart.c
 * @brief   UART 中断与 DMA 传输分发。
 ******************************************************************************
 */

#include "bsp_uart.h"

#include "arm_task.h"
#include "rc_system.h"
#include "usart.h"

#include <stdbool.h>

#define BSP_USART10_RX_ALIGN __attribute__((aligned(32)))
#define BSP_USART10_RECOVERY_RETRY_MS 5U

static uint8_t g_usart10_rx_buffer[UNITREE_RX_BUF_LEN]
    BSP_USART10_RX_ALIGN;
static volatile bool g_usart10_restart_pending = true;
static uint32_t g_usart10_next_restart_time_ms = 0U;

static HAL_StatusTypeDef bsp_usart10_start_receive(void)
{
    HAL_StatusTypeDef result;

    if ((huart10.RxState != HAL_UART_STATE_READY) ||
        HAL_IS_BIT_SET(huart10.Instance->CR3, USART_CR3_DMAR))
    {
        if (HAL_UART_AbortReceive(&huart10) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    result = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart10,
        g_usart10_rx_buffer,
        sizeof(g_usart10_rx_buffer));
    if (result == HAL_OK)
    {
        /* 固定 16 字节协议只处理整帧完成和 IDLE，不需要半传输回调。 */
        __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
        g_usart10_restart_pending = false;
        g_usart10_next_restart_time_ms = 0U;
    }

    return result;
}

void BSP_USART_Init(void)
{
    (void)bsp_usart10_start_receive();
}

void BSP_USART10_RecoverRxIfPending(void)
{
    uint32_t now_ms;

    if (!g_usart10_restart_pending)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if ((int32_t)(now_ms - g_usart10_next_restart_time_ms) < 0)
    {
        return;
    }

    if (bsp_usart10_start_receive() != HAL_OK)
    {
        /* HAL/DMA 暂时忙时由 ArmTask 延后重试，禁止在任务内阻塞等待。 */
        g_usart10_next_restart_time_ms =
            now_ms + BSP_USART10_RECOVERY_RETRY_MS;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == &huart5)
    {
        RcSystem_UartRxEvent(huart, size);
        return;
    }

    if (huart != &huart10)
    {
        return;
    }

    if (size == UNITREE_RX_BUF_LEN)
    {
        ArmTask_OnUsart10Rx(g_usart10_rx_buffer, size);
    }
    else if ((HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE) &&
             (size > 0U))
    {
        /* 从半帧启动时丢弃本段，在任务上下文重新对齐下一帧。 */
        g_usart10_restart_pending = true;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    RcSystem_UartTxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    RcSystem_UartError(huart);
    if (huart == &huart10)
    {
        g_usart10_restart_pending = true;
    }
}
