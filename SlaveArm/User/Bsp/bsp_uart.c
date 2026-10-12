/** @file bsp_uart.c @brief 按本板职责分发 UART 事件，USART10 专用于板间转发。 */
#include "bsp_uart.h"
#include "board_link.h"
#include "rc_system.h"

void BSP_USART_Init(void)
{
    BoardLink_Start();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == &huart10)
    {
        BoardLink_OnRxEvent(size);
        return;
    }
    if (huart == &huart5)
    {
        RcSystem_UartRxEvent(huart, size);
    }
    /* UART7 仅保留 VOFA TX。 */
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart10)
    {
        BoardLink_OnTxComplete();
        return;
    }
    RcSystem_UartTxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart10)
    {
        BoardLink_OnError();
        return;
    }
    RcSystem_UartError(huart);
}
