/** @file bsp_uart.c @brief 按本板职责分发 UART 事件，USART10 专用于板间转发。 */
#include "bsp_uart.h"
#include "board_link_system.h"

void BSP_USART_Init(void)
{
    BoardLinkSystem_Start();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == &huart10)
    {
        BoardLinkSystem_OnRxEvent(size);
        return;
    }
    /* UART7 仅保留 VOFA TX，底盘没有其他串口接收消费者。 */
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart10)
    {
        BoardLinkSystem_OnTxComplete();
        return;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart10)
    {
        BoardLinkSystem_OnError();
        return;
    }
}
