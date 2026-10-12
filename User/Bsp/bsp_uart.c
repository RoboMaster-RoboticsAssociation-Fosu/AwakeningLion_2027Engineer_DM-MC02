/** @file bsp_uart.c @brief 按本板职责分发 UART 事件，USART10 专用于板间转发。 */
#include "bsp_uart.h"
#include "board_link.h"
#include "robot_board_config.h"
#if ROBOT_BOARD_ROLE == ROBOT_BOARD_ARM
#include "rc_system.h"
#endif

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
#if ROBOT_BOARD_ROLE == ROBOT_BOARD_ARM
    if (huart == &huart5)
    {
        RcSystem_UartRxEvent(huart, size);
    }
#endif
    /* UART7 仅保留 VOFA TX；新自定义控制器协议本轮不接收。 */
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart10)
    {
        BoardLink_OnTxComplete();
        return;
    }
#if ROBOT_BOARD_ROLE == ROBOT_BOARD_ARM
    RcSystem_UartTxComplete(huart);
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart10)
    {
        BoardLink_OnError();
        return;
    }
#if ROBOT_BOARD_ROLE == ROBOT_BOARD_ARM
    RcSystem_UartError(huart);
#endif
}
