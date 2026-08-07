/**
 ******************************************************************************
 * @file    bsp_uart.c
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   UART通信功能函数
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 注意初始化BSP_USART_Init函数
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "bsp_uart.h"
#include "rc_task.h"
#include "Unitree_Motor.h"
/* Defines ----------------------------------------------------------------- */

/* Global variable --------------------------------------------------------- */

/* Static Fun -------------------------------------------------------------- */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength);
static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

/* Functions --------------------------------------------------------------- */
/**
 * @brief  BSP层USART初始化
 * @param  无
 * @return 无
 * @note   无
 */
void BSP_USART_Init(void){
	USART_RxDMA_MultiBuffer_Init(&huart2,  (uint32_t *)Unitree_MultiRx_Buf[0], (uint32_t *)Unitree_MultiRx_Buf[1], UNITREE_RX_BUF_LEN);
}

/* Private functions ------------------------------------------------------- */
/**
  * @brief  初始化多缓冲DMA传输并使能中断
  * @param  huart: UART句柄指针
  * @param  DstAddress: 目标内存缓冲区地址指针
  * @param  SecondMemAddress: 第二个内存缓冲区地址指针（用于多缓冲传输）
  * @param  DataLength: 要传输的数据长度
  * @return 无
  * @note   无
  */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	huart->RxXferSize    = DataLength * 2;

	SET_BIT(huart->Instance->CR3,USART_CR3_DMAR); 	// 使能DMA接收
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 		// 使能空闲中断
	/* 禁用DMA */
	do{
		__HAL_DMA_DISABLE(huart->hdmarx);
	}while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);
	/* 配置DMA */ 
	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;
	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;
	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;
	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;
	/* 使能双缓冲区模式 */ 
	SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);
	/* 使能DMA */ 
	__HAL_DMA_ENABLE(huart->hdmarx);	
}

/**
  * @brief  用户USART2接收事件回调 (Unitree电机)
  * @param  huart: UART句柄
  * @param  Size: 接收缓冲区中可用数据的数量
  * @return 无
  * @note   无
  */
static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){
	/* 当前使用的内存缓冲区是内存0 */
	if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET){
		/* 禁用DMA */
		__HAL_DMA_DISABLE(huart->hdmarx);
		/* 切换内存0到内存1 */
		((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
		/* 重置接收计数 */
		__HAL_DMA_SET_COUNTER(huart->hdmarx,UNITREE_RX_BUF_LEN*2);
		/* 判断大小是否等于接收数据的长度 */
		if(Size == UNITREE_RX_BUF_LEN)
		{
			/* 内存0数据解包 */
			Uintree_RxInfo_Unpack(Unitree_MultiRx_Buf[0],&Unitree_Rx_Info);
		}
	}
	/* 当前使用的内存缓冲区是内存1 */
	else{
		/* 禁用DMA */
		__HAL_DMA_DISABLE(huart->hdmarx);
		/* 切换内存1到内存0 */
		((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
		/* 重置接收计数 */
		__HAL_DMA_SET_COUNTER(huart->hdmarx,UNITREE_RX_BUF_LEN*2);
		if(Size == UNITREE_RX_BUF_LEN)
		{
			/* 内存1数据解包 */
			Uintree_RxInfo_Unpack(Unitree_MultiRx_Buf[1],&Unitree_Rx_Info);
		}		
	}
	/* 使能DMA */
	__HAL_DMA_ENABLE(huart->hdmarx);
}

/* Interrupt functions ----------------------------------------------------- */
/**
  * @brief  接收事件回调
  * @param  huart: UART句柄
  * @param  Size: 接收缓冲区中可用数据的数量
  * @return 无
  * @note   在高级接收服务使用后调用的Rx事件通知
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
	if(huart == &huart5)
	{
		RC_Task_UartRxEvent(huart,Size);
	} 
	else if(huart == &huart2)
	{
		USER_USART2_RxHandler(huart,Size);
	}
}

/**
  * @brief  UART发送完成回调
  * @param  huart: UART句柄
  * @return 无
  * @note   UART5事件由rc_task释放DbgDisp异步发送缓冲区
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	RC_Task_UartTxComplete(huart);
}

/**
  * @brief  UART错误回调
  * @param  huart: UART句柄
  * @return 无
  * @note   UART5事件由rc_task请求在任务上下文恢复DMA接收
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	RC_Task_UartError(huart);
}

/* ------------------------------------------------------------------------- */
