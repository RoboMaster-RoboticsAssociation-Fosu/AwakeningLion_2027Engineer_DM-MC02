/**
 ******************************************************************************
 * @file    bsp_can.c
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   CAN通信功能函数
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "bsp_can.h"
#include "fdcan.h"
#include "chassis_task.h"
#include "string.h"

/* Defines ----------------------------------------------------------------- */

/* Global variable --------------------------------------------------------- */
FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

FDCAN_RxHeaderTypeDef RxHeader3;
uint8_t g_Can3RxData[64];

/* Static Fun -------------------------------------------------------------- */

/* Functions --------------------------------------------------------------- */
/**
 * @brief  FDCAN1过滤器和中断配置
 * @param  无
 * @return 无
 * @note   无
 */
void FDCAN1_Config(void)
{
	FDCAN_FilterTypeDef sFilterConfig;
	/* 配置Rx过滤器 */	
	sFilterConfig.IdType = FDCAN_STANDARD_ID; //标准ID
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x00000000;
	sFilterConfig.FilterID2 = 0x00000000;

	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/* 全局过滤器配置 */
	/* 接收到消息ID为标准ID，过滤器不匹配，则拒绝标准ID远程帧 */
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}
	
	/* 激活RX FIFO0新消息通知中断 */
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}
	
  	/* 启动FDCAN模块 */
  	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  	{
  	  Error_Handler();
  	}
}

/**
 * @brief  FDCAN2过滤器和中断配置
 * @param  无
 * @return 无
 * @note   无
 */
void FDCAN2_Config(void)
{
	FDCAN_FilterTypeDef sFilterConfig;
	/* 配置Rx过滤器 */
	sFilterConfig.IdType =  FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 1;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig.FilterID1 = 0x00000000;
	sFilterConfig.FilterID2 = 0x00000000;
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/* 全局过滤器配置 */
	/* 接收到消息ID为标准ID，过滤器不匹配，则拒绝标准ID远程帧 */
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}

	/* 激活RX FIFO1新消息通知中断 */
	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}
	
	/* 启动FDCAN模块 */
	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief  FDCAN3过滤器和中断配置
 * @param  无
 * @return 无
 * @note   无
 */
void FDCAN3_Config(void)
{
	FDCAN_FilterTypeDef sFilterConfig;
	/* 配置Rx过滤器 */
	sFilterConfig.IdType =  FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 2;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x00000000;
	sFilterConfig.FilterID2 = 0x00000000;
	if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/* 全局过滤器配置 */
	/* 接收到消息ID为标准ID，过滤器不匹配，则拒绝标准ID远程帧 */
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}

	/* 激活RX FIFO1新消息通知中断 */
	if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}
	
	/* 启动FDCAN模块 */
	if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief  CAN发送数据
 * @param  hcan: CAN句柄
 * @param  id: 	 CAN ID
 * @param  data: 数据指针
 * @param  len:  数据长度
 * @return 0 表示成功，非零表示参数错误或 HAL 发送失败
 * @note   本函数只发送经典 CAN 帧，因此数据长度不得超过 8 字节。
 */
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;
	HAL_StatusTypeDef hal_status;

	if((hcan == NULL) || (data == NULL) || (len > 8U))
	{
		return 1U;
	}

	TxHeader.Identifier = id;
	TxHeader.IdType =  FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = len;
											
	TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // 比特率切换关闭，仅用于经典CAN
	TxHeader.FDFormat =  FDCAN_CLASSIC_CAN; // 经典CAN
	TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0; // 消息标记

	hal_status = HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data);
	return (uint8_t)hal_status;
}

/* Private functions ------------------------------------------------------- */

/* Interrupt functions ----------------------------------------------------- */
/**
 * @brief  FDCAN RxFifo0回调函数
 * @param  hfdcan: FDCAN句柄
 * @param  RxFifo0ITs: RxFifo0中断标志
 * @return 无
 * @note   在FDCAN1和FDCAN3接收到新消息时调用
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == FDCAN1)
    {
		/* 从RX_FIFO0检索Rx消息 */
		memset(g_Can1RxData, 0, sizeof(g_Can1RxData));	//清空接收缓冲区	
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
			
	}
	else if(hfdcan->Instance == FDCAN3)
	{
		/* 从RX_FIFO0检索Rx消息 */
		memset(g_Can3RxData, 0, sizeof(g_Can3RxData));	// 清空接收缓冲区	
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader3, g_Can3RxData);
			
		switch(RxHeader3.Identifier)
		{
			
			
			
			default: break;
		}
	}
  }
}

/**
 * @brief  FDCAN RxFifo1回调函数
 * @param  hfdcan: FDCAN句柄
 * @param  RxFifo1ITs: RxFifo1中断标志
 * @return 无
 * @note   在FDCAN2接收到新消息时调用
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == FDCAN2)
    {
		/* 从RX_FIFO1检索Rx消息 */
		memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
		Chassis_TaskOnCan2Rx(RxHeader2.Identifier,
							g_Can2RxData,
							RxHeader2.DataLength);
		switch(RxHeader2.Identifier)
		{
//			case Chassis_J4340_Motor_L_RxID:DM_Motor_Info_Update(&Chassis_Ctrl.Joint_Motor[LEFT],g_Can2RxData,RxHeader2.DataLength);break;
//			case Chassis_J4340_Motor_R_RxID:DM_Motor_Info_Update(&Chassis_Ctrl.Joint_Motor[RIGHT],g_Can2RxData,RxHeader2.DataLength);break;				
			default: break;
		}	
    }
  }
}

/* ------------------------------------------------------------------------- */
