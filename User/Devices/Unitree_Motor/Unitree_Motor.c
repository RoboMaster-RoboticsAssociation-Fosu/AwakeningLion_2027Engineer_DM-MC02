/**
 ******************************************************************************
 * @file    Unitree_Motor.c
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   Unitree电机驱动函数
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "Unitree_Motor.h"
#include "CRC.h"
#include "user_lib.h"
#include "user_sys_config.h"
#include "gpio.h"

/* Defines ----------------------------------------------------------------- */
#define CALIBRATION_COUNT 				5u			/* 校准采样次数 */
#define Unitree_RxHEAD 					0xFDEE		/* Unitree接收帧头 */

/* Global variable --------------------------------------------------------- */
__attribute__((section (".AXI_SRAM"))) uint8_t Unitree_MultiRx_Buf[2][UNITREE_RX_BUF_LEN];
Unitree_RxInfo_t Unitree_Rx_Info;

/* Static Fun -------------------------------------------------------------- */
static void Unitree_get_offset(Unitree_Motor_Info_t *Motor);
static void Unitree_Motor_Info_Update(Unitree_Motor_Info_t *Motor, Unitree_RxInfo_t *Unitree_RxInfo);
static void Unitree_ID_Select(uint8_t ID, Unitree_RxInfo_t *Unitree_RxInfo);

/* Functions --------------------------------------------------------------- */
/**
 * @brief  Unitree电机初始化
 * @param  Motor: Unitree电机信息结构体指针
 * @param  Motor_num: 电机数量
 * @return 无
 * @note   无
 */
void Unitree_Motor_Init(Unitree_Motor_Info_t *Motor,uint8_t Motor_num)
{
	for(uint8_t i=0;i<Motor_num;i++)
	{	
		if(Motor[i].Initlized == false)
		{
			Motor[i].Initlized = true;
			Motor[i].Data.cnt = 0;
			Motor[i].Data.pos_sum = 0;
			Motor[i].Data.Pos_Offset = 0;
			Motor[i].Cmd.Tx_ID = Motor[i].ID_Set.Tx_ID;
			Motor[i].Cmd.Mode = UNITREE_MODE_LOCK;
		}
		else break;
	};
}

/**
 * @brief  Unitree电机接收信息解包
 * @param  Motor_buf: 电机数据缓冲区指针
 * @param  Unitree_RxInfo: 接收数据结构体指针
 * @return 无
 * @note   无
 */
void Uintree_RxInfo_Unpack(volatile const uint8_t *Motor_buf,Unitree_RxInfo_t *Unitree_RxInfo)
{
	Unitree_RxInfo->HEAD = ((uint16_t)Motor_buf[0] << 8 | (uint16_t)Motor_buf[1]);
	if(Unitree_RxInfo->HEAD == Unitree_RxHEAD)
	{
		Unitree_RxInfo->Mode_Info.raw = Motor_buf[2];
		Unitree_RxInfo->T_fbk = (int16_t)((uint16_t)Motor_buf[3] | (uint16_t)Motor_buf[4] << 8);
		Unitree_RxInfo->W_fbk = (int16_t)((uint16_t)Motor_buf[5] | (uint16_t)Motor_buf[6] << 8);
		Unitree_RxInfo->Theta_fbk = (int32_t)((uint32_t)Motor_buf[7] | (uint32_t)Motor_buf[8] << 8 | (uint32_t)Motor_buf[9] << 16 | (uint32_t)Motor_buf[10] << 24);
		Unitree_RxInfo->Temp = (int8_t)Motor_buf[11];
		Unitree_RxInfo->Status.raw = (int16_t)(Motor_buf[12] | (uint16_t)Motor_buf[13] << 8);
		Unitree_RxInfo->CRC16 = (uint16_t)(Motor_buf[14] | (uint16_t)Motor_buf[15] << 8);
		/* 校验CRC16 */
		if(Verify_CRC16_Check_Sum((uint8_t *)Motor_buf,UNITREE_RX_BUF_LEN))
			/* 校验通过，选择电机ID */
			Unitree_ID_Select(Unitree_RxInfo->Mode_Info.bits.ID,Unitree_RxInfo);
		else return;
	}
	else return;
}

/**
 * @brief  Unitree电机指令
 * @param  Motor: Unitree电机信息结构体指针
 * @param  mode: 模式
 * @param  T_ff: 前馈扭矩
 * @param  W_des: 期望速度
 * @param  Pos_des: 期望位置
 * @param  K_p: 位置比例系数
 * @param  K_w: 速度比例系数
 * @return 无
 * @note   无
 */
void Unitree_Motor_Cmd(Unitree_Motor_Info_t *Motor, uint8_t mode, float T_ff, float W_des, float Pos_des, float K_p, float K_w)
{
	Motor->Cmd.Tx_ID = Motor->ID_Set.Tx_ID & 0xF;
	Motor->Cmd.Mode  = mode & 0x07;
	Motor->Cmd.T_set = (int16_t)(T_ff * 256.f);
	Motor->Cmd.W_set = (int16_t)(W_des * 256.f / 2 / PI * Unitree_8010_Ratio);
	Motor->Cmd.Pos_set = (int32_t)(Pos_des * 32768.f / 2 / PI * Unitree_8010_Ratio);
	Motor->Cmd.K_pos = (int16_t)(K_p * 1280.f) ;
	Motor->Cmd.K_spd = (int16_t)(K_w * 1280.f) ; 
}

/**
 * @brief  Unitree多电机指令
 * @param  Motor: Unitree电机信息结构体指针
 * @param  mode: 模式
 * @param  Motor_num: 电机数量
 * @param  T_ff: 前馈扭矩
 * @param  W_des: 期望速度
 * @param  Pos_des: 期望位置
 * @param  K_p: 位置比例系数
 * @param  K_w: 速度比例系数
 * @return 无
 * @note   无
 */
void Unitree_Motors_Cmd(Unitree_Motor_Info_t *Motor, uint8_t mode, uint8_t Motor_num, float T_ff, float W_des, float Pos_des, float K_p, float K_w)
{
	for(uint8_t i=0;i<Motor_num;i++)
	{
		Unitree_Motor_Cmd(&Motor[i], mode, T_ff, W_des, Pos_des, K_p, K_w);
	}
}

/**
 * @brief  Unitree多电机失能指令
 * @param  Motor: Unitree电机信息结构体指针
 * @param  Motor_num: 电机数量
 * @return 无
 * @note   无
 */
void Unitree_Motors_Discmd(Unitree_Motor_Info_t *Motor, uint8_t Motor_num)
{
	for(uint8_t i = 0;i<Motor_num;i++)
	{
		Motor[i].Cmd.Tx_ID = Motor[i].ID_Set.Tx_ID;
		Motor[i].Cmd.Mode  = UNITREE_MODE_LOCK;
	}
}

/**
 * @brief  Unitree电机控制
 * @param  huart: UART句柄
 * @param  Motor: Unitree电机信息结构体指针
 * @param  delay_time: 延时时间
 * @return 无
 * @note   无
 */
void Unitree_Motor_Ctrl(UART_HandleTypeDef *huart, Unitree_Motor_Info_t *Motor, uint16_t delay_time)
{
	static uint8_t Data[UNITREE_TX_BUF_LEN];

	Data[0] = 0xFE;
	Data[1] = 0xEE;
	Data[2] = ((uint8_t)Motor->Cmd.Mode & 0x7) << 4 | (Motor->Cmd.Tx_ID & 0xF);
	
	Data[3] = (Motor->Cmd.T_set) & 0xFF;
	Data[4] = (Motor->Cmd.T_set >> 8) & 0xFF;
	Data[5] = (Motor->Cmd.W_set ) & 0xFF;
	Data[6] = (Motor->Cmd.W_set >> 8) & 0xFF;
	Data[7] = (Motor->Cmd.Pos_set ) & 0xFF;
	Data[8] = (Motor->Cmd.Pos_set >> 8 ) & 0xFF;
	Data[9] = (Motor->Cmd.Pos_set >> 16) & 0xFF;
	Data[10]= (Motor->Cmd.Pos_set >> 24) & 0xFF;
	Data[11]= (Motor->Cmd.K_pos	) & 0xFF;
	Data[12]= (Motor->Cmd.K_pos	>> 8) & 0xFF;
	Data[13]= (Motor->Cmd.K_spd	) & 0xFF;
	Data[14]= (Motor->Cmd.K_spd	>> 8) & 0xFF;
	
	Motor->Cmd.CRC16 = Get_CRC16_Check_Sum(Data, UNITREE_TX_BUF_LEN - 2, 0x0000);
	
	Data[15]= (Motor->Cmd.CRC16) & 0xFF;
	Data[16]= (Motor->Cmd.CRC16 >> 8) & 0xFF;
	
	HAL_UART_Transmit_DMA(huart,Data,UNITREE_TX_BUF_LEN);
	
	osDelay(delay_time);
}

/**
 * @brief  Unitree多电机控制
 * @param  huart: UART句柄
 * @param  Motor: Unitree电机信息结构体指针
 * @param  Motor_num: 电机数量
 * @param  delay_time: 延时时间
 * @return 无
 * @note   无
 */
void Unitree_Motors_Ctrl(UART_HandleTypeDef *huart, Unitree_Motor_Info_t *Motor, uint8_t Motor_num, uint16_t delay_time)
{
	for(uint8_t i = 0;i<Motor_num;i++)
	{
		Unitree_Motor_Ctrl(huart,&Motor[i],delay_time);
	}
}

/* Private functions ------------------------------------------------------- */
/**
 * @brief  Unitree电机获取零点偏移
 * @param  Motor: Unitree电机信息结构体指针
 * @return 无
 * @note   上电/单片机重启电机零点校准
 */
static void Unitree_get_offset(Unitree_Motor_Info_t *Motor)
{
	if (Motor->Data.cnt < CALIBRATION_COUNT)
	{
		Motor->Data.pos_sum += Motor->Data.Pos;
		Motor->Data.cnt++;
		if (Motor->Data.cnt == CALIBRATION_COUNT) 
		{
			Motor->Data.Pos_Offset = Motor->Data.pos_sum / CALIBRATION_COUNT;
			if(Motor->Data.Rx_ID == 1) Power_OUT1_ON;
		}
	}
	else 
	{
		Motor->Data.Pos_Out = Motor->Data.Pos - Motor->Data.Pos_Offset;	
	}
}

/**
 * @brief  Unitree电机接收参数处理
 * @param  Motor: Unitree电机信息结构体指针
 * @param  Unitree_RxInfo: 接收数据结构体指针
 * @return 无
 * @note   无
 */
static void Unitree_Motor_Info_Update(Unitree_Motor_Info_t *Motor, Unitree_RxInfo_t *Unitree_RxInfo)
{
	Motor->Data.Rx_ID = Unitree_RxInfo->Mode_Info.bits.ID;
	Motor->Data.Error_Type  = (MotorError_Type_e)Unitree_RxInfo->Status.bits.MERROR;
	Motor->Data.Mode_Status = (MotorMode_Status_e)Unitree_RxInfo->Mode_Info.bits.STATUS;
	Motor->Data.Tor = ((float)Unitree_RxInfo->T_fbk / 256 * Unitree_8010_Ratio);
	Motor->Data.Vel = ((float)Unitree_RxInfo->W_fbk * 2*PI / 256 / Unitree_8010_Ratio) ;
	Motor->Data.Pos = ((float)Unitree_RxInfo->Theta_fbk * 2*PI / 32768 / Unitree_8010_Ratio);
	Motor->Data.Temp = (float)Unitree_RxInfo->Temp ;
	Unitree_get_offset(Motor);
}

/**
 * @brief  Unitree电机ID选择
 * @param  ID: 电机ID
 * @param  Unitree_RxInfo: 接收数据结构体指针
 * @return 无
 * @note   无
 */
static void Unitree_ID_Select(uint8_t ID, Unitree_RxInfo_t *Unitree_RxInfo)
{
	switch(ID) // 根据ID选择电机
	{
		// case 0:Unitree_Motor_Info_Update(&Chassis_Ctrl.Joint_Motor[0],Unitree_RxInfo); break;
		// case 1:Unitree_Motor_Info_Update(&Chassis_Ctrl.Joint_Motor[1],Unitree_RxInfo); break;
	}
}

/* Interrupt functions ----------------------------------------------------- */

/* ------------------------------------------------------------------------- */
