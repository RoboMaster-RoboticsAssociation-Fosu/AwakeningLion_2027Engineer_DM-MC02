/**
 ******************************************************************************
 * @file    DJI_Motor.c
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   DJI电机驱动函数
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "DJI_Motor.h"
#include "bsp_dwt.h"
#include "fdcan.h"
#include "stdbool.h"
#include "cmsis_os2.h"
#include <math.h>

/* Defines ----------------------------------------------------------------- */

/* Global variable --------------------------------------------------------- */

/* Static Fun -------------------------------------------------------------- */
static float DJI_Motor_Encoder_To_Anglesum(DJI_Motor_Data_t *Data,float Torque_Ratio,uint16_t MAXEncoder);

/* Functions --------------------------------------------------------------- */
/**
 * @brief  大疆电机控制
 * @param  DJI_Motor: DJI电机信息结构体指针
 * @param  hcan: CAN句柄
 * @param  delay_time: 延时时间
 * @return 0 表示CAN发送成功，非零表示发送失败
 * @note   delay_time为0时不阻塞当前任务
 */
uint8_t DJI_Motor_ctrl(DJI_Motor_Info_t *DJI_Motor,hcan_t* hcan, uint16_t delay_time)
{
	const int16_t current[4] = {
		DJI_Motor[0].Data.SET_Current,
		DJI_Motor[1].Data.SET_Current,
		DJI_Motor[2].Data.SET_Current,
		DJI_Motor[3].Data.SET_Current,
	};
	uint8_t send_status = DJI_Motor_SendGroupCurrent(
		hcan,
		DJI_Motor[0].ID_Set.TxIdentifier,
		current);
	
	if(delay_time > 0U)
	{
		osDelay(delay_time);
	}
	return send_status;
}

/**
 * @brief 发送一组四槽 DJI 电流指令。
 * @param hcan CAN 句柄。
 * @param tx_identifier DJI 控制组 ID。
 * @param current 四个连续电机槽位的电流指令。
 * @return 0 表示 CAN 入队成功，非零表示参数或 HAL 发送失败。
 * @note 只负责 DJI 协议的大端打包，不等待邮箱或电机应答。
 */
uint8_t DJI_Motor_SendGroupCurrent(hcan_t *hcan,
								   uint32_t tx_identifier,
								   const int16_t current[4])
{
	uint8_t data[8];
	uint32_t index;

	if ((hcan == NULL) || (current == NULL))
	{
		return 1U;
	}

	for (index = 0U; index < 4U; index++)
	{
		data[index * 2U] = (uint8_t)(((uint16_t)current[index] >> 8U) & 0xFFU);
		data[index * 2U + 1U] = (uint8_t)((uint16_t)current[index] & 0xFFU);
	}

	return canx_send_data(hcan, tx_identifier, data, sizeof(data));
}

/** @brief 选择型号换算参数后编码整组力矩；成功发送才发布限幅结果。 */
DJI_Motor_TorqueStatus_e DJI_Motor_SendGroupTorque(hcan_t *hcan,
                                                 DJI_Motor_Type_e motor_type,
                                                 uint32_t tx_identifier,
                                                 const float torque_nm[4],
                                                 float applied_torque_nm[4])
{
    int16_t current[4];
    float limited_torque_nm[4];
    float torque_max_nm;
    float command_per_torque_nm;
    int16_t current_command_max;
    uint32_t index;

    if ((hcan == NULL) || (torque_nm == NULL))
    {
        return DJI_MOTOR_TORQUE_INVALID_ARGUMENT;
    }
    /* 型号决定物理量换算和合法控制帧；未核验型号不猜测参数、不发送。 */
    switch (motor_type)
    {
        case DJI_GM6020:
            if ((tx_identifier != DJI_GM6020_CURRENT_GROUP1_4_TX_ID) &&
                (tx_identifier != DJI_GM6020_CURRENT_GROUP5_7_TX_ID))
            {
                return DJI_MOTOR_TORQUE_INVALID_ARGUMENT;
            }
            /* ID 5~7 组的最后两字节保留，第 4 槽必须为 0。 */
            if ((tx_identifier == DJI_GM6020_CURRENT_GROUP5_7_TX_ID) &&
                (torque_nm[3] != 0.0f))
            {
                return DJI_MOTOR_TORQUE_INVALID_ARGUMENT;
            }
            torque_max_nm = DJI_GM6020_TORQUE_MAX_NM;
            current_command_max = DJI_GM6020_CURRENT_COMMAND_MAX;
            break;
        /* 暂未支持型号保护开始：3508/2006 的转矩电流换算尚未核验。
         * 范围仅本力矩接口；取得对应型号协议、转矩常数并验证后增加换算分支。
         */
        default:
            return DJI_MOTOR_TORQUE_UNSUPPORTED_TYPE;
        /* 暂未支持型号保护结束。 */
    }
    command_per_torque_nm = (float)current_command_max / torque_max_nm;

    for (index = 0U; index < 4U; index++)
    {
        if (!isfinite(torque_nm[index]))
        {
            return DJI_MOTOR_TORQUE_INVALID_ARGUMENT;
        }
        /* 先限幅再转换，避免大数溢出；端点直接编码，保证正负满量程准确。 */
        if (torque_nm[index] >= torque_max_nm)
        {
            limited_torque_nm[index] = torque_max_nm;
            current[index] = current_command_max;
        }
        else if (torque_nm[index] <= -torque_max_nm)
        {
            limited_torque_nm[index] = -torque_max_nm;
            current[index] = (int16_t)-current_command_max;
        }
        else
        {
            limited_torque_nm[index] = torque_nm[index];
            current[index] = (int16_t)roundf(torque_nm[index] * command_per_torque_nm);
        }
    }

    if (DJI_Motor_SendGroupCurrent(hcan, tx_identifier, current) != 0U)
    {
        return DJI_MOTOR_TORQUE_SEND_FAILED;
    }
    /* 整组入队成功后再提交输出，同时允许调用者原地更新目标数组。 */
    if (applied_torque_nm != NULL)
    {
        for (index = 0U; index < 4U; index++)
        {
            applied_torque_nm[index] = limited_torque_nm[index];
        }
    }
    return DJI_MOTOR_TORQUE_OK;
}

/**
 * @brief  更新大疆电机信息
 * @param  DJI_Motor: DJI电机信息结构体指针
 * @param  rx_data: 接收数据指针
 * @param  data_len: 数据长度
 * @return 无
 * @note   无
 */
void DJI_Motor_Info_Update(DJI_Motor_Info_t *DJI_Motor,uint8_t *rx_data,uint32_t data_len)
{
	/* 检查标识符 */
	if(data_len!=FDCAN_DLC_BYTES_8) return;
	
	/* 转换通用电机数据 */
	DJI_Motor->Data.Temperature = rx_data[6];
	DJI_Motor->Data.Encoder  = ((int16_t)rx_data[0] << 8 | (int16_t)rx_data[1]);
	DJI_Motor->Data.Velocity = ((int16_t)rx_data[2] << 8 | (int16_t)rx_data[3]);
	DJI_Motor->Data.Current  = ((int16_t)rx_data[4] << 8 | (int16_t)rx_data[5]);
	DJI_Motor->feedback_valid = true;
	DJI_Motor->last_feedback_time_ms = DWT_GetTimeMs();

	/* 将编码器值转换为角度 */
	switch(DJI_Motor->Motor_Type)
	{
		case DJI_GM6020:
			DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Anglesum(&DJI_Motor->Data,DJI_6020_Ratio,8192); 	//6020减速比为1:1，输出轴为位置
		break;
	
		case DJI_M3508:
			DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Anglesum(&DJI_Motor->Data,DJI_3508_Ratio,8192);
		break;
		
		case DJI_M2006:
			DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Anglesum(&DJI_Motor->Data,DJI_2006_Ratio,8192);
		break;
		default:break;
	}
}

/**
  * @brief  循环约束
  * @param  Input: 输入值
  * @param  Min_Value: 最小值
  * @param  Max_Value: 最大值
  * @return 约束后的值
  * @note   无
  */
float F_Loop_Constrain(float Input, float Min_Value, float Max_Value)
{
  if (Max_Value < Min_Value)
  {
    return Input;
  }
  
  float len = Max_Value - Min_Value;    

  if (Input > Max_Value)
  {
      do{
          Input -= len;
      }while (Input > Max_Value);
  }
  else if (Input < Min_Value)
  {
      do{
          Input += len;
      }while (Input < Min_Value);
  }
  return Input;
}

/* Private functions ------------------------------------------------------- */
/**
 * @brief  编码器值转累计角度
 * @param  Data: DJI电机数据结构体指针
 * @param  Torque_Ratio: 减速比
 * @param  MAXEncoder: 编码器最大值
 * @return 累计角度(°) 
 * @note   无
 */
static float DJI_Motor_Encoder_To_Anglesum(DJI_Motor_Data_t *Data,float Torque_Ratio,uint16_t MAXEncoder)
{
  float res1 = 0,res2 =0;
  
  if(Data == NULL) return 0;
  
  /* 判断电机是否初始化 */
  if(Data->Initlized != true)
  {
    /* 更新上一次的编码器值 */
    Data->Last_Encoder = Data->Encoder;

    /* 重置角度 */
    Data->Angle = 0;

    /* 设置初始化标志 */
    Data->Initlized = true;
  }
  
  /* 获取可能的最小编码器误差 */
  if(Data->Encoder < Data->Last_Encoder)
  {
      res1 = Data->Encoder - Data->Last_Encoder + MAXEncoder;
  }
  else if(Data->Encoder > Data->Last_Encoder)
  {
      res1 = Data->Encoder - Data->Last_Encoder - MAXEncoder;
  }
  res2 = Data->Encoder - Data->Last_Encoder;
  
  /* 更新上一次的编码器值 */
  Data->Last_Encoder = Data->Encoder;
  
  /* 将编码器数据转换为总角度 */
	if(fabsf(res1) > fabsf(res2))
	{
		Data->Angle += (float)res2/(MAXEncoder*Torque_Ratio)*360.f;
	}
	else
	{
		Data->Angle += (float)res1/(MAXEncoder*Torque_Ratio)*360.f;
	}
  
  return Data->Angle;
}

/* Interrupt functions ----------------------------------------------------- */

/* ------------------------------------------------------------------------- */
