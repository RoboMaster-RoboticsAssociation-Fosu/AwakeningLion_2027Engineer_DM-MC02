/**
 ******************************************************************************
 * @file    DJI_Motor.h
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   DJI电机驱动函数声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef _DJI_MOTOR_H
#define _DJI_MOTOR_H

/* Includes ----------------------------------------------------------------- */
#include "stdbool.h"
#include "stm32h723xx.h"
#include "bsp_can.h"

/* Defines ------------------------------------------------------------------ */
/* 3508最大转速(rpm) */
#define MAX_3508_RPM 					8500
/* C620电流指令绝对值上限 */
#define DJI_C620_CURRENT_COMMAND_MAX		16384
/* C610电流指令绝对值上限 */
#define DJI_C610_CURRENT_COMMAND_MAX		10000
/* 底盘3508电机发送ID */
#define Chassis_3508_MotorA_TxID		0x200
#define Chassis_3508_MotorB_TxID		0x1FF
/* 底盘3508电机[n]接收ID */
#define Chassis_3508_Motor1_RxID		0x201
#define Chassis_3508_Motor2_RxID		0x202
#define Chassis_3508_Motor3_RxID		0x203
#define Chassis_3508_Motor4_RxID		0x204
#define Chassis_3508_Motor5_RxID		0x205
/* 3508减速比 */
#define DJI_3508_Ratio 					19.0f
/* 2006减速比 */
#define DJI_2006_Ratio 					36.f
/* 6020减速比 */
#define DJI_6020_Ratio 					1.f

/* Enums -------------------------------------------------------------------- */
/**
 * @brief 电机类型枚举
 */
typedef enum{
	DJI_GM6020,
    DJI_M3508,
    DJI_M2006,
    DJI_MOTOR_TYPE_NUM,
}DJI_Motor_Type_e;

/**
 * @brief 电机使能模式枚举
 */
typedef enum{
	Motor_Enable,
	Motor_Disable,
	Motor_Save_Zero_Position,
	DM_Motor_CMD_Type_Num,
}DM_Motor_CMD_Type_e;

/* Structs ------------------------------------------------------------------ */
/**
 * @brief 电机ID结构体
 */
typedef struct{
  uint32_t TxIdentifier;	/*!< FDCAN发送标识符 */
  uint32_t RxIdentifier;	/*!< FDCAN接收标识符 */
}DJI_Motor_ID_t;

/**
 * @brief 电机接收数据结构体
 */
typedef struct {
	bool 	 Initlized;		/*!< 初始化标志 */
	int16_t  SET_Current;	/*!< 电机设定电流 */
	int16_t  Current;		/*!< 电机电流 */
	int16_t  Velocity;		/*!< 电机转速 (RPM)*/
	int16_t  Encoder;		/*!< 电机编码器角度 */
	int16_t  Last_Encoder;	/*!< 上一次电机编码器角度 */
	float    Angle;			/*!< 电机角度(°) */
	uint8_t  Temperature;	/*!< 电机温度 */
}DJI_Motor_Data_t;

/**
 * @brief 电机信息结构体
 */
typedef struct{
	DJI_Motor_Type_e Motor_Type;	/*!< 电机类型 */
	DJI_Motor_ID_t ID_Set;	/*!< CAN传输信息 */
	DJI_Motor_Data_t Data;	/*!< 电机设备信息 */
	float wheel_T;					/*!< 车轮的输出扭矩，单位为N·m */
	bool feedback_valid;			/*!< 已收到至少一帧合法反馈 */
	uint32_t last_feedback_time_ms;	/*!< 最近合法反馈时间 */
}DJI_Motor_Info_t;

/* Externs ------------------------------------------------------------------ */

/* Functions ---------------------------------------------------------------- */
uint8_t DJI_Motor_ctrl(DJI_Motor_Info_t *DJI_Motor,hcan_t* hcan, uint16_t delay_time);
void DJI_Motor_Info_Update(DJI_Motor_Info_t *DJI_Motor,uint8_t *rx_data,uint32_t data_len);

/* -------------------------------------------------------------------------- */
#endif /* _DJI_MOTOR_H */
