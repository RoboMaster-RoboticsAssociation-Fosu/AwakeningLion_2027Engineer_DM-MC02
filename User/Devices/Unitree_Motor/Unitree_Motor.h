/**
 ******************************************************************************
 * @file    Unitree_Motor.h
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   Unitree电机驱动函数声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __UNITREE_MOTOR_H
#define __UNITREE_MOTOR_H

/* Includes ----------------------------------------------------------------- */
#include "stm32h7xx.h"

/* Defines ------------------------------------------------------------------ */
#define Chassis_Go8010_Motor_R_ID		0			/* 底盘Go8010右电机ID */
#define Chassis_Go8010_Motor_L_ID		1			/* 底盘Go8010左电机ID */

#define UNITREE_RX_BUF_LEN				16u			/* 收发数据长度 */
#define UNITREE_TX_BUF_LEN				17u			/* 发送数据长度 */

#define Kp_MAX							25.599f		/* Kp值无限制 */
#define Kp_MIN							0f			/* Kp值最小 */
#define Kw_MAX							25.599f		/* Kw值无限制 */
#define Kw_MIN							0f			/* Kw值最小 */

#define Unitree_8010_Ratio				6.33f		/* Go_8010_减速比 */

/* Enums -------------------------------------------------------------------- */
/**
 * @brief 电机类型枚举
 */
typedef enum {
	Uni_Go8010 = 0,
}Unitree_Motor_Type_e;

/**
 * @brief 电机模式枚举
 */
typedef enum {
    UNITREE_MODE_LOCK  = 0,      	// 锁定(Default)
    UNITREE_MODE_FOC   = 1,       	// FOC闭环
    UNITREE_MODE_CALIB = 2, 		// 编码器校准
}  MotorMode_Status_e;

/**
 * @brief 电机错误枚举
 */
typedef enum{
	MOTOR_ERROR_NORMAL 		= 0,   // 0.正常
    MOTOR_ERROR_OVERHEAT 	= 1,   // 1.过热
    MOTOR_ERROR_OVERCURRENT = 2,   // 2.过流
    MOTOR_ERROR_OVERVOLTAGE = 3,   // 3.过压
    MOTOR_ERROR_ENCODER 	= 4,   // 4.编码器故障
    MOTOR_ERROR_RESERVED_5 	= 5,
    MOTOR_ERROR_RESERVED_6 	= 6,
    MOTOR_ERROR_RESERVED_7 	= 7    // 5-7.保留
} MotorError_Type_e;

/* Structs ------------------------------------------------------------------ */
/**
 * @brief 接收结构体
 */
#pragma pack(push, 1)				/* 单字节对齐 */
typedef struct{
	uint16_t HEAD;					/* 数据包头 */
	union{
	  struct{
	    uint8_t ID		: 4;		/* 目标电机ID */
	    uint8_t STATUS	: 3;		/* 电机工作模式 */
	    uint8_t RESV1	: 1;		/* 保留位1 */
	  }bits;
	  uint8_t raw;					/* 原始数据raw */
	}Mode_Info;
	int16_t	 T_fbk;					/* 实际反馈电机转矩(N*M) */
	int16_t  W_fbk;					/* 实际反馈电机速度(rad/s) */
	int32_t  Theta_fbk;				/* 实际反馈电机位置(多圈累计)(rad) */
	int8_t   Temp;					/* 电机温度 */
	union{
	  struct{
		uint16_t  MERROR : 3;		/* 电机错误标识 */
		uint16_t  FORCE	 : 12; 		/* 足端力 */
		uint16_t  RESV2	 : 1;		/* 保留位2 */
	  }bits;
	  uint16_t raw;					/* 原始数据raw */
	}Status;
	uint16_t CRC16;					/* CRC16校验 */
} Unitree_RxInfo_t;
#pragma pack(pop)

/**
 * @brief 电机接收数据结构体
 */
typedef struct{
	uint8_t	Rx_ID;						/* 电机接收ID */
	float Tor;							/* 电机输出力矩 */
	float Vel;							/* 电机输出转速 */
	float Pos;							/* 电机原始角度(多圈累计) */
	float Pos_Offset;					/* 电机补偿角度 */
	float Pos_Out;						/* 电机补偿后角度 */
	float Temp;							/* 电机温度 */
	MotorMode_Status_e Mode_Status;		/* 电机模式状态 */
	MotorError_Type_e  Error_Type;		/* 电机错误类型 */
	uint8_t cnt;
	float 	pos_sum;
} Unitree_Motor_Data_t;

/**
 * @brief 电机ID结构体
 */
typedef struct{
	uint8_t Tx_ID;
	uint8_t Rx_ID;
} Unitree_Motor_ID_t;

/**
 * @brief 电机控制参数结构体
 */
typedef struct{
	uint8_t Tx_len;					/* 发送长度 */
	uint8_t Tx_ID;					/* 发送ID */
	MotorMode_Status_e Mode;		/* 工作模式 */
	int16_t T_set;					/* 给定力矩 */
	int16_t W_set;					/* 给定转速 */
	int32_t Pos_set;				/* 给定角度位置 */
	int16_t K_pos;					/* 给定刚度Kp */
	int16_t K_spd;					/* 速度刚度(阻尼)Kd */
	uint16_t CRC16;					/* CRC校验 */
} Unitree_Motor_Cmd_t;

/**
 * @brief 电机信息结构体
 */
typedef struct{
	bool Initlized;							/* 初始化标志 */
	Unitree_Motor_Type_e  	   Motor_Type;	/* 电机类型 */
	Unitree_Motor_ID_t   ID_Set;		/* 电机ID设置 */
	Unitree_Motor_Data_t Data;		/* 电机接收数据 */
	Unitree_Motor_Cmd_t  Cmd;			/* 电机控制参数 */
} Unitree_Motor_Info_t;

/* Externs ------------------------------------------------------------------ */
extern Unitree_RxInfo_t Unitree_Rx_Info;
extern uint8_t Unitree_MultiRx_Buf[2][UNITREE_RX_BUF_LEN];

/* Functions ---------------------------------------------------------------- */
void Unitree_Motor_Init(Unitree_Motor_Info_t *Motor,uint8_t Motor_num);
void Uintree_RxInfo_Unpack(volatile const uint8_t *Motor_buf,Unitree_RxInfo_t *Rx_Data);
void Unitree_Motor_Cmd(Unitree_Motor_Info_t *Motor, uint8_t mode, float T_ff, float W_des, float Pos_des, float K_p, float K_w);
void Unitree_Motors_Cmd(Unitree_Motor_Info_t *Motor, uint8_t mode, uint8_t Motor_num, float T_ff, float W_des, float Pos_des, float K_p, float K_w);
void Unitree_Motor_Ctrl(UART_HandleTypeDef *huart, Unitree_Motor_Info_t *Motor, uint16_t delay_time);
void Unitree_Motors_Ctrl(UART_HandleTypeDef *huart, Unitree_Motor_Info_t *Motor, uint8_t Motor_num, uint16_t delay_time);
void Unitree_Motors_Discmd(Unitree_Motor_Info_t *Motor, uint8_t Motor_num);

/* -------------------------------------------------------------------------- */
#endif /* __UNITREE_MOTOR_H */
