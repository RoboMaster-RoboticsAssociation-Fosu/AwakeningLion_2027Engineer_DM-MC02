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
/* GM6020 ID 5：反馈 ID = 0x204 + motor_id；电流控制使用 ID 5~7 组。 */
#define DJI_GM6020_MOTOR5_RX_ID          0x209U
#define DJI_GM6020_CURRENT_GROUP1_4_TX_ID 0x1FEU
#define DJI_GM6020_CURRENT_GROUP5_7_TX_ID 0x2FEU
/* GM6020 手册 v1.4（2023.10）第 6、7、11 页：转矩电流与轴端力矩的换算。
 * ±16384 对应 ±3 A；0.741 N·m/A 是转矩常数，不适用于电源输入电流。
 * 下述最大力矩仅为协议给定量程，持续工作及机械安全限位由控制任务负责。
 */
#define DJI_GM6020_TORQUE_CONSTANT_NM_PER_A 0.741f
#define DJI_GM6020_CURRENT_MAX_A           3.0f
#define DJI_GM6020_CURRENT_COMMAND_MAX     16384
#define DJI_GM6020_TORQUE_MAX_NM \
    (DJI_GM6020_TORQUE_CONSTANT_NM_PER_A * DJI_GM6020_CURRENT_MAX_A)
/* 纯物理量换算；参数仅求值一次，不限幅、不发送、不处理 NaN/Inf。 */
#define DJI_GM6020_CURRENT_TO_TORQUE_NM(current_a) \
    ((current_a) * DJI_GM6020_TORQUE_CONSTANT_NM_PER_A)
#define DJI_GM6020_TORQUE_TO_CURRENT_A(torque_nm) \
    ((torque_nm) / DJI_GM6020_TORQUE_CONSTANT_NM_PER_A)
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

/** @brief 力矩控制接口的处理结果；不改变旧电流发送接口的返回约定。 */
typedef enum
{
    DJI_MOTOR_TORQUE_OK = 0,
    DJI_MOTOR_TORQUE_INVALID_ARGUMENT,
    DJI_MOTOR_TORQUE_UNSUPPORTED_TYPE,
    DJI_MOTOR_TORQUE_SEND_FAILED,
} DJI_Motor_TorqueStatus_e;

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
	bool feedback_valid;			/*!< 已收到过反馈的锁存位，不表示当前在线 */
	uint32_t last_feedback_time_ms;	/*!< 最近合法反馈时间 */
}DJI_Motor_Info_t;

/* Externs ------------------------------------------------------------------ */

/* Functions ---------------------------------------------------------------- */
uint8_t DJI_Motor_ctrl(DJI_Motor_Info_t *DJI_Motor,hcan_t* hcan, uint16_t delay_time);
uint8_t DJI_Motor_SendGroupCurrent(hcan_t *hcan,
								   uint32_t tx_identifier,
								   const int16_t current[4]);
/**
 * @brief 按 DJI 电机类型将目标轴端力矩（N·m）转换为电流，限幅后一次 CAN 组发送。
 * @param hcan CAN 句柄，由所属任务初始化并调度。
 * @param motor_type 组内电机类型；目前仅 GM6020 已核验换算，其他类型返回不支持。
 * @param tx_identifier GM6020 仅支持 0x1FE（ID 1~4）或 0x2FE（ID 5~7）。
 * @param torque_nm 四槽目标力矩；按组内电机 ID 排序，0x2FE 的第 4 槽必须为 0。
 * @param applied_torque_nm 可选四槽输出；发送成功后写入限幅后、量化前的力矩。
 *                         可以与输入数组相同；任何失败均保留原值。
 * @return OK 表示 CAN 入队成功（含限幅）；其余状态区分参数错误、型号不支持及发送失败。
 * @note GM6020 须启用电流环；本接口不是外部位置闭环或实测力矩反馈。
 *       有限输入按对应型号协议量程限幅，编码四舍五入；NaN/Inf 不发送。
 *       同组所有电机须由同一 owner 一起提交，未使用槽填 0；本接口不等待、不重试。
 */
DJI_Motor_TorqueStatus_e DJI_Motor_SendGroupTorque(hcan_t *hcan,
                                                 DJI_Motor_Type_e motor_type,
                                                 uint32_t tx_identifier,
                                                 const float torque_nm[4],
                                                 float applied_torque_nm[4]);
void DJI_Motor_Info_Update(DJI_Motor_Info_t *DJI_Motor,uint8_t *rx_data,uint32_t data_len);

/* -------------------------------------------------------------------------- */
#endif /* _DJI_MOTOR_H */
