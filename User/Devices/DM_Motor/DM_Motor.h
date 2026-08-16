/**
 ******************************************************************************
 * @file    DM_Motor.h
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   DM系列电机驱动函数声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H

/* Includes ----------------------------------------------------------------- */
#include "main.h"
#include "bsp_can.h"
#include <stdbool.h>

/* Defines ------------------------------------------------------------------ */
#define MIT_MODE 					0x000		/* MIT模式 */
#define POS_MODE					0x100		/* 位置模式 */
#define SPEED_MODE					0x200		/* 速度模式 */

#define KP_MIN    0.0f
#define KP_MAX    500.0f
#define KD_MIN    0.0f
#define KD_MAX    5.0f

#define J4310_P_MIN     -12.5f
#define J4310_P_MAX     12.5f
#define J4310_V_MIN     -30.0f
#define J4310_V_MAX     30.0f
#define J4310_T_MIN     -10.0f
#define J4310_T_MAX     10.0f

#define J4340_P_MIN     -12.5f
#define J4340_P_MAX     12.5f
#define J4340_V_MIN     -10.0f
#define J4340_V_MAX     10.0f
#define J4340_T_MIN 	-28.0f
#define J4340_T_MAX 	28.0f

#define J10010L_P_MIN    -12.5f
#define J10010L_P_MAX    12.5f
#define J10010L_V_MIN    -25.0f
#define J10010L_V_MAX    25.0f
/* MIT 协议编码满量程；不是电机机械峰值扭矩 120 Nm。 */
#define J10010L_T_MIN    -200.0f
#define J10010L_T_MAX    200.0f

/* Enums -------------------------------------------------------------------- */
/**
 * @brief DM电机类型枚举
 */
typedef enum{
    DM_J4310 = 0,
    DM_J4340,
    DM_J8006,
    DM_J10010L,
    DM_MOTOR_TYPE_NUM,
}DM_Motor_Type_e;

/**
 * @brief DM电机模式枚举
 */
typedef enum{
    Mit_mode = 0x000,
    Pos_mode = 0x100,
    Spd_mode = 0x200,
}DM_Motor_mode_e;

/**
 * @brief DM 反馈帧 D[0] 高四位状态码。
 * @note 0 和 1 分别是失能、使能；3~5、8~E 是说明书定义的驱动故障。
 */
typedef enum
{
	DM_MOTOR_STATE_DISABLED = 0x0,
	DM_MOTOR_STATE_ENABLED = 0x1,
	DM_MOTOR_STATE_OUTPUT_ENCODER_CALIBRATION_ERROR = 0x3,
	DM_MOTOR_STATE_SENSOR_ERROR = 0x4,
	DM_MOTOR_STATE_MOTOR_ENCODER_CALIBRATION_ERROR = 0x5,
	DM_MOTOR_STATE_OVER_VOLTAGE = 0x8,
	DM_MOTOR_STATE_UNDER_VOLTAGE = 0x9,
	DM_MOTOR_STATE_OVER_CURRENT = 0xA,
	DM_MOTOR_STATE_MOS_OVER_TEMPERATURE = 0xB,
	DM_MOTOR_STATE_COIL_OVER_TEMPERATURE = 0xC,
	DM_MOTOR_STATE_COMMUNICATION_LOST = 0xD,
	DM_MOTOR_STATE_OVERLOAD = 0xE,
}DM_Motor_State_e;

/* Structs ------------------------------------------------------------------ */
/**
 * @brief DM电机ID结构体
 */
typedef struct
{
  uint32_t TxIdentifier;	/* FDCAN发送标识符 */
  uint32_t RxIdentifier;	/* FDCAN接收标识符 */
}DM_Motor_ID_t;

/**
 * @brief DM电机数据结构体
 */
typedef struct 
{
	uint16_t id;			/* 电机ID */
	DM_Motor_State_e state;	/* D[0] 高四位状态码 */
	int p_int;			    /* 位置整数 */
	int v_int;			    /* 速度整数 */
	int t_int;			    /* 扭矩整数 */
	int kp_int;			    /* KP整数 */
	int kd_int;			    /* KD整数 */
	float pos;			    /* 位置 */
	float vel;			    /* 速度 */
	float tor;			    /* 扭矩 */
	float Kp;			    /* KP系数 */
	float Kd;			    /* KD系数 */
	float Tmos;			    /* MOS管温度 */
	float Tcoil;			/* 线圈温度 */
}DM_Motor_Data_t;

/**
 * @brief 单台 DM 电机的使能/失能请求状态。
 * @note 仅保存实例自身的跨周期状态；发送顺序仍由电机所属任务调度。
 */
typedef struct
{
	bool requested_enabled;	/* 当前请求的使能状态 */
	bool pending;				/* 该状态等待首次发送或下一次重试 */
	bool awaiting_response;	/* 已发送请求，正在等待对应的新反馈 */
	bool failed;				/* 达到最大次数仍未确认请求状态 */
	uint8_t attempt_count;	/* 本次状态切换已经执行的发送次数 */
	uint32_t request_time_ms;	/* 最近一次成功发送请求的时间 */
	uint32_t next_retry_time_ms;	/* 允许执行下一次发送的最早时间 */
	uint32_t request_feedback_sequence;	/* 当前请求已消费到的反馈序号 */
}DM_Motor_CommandState_t;

/**
 * @brief DM电机实例，包含固定装配配置和最近一次反馈。
 */
typedef struct
{
    DM_Motor_mode_e Mode;		    /* 电机模式 */
    DM_Motor_Type_e Motor_Type;	    /* 电机类型 */
    DM_Motor_ID_t ID_Set;		    /* ID设置 */
    DM_Motor_Data_t Data;		    /* 数据 */
	DM_Motor_CommandState_t Command;	/* 使能/失能请求状态 */
	bool feedback_valid;				/* 已收到过反馈的锁存位，不表示当前在线 */
	uint32_t last_feedback_time_ms;	/* 最近合法反馈时间 */
	uint32_t feedback_sequence;		/* 每收到一帧合法反馈递增一次 */
}DM_Motor_Info_t;

/**
 * @brief 使用固定模式、型号和收发 ID 静态创建一个 DM 电机实例。
 * @note 反馈数据和在线状态由 C 的静态零初始化完成，不需要额外 Init。
 */
#define DM_MOTOR_CREATE(mode_value, type_value, tx_id_value, rx_id_value) \
	{ \
		.Mode = (mode_value), \
		.Motor_Type = (type_value), \
		.ID_Set = { \
			.TxIdentifier = (tx_id_value), \
			.RxIdentifier = (rx_id_value), \
		}, \
		.Command = { \
			.requested_enabled = false, \
			.pending = true, \
		}, \
	}

/* Externs ------------------------------------------------------------------ */

/* Functions ---------------------------------------------------------------- */
uint8_t DM_Enable_Motor(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, uint8_t delay_time);
uint8_t DM_Disable_Motor(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, uint8_t delay_time);
uint8_t DM_Save_Motor_Zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, uint8_t delay_time);
void DM_Motor_Info_Update(DM_Motor_Info_t *motor, const uint8_t *rx_data,uint32_t data_len);
uint8_t DM_Motor_Ctrl(hcan_t *hcan, volatile DM_Motor_Info_t *motor, float pos, float vel, float kp, float kd, float torq, uint8_t delay_time);

/* -------------------------------------------------------------------------- */
#endif /* __DM_MOTOR_H */
