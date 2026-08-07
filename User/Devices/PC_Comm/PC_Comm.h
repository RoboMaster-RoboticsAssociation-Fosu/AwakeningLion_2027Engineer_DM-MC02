/**
 ******************************************************************************
 * @file    PC_Comm.h
 * @version V1.0.0
 * @date    2026.04.11
 * @brief   上位机通信驱动实现声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 模板示例 未运用到实际工程里面
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __PC_COMM_H
#define __PC_COMM_H

/* Includes ----------------------------------------------------------------- */
#include "cmsis_os.h"
#include "stdbool.h"

/* Defines ------------------------------------------------------------------ */
#define PC_RxHEAD						0xAA		/* 上位机接收帧头 */
#define PC_RX_DATA_LEN					16u			/* 接收数据长度 */
// #define PC_TX_BUF_LEN								/* 发送数据长度(待定义) */

/* Enums -------------------------------------------------------------------- */

/* Structs ------------------------------------------------------------------ */
/**
 * @brief 数据类型转换联合体
 */
typedef union {
	float fval;			/* 浮点数值 */
	uint32_t uval;		/* 无符号整数值 */
} PC_turn_Typedef;

/**
 * @brief 上位机帧头结构体
 */
typedef struct {
    uint8_t RxHEAD;		/* 接收帧头 */
} PC_Head_Typedef;

/**
 * @brief 上位机信息结构体
 */
typedef struct {
    uint8_t HEAD;				/* 帧头 */
    uint8_t Data_Len;			/* 数据长度 */
    uint8_t Checksum;			/* 校验和 */
    PC_turn_Typedef  Vx;		/* X方向速度 */
    PC_turn_Typedef  Vy;		/* Y方向速度 */
    PC_turn_Typedef  Vz;		/* Z方向速度 */
    uint8_t Reserved[4];		/* 保留字节 */
    bool pc_lost;				/* 上位机丢失标志 */
	bool pc_active;				/* 上位机活动标志 */
	uint8_t online_cnt;			/* 在线计数 */
} PC_Info_Typedef;

/* Externs ------------------------------------------------------------------ */
extern uint8_t buff[19];
extern PC_Info_Typedef PC_RxInfo;
extern uint8_t PC_MultiRx_Buf[PC_RX_DATA_LEN];

/* Functions ---------------------------------------------------------------- */
void PC_Info_Update(uint8_t *buff,PC_Info_Typedef *data);
void PC_Info_Upload(void);
void PC_Offline_Detect(PC_Info_Typedef *PC_Info);

/* -------------------------------------------------------------------------- */
#endif /* __PC_COMM_H */