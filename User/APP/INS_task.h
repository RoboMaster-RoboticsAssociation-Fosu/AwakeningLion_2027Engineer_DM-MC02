/**
 ******************************************************************************
 * @file    ins_task.h
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   姿态解算任务功能声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __INS_TASK_H
#define __INS_TASK_H

/* Includes ----------------------------------------------------------------- */
#include "stdint.h"
#include "BMI088driver.h"
#include "QuaternionEKF.h"

/* Defines ------------------------------------------------------------------ */
/* 坐标轴 */
#define X					0					// X轴
#define Y					1					// Y轴
#define Z					2					// Z轴

/* 任务周期 */
#define INS_TASK_PERIOD		1					// 姿态解算任务周期 (ms)

/* Enums -------------------------------------------------------------------- */


/* Structs ------------------------------------------------------------------ */
/**
 * @brief 姿态信息结构体
 */
typedef struct
{
    float q[4];					// 四元数

    float Gyro[3];				// 角速度 (rad/s)，当前安装下 Gyro[Z] 俯视逆时针为正
    float Accel[3];				// 加速度 (m/s^2)
    float MotionAccel_b[3];		// 机体坐标系下运动加速度
    float MotionAccel_n[3];		// 大地坐标系下运动加速度

    float AccelLPF;				// 加速度低通滤波系数

    float xn[3];				// x轴在地理坐标系下的分量
    float yn[3];				// y轴在地理坐标系下的分量
    float zn[3];				// z轴在地理坐标系下的分量

    float atanxz;				//俯仰角
    float atanyz;				//横滚角

    float Roll;					// 横滚角 (rad)
    float Pitch;				// 俯仰角 (rad)
    float Yaw;					// 偏航角 (rad)，俯视逆时针为正
    float YawTotalAngle;		// 累计偏航角 (rad)，俯视逆时针为正
	float YawAngleLast;			// 上一次的偏航角 (rad)
	float YawRoundCount;		// 偏航角圈数
	
	float v_n;					// 大地坐标系下水平运动速度
	float x_n;					// 大地坐标系下水平运动位移
	
	uint8_t ins_flag;			// 姿态解算完成标志
	uint32_t last_update_time_ms;	// 最近一次姿态更新时刻
} INS_t;

/**
 * @brief IMU参数结构体
 */
typedef struct
{
    uint8_t flag;				// 参数存在标志

    float scale[3];				// 缩放比例

    float Yaw;					// 偏航角 (rad)
    float Pitch;				// 俯仰角 (rad)
    float Roll;					// 横滚角 (rad)
} IMU_Param_t;

/* Externs ------------------------------------------------------------------ */
extern INS_t INS;				// 姿态信息结构体变量

/* Functions ---------------------------------------------------------------- */
void INS_Init(void);
void INS_task(void);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

/* -------------------------------------------------------------------------- */
#endif /* __INS_TASK_H */
