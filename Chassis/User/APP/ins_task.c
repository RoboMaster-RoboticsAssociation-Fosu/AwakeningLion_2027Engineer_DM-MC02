/**
 ******************************************************************************
 * @file    ins_task.c
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   姿态解算任务功能实现
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "ins_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_dwt.h"
#include "mahony_filter.h"
#include "BMI088driver.h"
#include "cmsis_os2.h"

/* Defines ----------------------------------------------------------------- */
#define INS_WARMUP_TIME_MS 5000U

/** @brief 本地传感器数组的轴索引，不向公共头文件暴露通用 X/Y/Z 宏。 */
enum
{
    INS_AXIS_X = 0,
    INS_AXIS_Y,
    INS_AXIS_Z
};

/* Global variable --------------------------------------------------------- */
INS_t INS;						// 姿态信息结构体变量


struct MAHONY_FILTER_t mahony;		// Mahony滤波器结构体
Axis3f Gyro,Accel;					// 陀螺仪和加速度计数据
float gravity[3] = {0, 0, 9.81f};	// 重力加速度向量

uint32_t INS_DWT_Count = 0;		// DWT计数器
float ins_dt = 0.0f;			// 姿态解算时间间隔
int stop_time;					// 停止时间
static uint32_t ins_warmup_start_ms;	// 姿态滤波预热起始时刻

/* Static Fun -------------------------------------------------------------- */


/* Functions --------------------------------------------------------------- */
/**
  * @brief  姿态解算任务初始化
  * @param  无
  * @return 无
  * @note   无
  */
void InsTask_Init(void)
{ 
	mahony_init(&mahony,1.0f,0.0f,0.001f);
   	INS.AccelLPF = 0.0089f;
	INS.ins_flag = 0U;
	ins_warmup_start_ms = DWT_GetTimeMs();
	/* 建立任务自己的时间基准，避免首次 dt 包含上电后的启动等待时间。 */
	(void)DWT_GetDeltaT(&INS_DWT_Count);
}

/**
  * @brief  姿态解算任务主函数
  * @param  无
  * @return 无
  * @note   此函数在操作系统任务中循环调用
  */
void InsTask_Run(void)
{
	 InsTask_Init();
	 
	 INS_t updated = INS;
	 while(1)
	 {	
		uint32_t now_ms;

		ins_dt = DWT_GetDeltaT(&INS_DWT_Count);
	
		mahony.dt = ins_dt;

		BMI088_Read(&BMI088);

		updated.Accel[INS_AXIS_X] = BMI088.Accel[INS_AXIS_X];
		updated.Accel[INS_AXIS_Y] = BMI088.Accel[INS_AXIS_Y];
		updated.Accel[INS_AXIS_Z] = BMI088.Accel[INS_AXIS_Z];
		Accel.x=BMI088.Accel[0];
		Accel.y=BMI088.Accel[1];
		Accel.z=BMI088.Accel[2];
		updated.Gyro[INS_AXIS_X] = BMI088.Gyro[INS_AXIS_X];
		updated.Gyro[INS_AXIS_Y] = BMI088.Gyro[INS_AXIS_Y];
		updated.Gyro[INS_AXIS_Z] = BMI088.Gyro[INS_AXIS_Z];
		Gyro.x=BMI088.Gyro[0];
		Gyro.y=BMI088.Gyro[1];
		Gyro.z=BMI088.Gyro[2];

		mahony_input(&mahony,Gyro,Accel);
		mahony_update(&mahony);
		mahony_output(&mahony);
		RotationMatrix_update(&mahony);
				
		updated.q[0]=mahony.q0;
		updated.q[1]=mahony.q1;
		updated.q[2]=mahony.q2;
		updated.q[3]=mahony.q3;

		/* 预热期也持续发布姿态，但由 ins_flag 阻止控制任务提前使用。 */
		updated.Roll=mahony.pitch;
		updated.Pitch=mahony.roll;
		updated.Yaw=mahony.yaw;
	   
	  // 将重力从大地坐标系n转换到机体坐标系b，用于从加速度计数据中减去重力加速度，得到运动加速度
		float gravity_b[3];
		EarthFrameToBodyFrame(gravity, gravity_b, updated.q);
		for (uint8_t i = 0; i < 3; i++) { // 同时对三个轴进行低通滤波
		  updated.MotionAccel_b[i] = (updated.Accel[i] - gravity_b[i]) * ins_dt / (updated.AccelLPF + ins_dt)
										+ updated.MotionAccel_b[i] * updated.AccelLPF / (updated.AccelLPF + ins_dt);
//		  updated.MotionAccel_b[i] = (updated.Accel[i] ) * dt / (updated.AccelLPF + dt)
//										+ updated.MotionAccel_b[i] * updated.AccelLPF / (updated.AccelLPF + dt);
		}
		BodyFrameToEarthFrame(updated.MotionAccel_b, updated.MotionAccel_n, updated.q); // 转换回大地坐标系n
		
		//零点漂移
		if(fabsf(updated.MotionAccel_n[0])<0.02f) {
		  updated.MotionAccel_n[0]=0.0f;	//x轴
		}
		if(fabsf(updated.MotionAccel_n[1])<0.02f) {
		  updated.MotionAccel_n[1]=0.0f;	//y轴
		}
		if(fabsf(updated.MotionAccel_n[2])<0.04f) {
		  updated.MotionAccel_n[2]=0.0f;//z轴
		  stop_time++;
		}
//		if(stop_time>10)
//		{//静止10ms
//		  	stop_time=0;
//			updated.v_n=0.0f;
//		}
			
		now_ms = DWT_GetTimeMs();
		if ((uint32_t)(now_ms - ins_warmup_start_ms) >= INS_WARMUP_TIME_MS) {
			updated.v_n=updated.v_n+updated.MotionAccel_n[1]*0.001f;
			updated.x_n=updated.x_n+updated.v_n*0.001f;

			if (updated.ins_flag == 0U)
			{
				/* 预热结束时从当前偏航角开始计圈，忽略收敛过程中的跨界。 */
				updated.YawRoundCount = 0.0f;
				updated.YawAngleLast = updated.Yaw;
				updated.YawTotalAngle = updated.Yaw;
				updated.ins_flag = 1U;
			}
			else if (updated.Yaw - updated.YawAngleLast > 3.1415926f)
			{
				updated.YawRoundCount--;
			}
			else if (updated.Yaw - updated.YawAngleLast < -3.1415926f)
			{
				updated.YawRoundCount++;
			}
			updated.YawTotalAngle = 6.283f* updated.YawRoundCount + updated.Yaw;
			updated.YawAngleLast = updated.Yaw;
		}
		updated.last_update_time_ms = now_ms;
		taskENTER_CRITICAL();
		/* 只发布完成的一整周期，防止高优先级转发任务读取半更新姿态。 */
		INS = updated;
		taskEXIT_CRITICAL();
		
		osDelay(1);
	}
} 

/* Private functions ------------------------------------------------------- */
/**
 * @brief  将三维向量从机体坐标系转换到大地坐标系
 * @param  *vecBF: 机体坐标系下的向量
 * @param  *vecEF: 大地坐标系下的向量
 * @param  *q: 四元数
 * @return 无
 * @note   无
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief  将三维向量从大地坐标系转换到机体坐标系
 * @param  *vecEF: 大地坐标系下的向量
 * @param  *vecBF: 机体坐标系下的向量
 * @param  *q: 四元数
 * @return 无
 * @note   无
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/* Interrupt functions ----------------------------------------------------- */


/* ------------------------------------------------------------------------- */

