/**
 * @file ins_types.h
 * @brief 板间姿态快照的数据契约；不依赖本地 IMU 驱动或滤波算法。
 * @note 底盘 INS 任务发布，从臂 BoardLink 接收；字段、单位和线协议保持一致。
 */
#ifndef INS_TYPES_H
#define INS_TYPES_H

#include <stdint.h>

/** @brief 完整姿态快照；时间戳属于源板，不能与接收板时钟直接比较。 */
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

extern INS_t INS;

#endif /* INS_TYPES_H */
