/** @file ins_task.h @brief 底盘本地 BMI088 姿态采样和解算任务接口。 */
#ifndef INS_TASK_H
#define INS_TASK_H

#include "ins_types.h"

/** @brief 初始化 Mahony 滤波器、预热计时和任务时间基准。 */
void InsTask_Init(void);
/** @brief 周期采样 BMI088 并发布完整 INS 快照；预热期保留无效标志。 */
void InsTask_Run(void);
/** @brief 用四元数 q 将三维机体向量 vecBF 转换到大地向量 vecEF。 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
/** @brief 用四元数 q 将三维大地向量 vecEF 转换到机体向量 vecBF。 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif /* INS_TASK_H */
