/**
 ******************************************************************************
 * @file    vofa_system.h
 * @brief   本板控制任务的固定 VOFA 观测接口。
 ******************************************************************************
 */

#ifndef VOFA_SYSTEM_H
#define VOFA_SYSTEM_H

/** @brief 编译期可选的VOFA通道布局。 */
typedef enum
{

    /**
     * @brief 四个麦轮的速度闭环调试量。
     * @note 每轮占3通道：目标rpm、反馈rpm、C620电流指令；轮序与
     *       ChassisWheel_e一致，共12通道。
     */
    VOFA_VIEW_CHASSIS_SPEED_PID = 1,

    /**
     * @brief DBUS遥控器模拟量。
     * @note I0..I3为CH0..CH3，I4为iw；均为减去中心值后的有符号量。
     */
    VOFA_VIEW_REMOTE_INPUT = 2,

    /**
     * @brief INS姿态、角速度和加速度。
     * @note I0..I2为Roll/Pitch/Yaw(rad)，I3..I5为Gyro XYZ(rad/s)，
     *       I6..I8为Accel XYZ(m/s^2)。
     */
    VOFA_VIEW_INS_ATTITUDE = 3,



    /** @brief 视图数量边界，仅用于完整性检查，不能作为有效布局。 */
    VOFA_VIEW_COUNT = 6
} VofaView_e;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 按vofa_system.c中的VOFA_ACTIVE_VIEW上传一套固定通道。
 * @note 由ServiceTask每10 ms调用；切换视图需改宏并重新编译。
 */
void VofaSystem_Step(void);

#ifdef __cplusplus
}
#endif

#endif /* VOFA_SYSTEM_H */
