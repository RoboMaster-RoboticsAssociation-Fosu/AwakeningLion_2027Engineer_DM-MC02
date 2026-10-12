/**
 ******************************************************************************
 * @file    arm_custom_grip.h
 * @brief   自定义控制器夹爪按压方向状态机。
 ******************************************************************************
 */

#ifndef ARM_CUSTOM_GRIP_H
#define ARM_CUSTOM_GRIP_H

#include <stdbool.h>

/** @brief 按压交替方向、松开保持和掉线重连所需的最小跨周期状态。 */
typedef struct
{
    bool input_online;
    bool button_armed;
    bool previous_button_pressed;
    bool closing_on_next_press;
    bool active_direction_closing;
} ArmCustomGripState_t;

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 新进入CUSTOM模式时复位为“下一次按下闭合”。 */
void ArmCustomGrip_Reset(ArmCustomGripState_t *state);

/** @brief 停止当前按压并要求重新松开，保留下次运动方向。 */
void ArmCustomGrip_Disarm(ArmCustomGripState_t *state);

/**
 * @brief 根据按钮电平推进一周期夹爪目标。
 * @note 控制器掉线时返回当前已应用目标并保留按钮历史；重连后根据当前
 *       电平续动或识别新边沿。ArmCustomGrip_Disarm()仍要求重新松开。
 * @return true表示输出有效；参数非法时返回false且状态和输出保持不变。
 */
bool ArmCustomGrip_Update(ArmCustomGripState_t *state,
                          bool input_online,
                          bool button_pressed,
                          float applied_target_rad,
                          float desired_target_rad,
                          float step_rad,
                          float minimum_target_rad,
                          float maximum_target_rad,
                          float *output_target_rad);

#ifdef __cplusplus
}
#endif

#endif /* ARM_CUSTOM_GRIP_H */
