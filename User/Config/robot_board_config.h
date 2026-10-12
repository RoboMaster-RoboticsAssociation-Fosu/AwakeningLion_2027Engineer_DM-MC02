/**
 * @file robot_board_config.h
 * @brief 双主板职责选择。两板使用同一工程，编译时只启用所属控制链路。
 * @note 可在编译器 Defines 中设置 ROBOT_BOARD_ROLE=ROBOT_BOARD_CHASSIS。
 *       未设置时编译机械臂板；两板 USART10 线协议必须保持一致。
 */
#ifndef ROBOT_BOARD_CONFIG_H
#define ROBOT_BOARD_CONFIG_H

#define ROBOT_BOARD_ARM 1
#define ROBOT_BOARD_CHASSIS 2

#ifndef ROBOT_BOARD_ROLE
#define ROBOT_BOARD_ROLE ROBOT_BOARD_ARM
#endif

#if (ROBOT_BOARD_ROLE != ROBOT_BOARD_ARM) && (ROBOT_BOARD_ROLE != ROBOT_BOARD_CHASSIS)
#error "ROBOT_BOARD_ROLE must select ROBOT_BOARD_ARM or ROBOT_BOARD_CHASSIS"
#endif

#endif /* ROBOT_BOARD_CONFIG_H */
