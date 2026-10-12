/**
 ******************************************************************************
 * @file    Remote_Control.h
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   板间转发遥控快照的数据定义
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 待测试
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

/* Includes ----------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* Defines ------------------------------------------------------------------ */
#define SBUS_RX_BUF_LEN		18u			/* SBUS接收数据长度 */

#define RC_CH_VALUE_OFFSET	1024U		/* 遥控器通道数据偏移 */
#define RC_CH_VALUE_MAX		660			/* 遥控器通道有效值上限 */
#define RC_CHANNEL_COUNT	4U			/* 遥控器摇杆通道数量 */

#define KEY_SET_SHORT_TIME	50U			/* 键盘置位短时间 */
#define KEY_SET_LONG_TIME	1000U		/* 键盘置位长时间 */

#define KEY_UP				0x00U		/* 键盘抬起状态 */
#define KEY_DOWN			0x01U		/* 键盘按下状态 */
/* 鼠标最大速度 */
#define MOUSE_SPEED_MAX		300U		
/* 遥控器拨杆状态 */
#define RC_SW_UP			1			/* 遥控器开关(上) */
#define RC_SW_MID			3			/* 遥控器开关(中) */
#define RC_SW_DOWN			2			/* 遥控器开关(下) */
#define RC_CH_DEADZONE		5U			/* 遥控器通道死区 */
/* 遥控器控制灵敏度 */
#define RC_CONTROL_SENSITIVITY	0.7f		
/* 遥控器拨杆 */
#define RC_SW_RIGHT			(remote_ctrl.rc.s[1])
#define RC_SW_LEFT			(remote_ctrl.rc.s[0])
/* 鼠标控制 */
#define MOUSE_X_MOVE_SPEED	(remote_ctrl.mouse.x)
#define MOUSE_Y_MOVE_SPEED	(remote_ctrl.mouse.y)
#define MOUSE_Z_MOVE_SPEED	(remote_ctrl.mouse.z)
#define MOUSE_PRESSED_LEFT	(remote_ctrl.mouse.press_l)
#define MOUSE_PRESSED_RIGHT	(remote_ctrl.mouse.press_r)
/* 键盘控制 */
#define KeyBoard_W		(remote_ctrl.key.set.W)
#define KeyBoard_S		(remote_ctrl.key.set.S)
#define KeyBoard_A		(remote_ctrl.key.set.A)
#define KeyBoard_D		(remote_ctrl.key.set.D)
#define KeyBoard_SHIFT	(remote_ctrl.key.set.SHIFT)
#define KeyBoard_CTRL	(remote_ctrl.key.set.CTRL)
#define KeyBoard_Q		(remote_ctrl.key.set.Q)
#define KeyBoard_E		(remote_ctrl.key.set.E)
#define KeyBoard_R		(remote_ctrl.key.set.R)
#define KeyBoard_F		(remote_ctrl.key.set.F)
#define KeyBoard_G		(remote_ctrl.key.set.G)
#define KeyBoard_Z		(remote_ctrl.key.set.Z)
#define KeyBoard_X		(remote_ctrl.key.set.X)
#define KeyBoard_C		(remote_ctrl.key.set.C)
#define KeyBoard_V		(remote_ctrl.key.set.V)
#define KeyBoard_B		(remote_ctrl.key.set.B)

/* Enums -------------------------------------------------------------------- */
/**
 * @brief 遥控器拨杆索引。
 * @note 与 rc.s[] 的存储顺序一致，SW2 对应 rc.s[1]。
 */
typedef enum
{
	REMOTE_SWITCH_SW1 = 0,
	REMOTE_SWITCH_SW2,
	REMOTE_SWITCH_COUNT
} Remote_SwitchIndex_e;

/**
 * @brief 遥控器信息结构体
 */
typedef  struct
{
	struct
	{
		int16_t ch[RC_CHANNEL_COUNT];
		uint8_t s[2];
		int16_t iw;		/* 拨轮线上值减 1024 后的 -660～+660 */
	} rc;

	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;

	union
	{
		uint16_t v;
		struct
		{
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
//			uint16_t Z:1;
//			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;
		} set;
	} key;

	bool rc_lost;			/* 丢失标志 */
	bool rc_active[RC_CHANNEL_COUNT];	/* 摇杆通道活动标志 */
	uint8_t online_cnt;		/* 在线计数 */
	uint32_t Last_Remote_Active_Time[RC_CHANNEL_COUNT];
} Remote_Info_t;

/* Externs ------------------------------------------------------------------ */
extern Remote_Info_t remote_ctrl;

/* -------------------------------------------------------------------------- */
#endif /* REMOTE_CONTROL_H */
