/**
 ******************************************************************************
 * @file    Remote_Control.h
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   遥控器数据处理函数声明
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
 * @brief SW1/SW2 切换到上、中、下目标位置的边沿位。
 * @note 多个不同边沿可以在消费者读取前累积在同一个位掩码中。
 */
typedef enum
{
	RC_SWITCH_EDGE_NONE = 0U,
	RC_SWITCH_EDGE_TO_UP = (1UL << 0U),
	RC_SWITCH_EDGE_TO_MID = (1UL << 1U),
	RC_SWITCH_EDGE_TO_DOWN = (1UL << 2U)
} RC_SwitchEdge_e;

/**
 * @brief 判断拨杆本次轮询是否切换到指定目标位置。
 * @param edges RC_EdgeCursor_t::switch_edges[] 中的边沿位掩码。
 */
#define RC_SWITCH_MOVED_TO_UP(edges) \
	((((uint32_t)(edges)) & ((uint32_t)RC_SWITCH_EDGE_TO_UP)) != 0U)
#define RC_SWITCH_MOVED_TO_MID(edges) \
	((((uint32_t)(edges)) & ((uint32_t)RC_SWITCH_EDGE_TO_MID)) != 0U)
#define RC_SWITCH_MOVED_TO_DOWN(edges) \
	((((uint32_t)(edges)) & ((uint32_t)RC_SWITCH_EDGE_TO_DOWN)) != 0U)

/**
 * @brief iw 拨轮跨越上、下阈值时的全部有向边沿位。
 * @note iw 已减去 1024，上区为 iw <= -330，下区为 iw >= 330；
 *       区间中部为中立区域。
 */
typedef enum
{
	RC_IW_EDGE_NONE = 0U,
	RC_IW_EDGE_ENTER_UP = (1UL << 0U),
	RC_IW_EDGE_LEAVE_UP = (1UL << 1U),
	RC_IW_EDGE_ENTER_DOWN = (1UL << 2U),
	RC_IW_EDGE_LEAVE_DOWN = (1UL << 3U)
} RC_IwEdge_e;

/**
 * @brief 单个任务独立持有的遥控器边沿游标和本次轮询结果。
 * @note 同类重复边沿合并为同一位，不保存发生次数和先后顺序。
 *       初始化后只能由所属任务读写，调用方不得修改 seen_sequence。
 */
typedef struct
{
	uint64_t seen_sequence;
	uint32_t switch_edges[REMOTE_SWITCH_COUNT];
	uint32_t iw_edges;
} RC_EdgeCursor_t;

/**
 * @brief 键盘状态枚举
 */
typedef enum
{
	UP,			/*!< 抬起 */
	SHORT_DOWN,	/*!< 短按 */
	DOWN,		/*!< 长按 */
	PRESS,		/*!< 按下 */
	RELAX,		/*!< 松开 */
	KeyBoard_Status_NUM,
}KeyBoard_Status_e;

/* Structs ------------------------------------------------------------------ */
/**
 * @brief 键盘信息结构体
 */
typedef struct
{
	uint16_t Count;
	KeyBoard_Status_e Status;
	KeyBoard_Status_e last_Status;
	bool last_KEY_PRESS;
	bool KEY_PRESS;
}KeyBoard_Info_t;

/**
 * @brief 遥控器按键结构体
 */
typedef struct
{
	KeyBoard_Info_t press_l;
	KeyBoard_Info_t press_r;
	KeyBoard_Info_t W;
	KeyBoard_Info_t S;
	KeyBoard_Info_t A;
	KeyBoard_Info_t D;
	KeyBoard_Info_t SHIFT;
	KeyBoard_Info_t CTRL;
	KeyBoard_Info_t Q;
	KeyBoard_Info_t E;
	KeyBoard_Info_t R;
	KeyBoard_Info_t F;
	KeyBoard_Info_t G;
//	KeyBoard_Info_t Z;
//	KeyBoard_Info_t X;
	KeyBoard_Info_t C;
	KeyBoard_Info_t V;
	KeyBoard_Info_t B;
}Remote_Pressed_t;

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

/* Functions ---------------------------------------------------------------- */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_t *remote_ctrl);
void Remote_Active_Detect( Remote_Info_t  *remote_ctrl);
void Remote_Offline_Detect( Remote_Info_t  *remote_ctrl);

/**
 * @brief 从当前发布位置初始化任务私有边沿游标。
 * @param cursor 调用任务独占的游标。
 * @note 初始化前已经发生的边沿不会回放；仅供 FreeRTOS 任务上下文调用。
 */
void Remote_EdgeCursor_Init(RC_EdgeCursor_t *cursor);

/**
 * @brief 更新该任务自上次轮询后出现过的全部边沿并推进自己的游标。
 * @param cursor 已初始化且由调用任务独占的游标。
 * @note 结果写入 cursor->switch_edges[] 和 cursor->iw_edges；
 *       不清除其他任务可见的边沿；仅供 FreeRTOS 任务上下文调用。
 */
void Remote_EdgeCursor_Poll(RC_EdgeCursor_t *cursor);

/**
 * @brief 发布同一有效 DBUS 帧产生的全部边沿。
 * @param sw1_edges 本帧 SW1 边沿。
 * @param sw2_edges 本帧 SW2 边沿。
 * @param iw_edges 本帧 iw 边沿。
 * @note 仅供 rc_task UART5 DMA 中断路径调用；不得从任务或其他中断调用。
 */
void Remote_Edge_PublishFromISR(uint32_t sw1_edges,
                                uint32_t sw2_edges,
                                uint32_t iw_edges);

/**
 * @brief 废弃所有尚未读取的历史边沿。
 * @note 仅供 rc_task 初始化和离线处理调用；不会回退全局发布序号。
 */
void Remote_Edge_Reset(void);

/* -------------------------------------------------------------------------- */
#endif /* REMOTE_CONTROL_H */
