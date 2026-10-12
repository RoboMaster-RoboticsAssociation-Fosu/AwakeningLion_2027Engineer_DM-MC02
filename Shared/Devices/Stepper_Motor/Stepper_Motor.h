/**
 ******************************************************************************
 * @file    Stepper_Motor.h
 * @version V1.0.0
 * @date    2026.04.11
 * @brief   步进电机驱动函数声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

/* Includes ----------------------------------------------------------------- */
#include "cmsis_os.h"

/* Defines ------------------------------------------------------------------ */
#define STEPPER_MOTOR_RX_BUF_LEN		31U			/* 接收数据长度 */
#define STEPPER_RX_CODE					0x43		/* 读取数据指令码 */
#define STEPPER_CMD_CODE				0xF3		/* 电机使能指令码 */
#define STEPPER_CAL_CODE				0x06		/* 校准指令码 */
#define STEPPER_SPEED_CODE				0xF6		/* 速度模式控制指令码 */
#define STEPPER_POS_CODE				0xFD		/* 位置控制指令码 */
#define STEPPER_POS_CURRENT_CODE		0xCB		/* 限流位置控制指令码 */
#define STEPPER_POS_STOP_CODE			0xFE		/* 位置停止指令码 */
#define STEPPER_ZERO_CMD_CODE			0x9A		/* 回零指令码 */
#define STEPPER_ZERO_RX_CODE			0x22		/* 读取回零参数指令码 */
#define STEPPER_ZERO_SET_CODE			0x4C		/* 设置回零参数指令码 */
#define STEPPER_CHECK_CODE				0x6B		/* 通信校验码 */
#define Stepper_Ratio					(13.7f / 1.0f)	/* 步进电机减速比 */

/* Enums -------------------------------------------------------------------- */
/**
 * @brief 步进电机固件类型枚举
 */
typedef enum
{
    Firmware_Emm = 0,		/* 标准固件 */
    Firmware_X = 1,			/* X固件 */
} Stepper_Motor_Type_e;

/**
 * @brief 步进电机数据更新状态枚举
 */
typedef enum
{
    Motor_Update_Success = 1,	/* 数据更新成功 */
    Motor_Update_Fail    = 0,	/* 数据更新失败 */
} Stepper_Motor_Update_e;

/**
 * @brief 步进电机使能控制枚举
 */
typedef enum
{
    Stepper_Enable  = 0x01,		/* 电机使能 */
    Stepper_Disable = 0x00,		/* 电机失能 */         
} Stepper_Motor_Cmd_e;

/**
 * @brief 步进电机转动方向枚举
 */
typedef enum
{
    Stepper_CW  = 0x00,			/* 顺时针方向 */
    Stepper_CCW = 0x01,			/* 逆时针方向 */
} Stepper_Motor_Dir_e;

/**
 * @brief 步进电机指令返回状态枚举
 */
typedef enum
{
    Rx_Ture             = 0x02,	/* 接收命令正确 */
    Rx_Zero_Triggered   = 0x12,	/* 回零触发/在零点 */
    Rx_Param_Error      = 0xE2,	/* 参数错误/保护触发 */
    Rx_Format_Error     = 0xEE,	/* 命令格式错误 */
    Rx_Action_Finished  = 0x9F,	/* 动作执行完成 */
} Stepper_Motor_Return_e;

/**
 * @brief 步进电机回零模式枚举
 */
typedef enum
{
    Zero_Single_Closest         = 00,	/* 单圈就近回零（默认） */
    Zero_Single_Dir             = 01,	/* 单圈方向回零 */
    Zero_No_Limit_Collision     = 02,	/* 无限位碰撞回零 */
    Zero_Limit_Collision        = 03,	/* 限位回零 */
    Zero_Abs                    = 04,	/* 绝对位置零点 */
    Zero_Last_Power_Off         = 05	/* 回到上次掉电位置 */
} Stepper_Motor_Zero_Mode_e;

/**
 * @brief 步进电机位置模式枚举
 */
typedef enum
{
    Pos_Mode_Rel_To_LastTarget = 0x00,	/* 相对上一目标位置运动 */
    Pos_Mode_Abs_To_Zero      = 0x01,	/* 相对坐标零点绝对运动 */
    Pos_Mode_Rel_To_Current   = 0x02	/* 相对当前位置相对运动 */
} Stepper_Motor_Pos_Mode_e;

/* Structs ------------------------------------------------------------------ */
/**
 * @brief 回零状态联合体
 */
typedef union
{
    uint8_t byte;               /* 整字节状态 */
    struct
    {
        uint8_t Enc_Rdy  : 1;   /* 编码器就绪标志 */
        uint8_t Cal_Rdy  : 1;   /* 校准表就绪标志 */
        uint8_t Org_SF   : 1;   /* 正在回零标志 */
        uint8_t Org_CF   : 1;   /* 回零失败标志 */
        uint8_t Otp_TF   : 1;   /* 过热保护标志 */
        uint8_t Ocp_TF   : 1;   /* 过流保护标志 */
        uint8_t Reserved1 : 1;  /* 保留位1 */
        uint8_t Reserved2 : 1;  /* 保留位2 */
    } bits;
} Zero_Status_t;

/**
 * @brief 电机运行状态联合体
 */
typedef union
{
    uint8_t byte;             /* 整字节状态 */
    struct
    {
        uint8_t Ens_TF  : 1;  /* 使能状态标志 */
        uint8_t Prf_TF  : 1;  /* 回零完成标志 */
        uint8_t Cgi_TF  : 1;  /* 堵转保护标志 */
        uint8_t Cgp_TF  : 1;  /* 过流/过热保护标志 */
        uint8_t Esi_LF  : 1;  /* 左限位触发标志 */
        uint8_t Esi_RF  : 1;  /* 右限位触发标志 */
        uint8_t Reserved : 1; /* 保留位 */
        uint8_t Oac_TF  : 1;  /* 动作完成/运行中标志 */
    } bits;
} Motor_Status_t;

/**
 * @brief 步进电机回零数据结构体
 */
typedef struct
{
    Stepper_Motor_Zero_Mode_e zero_mode;		/* 回零模式 */
    Stepper_Motor_Dir_e zero_dir;				/* 回零方向 */
    float zero_out_time;						/* 回零超时时间 */
    float zero_speed;							/* 回零速度 */
    float zero_detect_speed;					/* 回零检测速度 */
    float zero_detect_current;					/* 回零检测电流 */
    float zero_detect_time;						/* 回零检测时间 */
    uint8_t zero_POT_En;						/* 上电自动回零使能 */
} Stepper_Motor_Zero_Data_t;

/**
 * @brief 步进电机接收数据结构体
 */
typedef struct 
{
    uint8_t  motor_Addr;         /* 从机地址 */
    uint16_t bus_v_u16;          /* 总线电压 */
    uint16_t phase_c_u16;        /* 电机相电流 */
    uint16_t line_encoder_u16;   /* 线性编码器值 */
    uint8_t  sign1;              /* 目标位置符号位 */
	uint32_t target_pos_u32;     /* 目标位置 */
    uint8_t  sign2;              /* 转速符号位 */
	uint16_t speed_u16;      	 /* 实时转速 */
    uint8_t  sign3;              /* 位置符号位 */
    uint32_t pos_u32;            /* 实时位置 */
    uint8_t  sign4;              /* 位置误差符号位 */
    uint32_t pos_error_u32;      /* 位置误差 */
    uint8_t  zero_s;             /* 回零状态 */
    uint8_t  motor_s;            /* 电机状态 */
    uint8_t zero_mode;           /* 回零模式 */
    uint8_t zero_dir;            /* 回零方向 */
    uint16_t zero_speed_u16;     /* 回零速度 */
    uint32_t zero_out_time_u32;  /* 回零超时 */
    uint16_t zero_detect_speed_u16;		/* 回零检测速度 */
    uint16_t zero_detect_current_u16;	/* 回零检测电流 */
    uint16_t zero_detect_time_u16;		/* 回零检测时间 */
    uint8_t zero_POT_En;         /* 上电回零使能 */
    uint8_t Return_state_Cmd;    /* 使能指令返回状态 */
    uint8_t Return_state_Cal;    /* 校准指令返回状态 */
    uint8_t Return_state_Pos;    /* 位置指令返回状态 */
    uint8_t Return_state_Zero_Cmd;		/* 回零指令返回状态 */
    uint8_t Return_state_Zero_Set;		/* 回零设置返回状态 */
    uint8_t Return_state_Pos_Stop;		/* 停止指令返回状态 */
} Stepper_Motor_RxInfo_t;

/**
 * @brief 步进电机指令返回状态结构体
 */
typedef struct
{
    Stepper_Motor_Return_e Motor_Cmd;		/* 使能指令返回状态 */
    Stepper_Motor_Return_e Motor_Cal;		/* 校准指令返回状态 */
    Stepper_Motor_Return_e Motor_Pos;		/* 位置指令返回状态 */
    Stepper_Motor_Return_e Motor_Zero_Cmd;	/* 回零指令返回状态 */
    Stepper_Motor_Return_e Motor_Zero_Set;	/* 回零设置返回状态 */
    Stepper_Motor_Return_e Motor_Pos_Stop;	/* 停止指令返回状态 */
} Return_Status_t;

/**
 * @brief 步进电机数据结构体
 */
typedef struct 
{
    uint8_t Rx_Addr;						/* 接收地址 */
    float bus_voltage;						/* 总线电压 */
    float phase_current;					/* 相电流 */
    float line_encoder_value;				/* 编码器值 */
    float target_pos;						/* 目标位置 */
    float speed;							/* 实时转速 */
    float pos;								/* 实时位置 */
    float pos_error;						/* 位置误差 */
    Zero_Status_t zero_status;		/* 回零状态 */
    Motor_Status_t motor_status;		/* 电机状态 */
    Return_Status_t return_status;	/* 指令返回状态 */
} Stepper_Motor_Data_t;

/**
 * @brief 步进电机回零参数设置结构体
 */
typedef struct 
{
    uint8_t zero_save;								/* 回零参数保存 */
    Stepper_Motor_Zero_Mode_e zero_mode;			/* 回零模式 */
    Stepper_Motor_Dir_e zero_dir;					/* 回零方向 */
    uint16_t zero_speed;							/* 回零速度 */
    uint32_t zero_out_time;							/* 回零超时 */
    uint16_t zero_detect_speed;						/* 回零检测速度 */
    uint16_t zero_detect_current;					/* 回零检测电流 */
    uint16_t zero_detect_time;						/* 回零检测时间 */
    uint8_t zero_POT_En;							/* 上电回零使能 */
} Stepper_Motor_Zero_Set_t;

/**
 * @brief 步进电机参数设置结构体
 */
typedef struct 
{
    uint8_t motor_Addr;								/* 电机地址 */
    float Pos_Speed;								/* 位置速度 */
    Stepper_Motor_Type_e Firmware_v;				/* 固件版本 */
    Stepper_Motor_Zero_Set_t Zero_Set;		/* 回零参数设置 */
} Stepper_Motor_Set_t;

/**
 * @brief 步进电机总信息结构体
 */
typedef struct 
{
    Stepper_Motor_Update_e Motor_Update_Flag;		/* 数据更新标志 */
    Stepper_Motor_Set_t Set;					/* 参数设置 */
    Stepper_Motor_Data_t Data;				/* 运行数据 */
    Stepper_Motor_Zero_Data_t Zero_Data;		/* 回零数据 */
} Stepper_Motor_Info_t;

/* Externs ------------------------------------------------------------------ */
extern uint8_t Stepper_MultiRx_Buf[2][STEPPER_MOTOR_RX_BUF_LEN];
extern Stepper_Motor_RxInfo_t Stepper_Rx_Info;

/* Functions ---------------------------------------------------------------- */
void Stepper_Motor_Set_Cmd(Stepper_Motor_Info_t *Motor, Stepper_Motor_Cmd_e Cmd, uint32_t Delay_Time);
void Stepper_Motor_Set_Speed(Stepper_Motor_Info_t *Motor, int16_t Speed, uint8_t Accel, uint32_t Delay_Time);
void Stepper_Motor_Set_Pos(Stepper_Motor_Info_t *Motor,uint16_t Speed,uint8_t Ac_speed, float Target_pos, Stepper_Motor_Pos_Mode_e Pose_Mode, uint32_t Delay_Time);
void Stepper_Motor_Set_Pos_Current(Stepper_Motor_Info_t *Motor, uint16_t Speed, float Target_pos, Stepper_Motor_Pos_Mode_e Pose_Mode, uint16_t Max_Current, uint32_t Delay_Time);
void Stepper_Motor_Set_Pos_Stop(Stepper_Motor_Info_t *Motor, uint32_t Delay_Time);
void Stepper_Motor_Set_Zero_Cmd(Stepper_Motor_Info_t *Motor, Stepper_Motor_Zero_Mode_e Zero_Mode, uint32_t Delay_Time);
void Stepper_Motor_Set_Zero(Stepper_Motor_Info_t *Motor, uint32_t Delay_Time);
void Stepper_Motor_Call_Cal(Stepper_Motor_Info_t *Motor, uint32_t Delay_Time);
void Stepper_Motor_Call_Info(Stepper_Motor_Info_t *Motor, uint32_t Delay_Time);
void Stepper_RxInfo_Unpack(uint8_t *Rx_Buf, Stepper_Motor_RxInfo_t *RxInfo);

/* -------------------------------------------------------------------------- */
#endif /* __STEPPER_MOTOR_H */
