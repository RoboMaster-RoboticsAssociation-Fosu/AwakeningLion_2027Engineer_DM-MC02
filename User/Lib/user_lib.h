/**
 ******************************************************************************
 * @file    user_lib.h
 * @version V1.0.0
 * @date    2026.03.05
 * @brief   提供一些通用的工具函数和宏定义
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef _USER_LIB_H
#define _USER_LIB_H

/* Includes ----------------------------------------------------------------- */
#include "stdint.h"
#include "main.h"
#include "cmsis_os.h"

/* Defines ------------------------------------------------------------------ */
/* 调试模式选择 */
#define CHASSIS_DEBUG		1
#define GIMBAL_DEBUG		2
#define INS_DEBUG			3
#define RC_DEBUG			4
#define IMU_HEAT_DEBUG		5
#define SHOOT_DEBUG			6
#define AIMASSIST_DEBUG		7

/* 内存申请宏 */
#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc		pvPortMalloc
#else
#define user_malloc		malloc
#endif
#endif

/* 布尔类型定义 */
#ifndef TRUE
#define TRUE		1
#endif
#ifndef FALSE
#define FALSE		0
#endif

/* 数学相关 */
#ifndef RADIAN_COEF
#define RADIAN_COEF		57.295779513f	/* 弧度转角度系数 */
#endif
#ifndef PI
#define PI		        3.14159265354f	/* 圆周率 */
#endif

/* 功能宏 */
#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)
#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)
#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)
#define VAL_MIN(a, b)		(((a) < (b)) ? (a) : (b))
#define VAL_MAX(a, b)		(((a) > (b)) ? (a) : (b))
#define rad_format(Ang)		loop_float_constrain((Ang), -PI, PI)	/* 弧度格式化为-PI~PI */

/* Enums -------------------------------------------------------------------- */
/**
 * @brief 全局调试模式枚举
 */
enum
{
    CHASSIS_DEBUG_MODE = 1,	/* 底盘调试模式 */
    GIMBAL_DEBUG_MODE,		/* 云台调试模式 */
    INS_DEBUG_MODE,			/* INS调试模式 */
    RC_DEBUG_MODE,			/* 遥控器调试模式 */
    IMU_HEAT_DEBUG_MODE,	/* IMU加热调试模式 */
    SHOOT_DEBUG_MODE,		/* 发射机构调试模式 */
    AIMASSIST_DEBUG_MODE,	/* 自动瞄准调试模式 */
};

/* Structs ------------------------------------------------------------------ */
/**
 * @brief 斜坡函数源数据结构体
 */
typedef struct
{
    float input;		/* 输入值 */
    float out;			/* 输出值 */
    float min_value;	/* 限幅最小值 */
    float max_value;	/* 限幅最大值 */
    float frame_period;	/* 时间间隔 */
} ramp_function_source_t;

/**
 * @brief 最小二乘法结构体
 */
typedef __packed struct
{
    uint16_t Order;				/* 阶数 */
    uint32_t Count;				/* 计数 */

    float *x;					/* x缓存 */
    float *y;					/* y缓存 */

    float k;					/* 斜率 */
    float b;					/* 截距 */

    float StandardDeviation;	/* 标准差 */

    float t[4];					/* 临时变量 */
} Ordinary_Least_Squares_t;

/* Externs ------------------------------------------------------------------ */
extern uint8_t GlobalDebugMode;	/* 全局调试模式 */

/* Functions ---------------------------------------------------------------- */
float Sqrt(float x);
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);
float ramp_calc(ramp_function_source_t *ramp_source_type, float input);
float abs_limit(float num, float Limit);
float sign(float value);
float float_deadband(float Value, float minValue, float maxValue);
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
float float_constrain(float Value, float minValue, float maxValue);
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
float loop_float_constrain(float Input, float minValue, float maxValue);
float theta_format(float Ang);
int float_rounding(float raw);
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS);
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS);

/* -------------------------------------------------------------------------- */
#endif /* _USER_LIB_H */
