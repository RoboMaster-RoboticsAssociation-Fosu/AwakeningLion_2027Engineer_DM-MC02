/**
 ******************************************************************************
 * @file    user_lib.c
 * @version V1.0.0
 * @date    2026.03.05
 * @brief   用户自定义库函数实现
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "stdlib.h"
#include "string.h"
#include "user_lib.h"
#include "math.h"
#include "main.h"

/* Defines ----------------------------------------------------------------- */
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

/* Global variable --------------------------------------------------------- */
uint8_t GlobalDebugMode = 7;	// 全局调试模式

/* Static Fun -------------------------------------------------------------- */

/* Functions --------------------------------------------------------------- */
/**
  * @brief  快速开方
  * @param  x: 输入值
  * @return float: 计算结果
  * @note   使用牛顿迭代法
  */
float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;
    if (x <= 0){
        return 0;
    }
    // initial guess
    y = x / 2;
    // refine
    maxError = x * 0.001f;
    do{
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

/*
//快速求平方根倒数
float invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f375a86- (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}*/

/**
  * @brief  斜波函数初始化
  * @param  ramp_source_type: 指向斜波函数结构体的指针
  * @param  frame_period: 间隔时间 (s)
  * @param  max: 最大值
  * @param  min: 最小值
  * @return 无
  * @note   无
  */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief  斜波函数计算
  * @param  ramp_source_type: 指向斜波函数结构体的指针
  * @param  target: 目标值
  * @return float: 计算输出
  * @note   根据输入的目标值进行追赶，斜率由input成员决定
  */
float ramp_calc(ramp_function_source_t *ramp_source_type, float target)
{
    // 计算误差
    float error = target - ramp_source_type->out;
    
    // 计算最大允许步进（速度限制）
    float max_step = ramp_source_type->input * ramp_source_type->frame_period;  // 这里input应该是最大速度
    
    // 限制步进量
    if (fabsf(error) > max_step) {
        if (error > 0)
            ramp_source_type->out += max_step;
        else
            ramp_source_type->out -= max_step;
    } else {
        ramp_source_type->out = target;  // 直接到达目标
    }
    
    // 限幅
    if (ramp_source_type->out > ramp_source_type->max_value) {
        ramp_source_type->out = ramp_source_type->max_value;
    } else if (ramp_source_type->out < ramp_source_type->min_value) {
        ramp_source_type->out = ramp_source_type->min_value;
    }
    
    return ramp_source_type->out;
}

/**
  * @brief  绝对值限幅
  * @param  num: 输入值
  * @param  Limit: 限幅值
  * @return float: 输出值
  * @note   无
  */
float abs_limit(float num, float Limit)
{
    if (num > Limit)
    {
        num = Limit;
    }
    else if (num < -Limit)
    {
        num = -Limit;
    }
    return num;
}

/**
  * @brief  判断符号
  * @param  value: 输入值
  * @return float: 1.0f for >= 0, -1.0f for < 0
  * @note   无
  */
float sign(float value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

/**
  * @brief  浮点数死区
  * @param  Value: 输入值
  * @param  minValue: 死区下限
  * @param  maxValue: 死区上限
  * @return float: 输出值
  * @note   在(minValue, maxValue)区间内的值将被置零
  */
float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

/**
  * @brief  16位整型死区
  * @param  Value: 输入值
  * @param  minValue: 死区下限
  * @param  maxValue: 死区上限
  * @return int16_t: 输出值
  * @note   在(minValue, maxValue)区间内的值将被置零
  */
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

/**
  * @brief  浮点数限幅
  * @param  Value: 输入值
  * @param  minValue: 下限
  * @param  maxValue: 上限
  * @return float: 输出值
  * @note   无
  */
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

/**
  * @brief  16位整型限幅
  * @param  Value: 输入值
  * @param  minValue: 下限
  * @param  maxValue: 上限
  * @return int16_t: 输出值
  * @note   无
  */
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

/**
  * @brief  循环限幅
  * @param  Input: 输入值
  * @param  minValue: 下限
  * @param  maxValue: 上限
  * @return float: 输出值
  * @note   将输入值限制在[minValue, maxValue]区间内循环
  */
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

/**
  * @brief  角度格式化
  * @param  Ang: 输入角度
  * @return float: 输出角度
  * @note   将角度格式化到[-180, 180]度
  */
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

/**
  * @brief  浮点数四舍五入
  * @param  raw: 输入浮点数
  * @return int: 输出整数
  * @note   无
  */
int float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

/**
  * @brief  最小二乘法初始化
  * @param  OLS: 指向最小二乘法结构体的指针
  * @param  order: 样本数
  * @return 无
  * @note   无
  */
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order)
{
    OLS->Order = order;
    OLS->Count = 0;
    OLS->x = (float *)user_malloc(sizeof(float) * order);
    OLS->y = (float *)user_malloc(sizeof(float) * order);
    OLS->k = 0;
    OLS->b = 0;
    memset((void *)OLS->x, 0, sizeof(float) * order);
    memset((void *)OLS->y, 0, sizeof(float) * order);
    memset((void *)OLS->t, 0, sizeof(float) * 4);
}

/**
  * @brief  最小二乘法拟合更新
  * @param  OLS: 指向最小二乘法结构体的指针
  * @param  deltax: 信号新样本距上一个样本时间间隔
  * @param  y: 信号值
  * @return 无
  * @note   无
  */
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }
    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;
}

/**
  * @brief  最小二乘法提取信号微分
  * @param  OLS: 指向最小二乘法结构体的指针
  * @param  deltax: 信号新样本距上一个样本时间间隔
  * @param  y: 信号值
  * @return float: 斜率k
  * @note   无
  */
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k;
}

/**
  * @brief  获取最小二乘法提取的信号微分
  * @param  OLS: 指向最小二乘法结构体的指针
  * @return float: 斜率k
  * @note   无
  */
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k;
}

/**
  * @brief  最小二乘法平滑信号
  * @param  OLS: 指向最小二乘法结构体的指针
  * @param  deltax: 信号新样本距上一个样本时间间隔
  * @param  y: 信号值
  * @return float: 平滑后的输出
  * @note   无
  */
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

/**
  * @brief  获取最小二乘法平滑后的信号
  * @param  OLS: 指向最小二乘法结构体的指针
  * @return float: 平滑后的输出
  * @note   无
  */
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

/* Private functions ------------------------------------------------------- */

/* Interrupt functions ----------------------------------------------------- */

/* ------------------------------------------------------------------------- */
