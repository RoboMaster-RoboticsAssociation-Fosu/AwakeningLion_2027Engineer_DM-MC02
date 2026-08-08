/**
 ******************************************************************************
 * @file    HWT6033.c
 * @version V1.0.0
 * @date    2026.07.14
 * @brief   HWT6033-TTL 姿态传感器的 STM32H7 适配实现
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 协议解析由维特智能官方 wit_c_sdk 完成，本文件只负责 HAL 适配与单位换算。
 * * 官方 SDK 为单实例设计，同一时刻只能绑定一个 WIT 标准协议传感器。
 ******************************************************************************
 */

/* Includes ----------------------------------------------------------------- */
#include "HWT6033.h"
#include "bsp_dwt.h"
#include "wit_c_sdk.h"
#include <limits.h>
#include <string.h>

/* Defines ------------------------------------------------------------------ */
#define HWT6033_ACCELERATION_SCALE_G       (16.0f / 32768.0f)
#define HWT6033_ANGULAR_VELOCITY_SCALE_DPS (2000.0f / 32768.0f)
#define HWT6033_ANGLE_SCALE_DEG             (180.0f / 32768.0f)
#define HWT6033_TEMPERATURE_SCALE_C         (1.0f / 100.0f)

/* Global variable ---------------------------------------------------------- */
static UART_HandleTypeDef *hwt6033_uart = NULL;
static HWT6033_Data_t hwt6033_data;
static volatile HAL_StatusTypeDef hwt6033_last_tx_status = HAL_OK;
static volatile uint8_t hwt6033_initialized = 0U;

/* Static functions --------------------------------------------------------- */
static void HWT6033_UartTransmit(uint8_t *data, uint32_t length);
static void HWT6033_DelayMs(uint16_t delay_ms);
static void HWT6033_RegisterUpdated(uint32_t register_start, uint32_t register_count);
static uint8_t HWT6033_RangeOverlaps(uint32_t update_start,
                                     uint32_t update_count,
                                     uint32_t target_start,
                                     uint32_t target_count);
static uint32_t HWT6033_EnterCritical(void);
static void HWT6033_ExitCritical(uint32_t primask);

/* Functions ---------------------------------------------------------------- */
HWT6033_Status_e HWT6033_Init(UART_HandleTypeDef *huart)
{
    int32_t sdk_status;

    if (huart == NULL)
    {
        return HWT6033_STATUS_INVALID_ARGUMENT;
    }

    /* 官方 SDK 保存全局回调，重新初始化前必须清除上一次绑定。 */
    WitDeInit();
    memset(&hwt6033_data, 0, sizeof(hwt6033_data));
    memset(sReg, 0, sizeof(sReg));
    hwt6033_uart = huart;
    hwt6033_last_tx_status = HAL_OK;

    sdk_status = WitInit(WIT_PROTOCOL_NORMAL, 0x50U);
    if (sdk_status == WIT_HAL_OK)
    {
        sdk_status = WitSerialWriteRegister(HWT6033_UartTransmit);
    }
    if (sdk_status == WIT_HAL_OK)
    {
        sdk_status = WitRegisterCallBack(HWT6033_RegisterUpdated);
    }
    if (sdk_status == WIT_HAL_OK)
    {
        sdk_status = WitDelayMsRegister(HWT6033_DelayMs);
    }

    if (sdk_status != WIT_HAL_OK)
    {
        WitDeInit();
        hwt6033_uart = NULL;
        hwt6033_initialized = 0U;
        return HWT6033_STATUS_ERROR;
    }

    hwt6033_initialized = 1U;
    return HWT6033_STATUS_OK;
}

void HWT6033_DeInit(void)
{
    uint32_t primask = HWT6033_EnterCritical();

    hwt6033_initialized = 0U;
    hwt6033_uart = NULL;
    WitDeInit();

    HWT6033_ExitCritical(primask);
}

HWT6033_Status_e HWT6033_FeedByte(uint8_t data)
{
    if (hwt6033_initialized == 0U)
    {
        return HWT6033_STATUS_NOT_INITIALIZED;
    }

    WitSerialDataIn(data);
    return HWT6033_STATUS_OK;
}

HWT6033_Status_e HWT6033_FeedData(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    if ((data == NULL) && (length != 0U))
    {
        return HWT6033_STATUS_INVALID_ARGUMENT;
    }
    if (hwt6033_initialized == 0U)
    {
        return HWT6033_STATUS_NOT_INITIALIZED;
    }

    /* 官方解析器按字节维护帧状态，可连续处理拆包、粘包和多帧输入。 */
    for (index = 0U; index < length; ++index)
    {
        WitSerialDataIn(data[index]);
    }

    return HWT6033_STATUS_OK;
}

HWT6033_Status_e HWT6033_CopyData(HWT6033_Data_t *data)
{
    uint32_t primask;

    if (data == NULL)
    {
        return HWT6033_STATUS_INVALID_ARGUMENT;
    }
    if (hwt6033_initialized == 0U)
    {
        return HWT6033_STATUS_NOT_INITIALIZED;
    }

    /* 数据可能在 UART 中断中刷新，短临界区保证结构体快照一致。 */
    primask = HWT6033_EnterCritical();
    *data = hwt6033_data;
    HWT6033_ExitCritical(primask);

    return HWT6033_STATUS_OK;
}

uint32_t HWT6033_TakeUpdateFlags(void)
{
    uint32_t flags;
    uint32_t primask = HWT6033_EnterCritical();

    flags = hwt6033_data.update_flags;
    hwt6033_data.update_flags = 0U;

    HWT6033_ExitCritical(primask);
    return flags;
}

HAL_StatusTypeDef HWT6033_GetLastTransmitStatus(void)
{
    return hwt6033_last_tx_status;
}

/* Private functions -------------------------------------------------------- */
/**
 * @brief 官方 SDK 的串口发送回调
 * @param data 待发送字节
 * @param length 发送长度
 * @return 无
 * @note 该回调使用阻塞 HAL 发送，配置和校准接口只能从任务上下文调用。
 */
static void HWT6033_UartTransmit(uint8_t *data, uint32_t length)
{
    if ((hwt6033_uart == NULL) || (data == NULL) || (length > UINT16_MAX))
    {
        hwt6033_last_tx_status = HAL_ERROR;
        return;
    }

    hwt6033_last_tx_status = HAL_UART_Transmit(hwt6033_uart,
                                               data,
                                               (uint16_t)length,
                                               HWT6033_UART_TX_TIMEOUT_MS);
}

/**
 * @brief 官方 SDK 的毫秒延时回调
 * @param delay_ms 延时时间
 * @return 无
 */
static void HWT6033_DelayMs(uint16_t delay_ms)
{
    HAL_Delay(delay_ms);
}

/**
 * @brief 将官方寄存器缓存更新为工程侧物理量
 * @param register_start 更新区间起始寄存器
 * @param register_count 连续更新的寄存器数量
 * @return 无
 */
static void HWT6033_RegisterUpdated(uint32_t register_start, uint32_t register_count)
{
    uint32_t axis;
    uint32_t flags = 0U;

    if (HWT6033_RangeOverlaps(register_start, register_count, AX, 3U) != 0U)
    {
        for (axis = 0U; axis < 3U; ++axis)
        {
            hwt6033_data.acceleration_g[axis] =
                (float)sReg[AX + axis] * HWT6033_ACCELERATION_SCALE_G;
        }
        flags |= HWT6033_UPDATE_ACCELERATION;
    }

    if (HWT6033_RangeOverlaps(register_start, register_count, GX, 3U) != 0U)
    {
        for (axis = 0U; axis < 3U; ++axis)
        {
            hwt6033_data.angular_velocity_dps[axis] =
                (float)sReg[GX + axis] * HWT6033_ANGULAR_VELOCITY_SCALE_DPS;
        }
        flags |= HWT6033_UPDATE_ANGULAR_VELOCITY;
    }

    if (HWT6033_RangeOverlaps(register_start, register_count, Roll, 3U) != 0U)
    {
        for (axis = 0U; axis < 3U; ++axis)
        {
            hwt6033_data.angle_deg[axis] =
                (float)sReg[Roll + axis] * HWT6033_ANGLE_SCALE_DEG;
        }
        flags |= HWT6033_UPDATE_ANGLE;
    }

    if (HWT6033_RangeOverlaps(register_start, register_count, TEMP, 1U) != 0U)
    {
        hwt6033_data.temperature_c = (float)sReg[TEMP] * HWT6033_TEMPERATURE_SCALE_C;
        flags |= HWT6033_UPDATE_TEMPERATURE;
    }

    if (flags != 0U)
    {
        hwt6033_data.update_flags |= flags;
        hwt6033_data.last_update_ms = DWT_GetTimeMs();
    }
}

/**
 * @brief 判断两个连续寄存器区间是否相交
 * @return 1 表示相交，0 表示不相交或区间为空
 */
static uint8_t HWT6033_RangeOverlaps(uint32_t update_start,
                                     uint32_t update_count,
                                     uint32_t target_start,
                                     uint32_t target_count)
{
    uint32_t update_end;
    uint32_t target_end;

    if ((update_count == 0U) || (target_count == 0U))
    {
        return 0U;
    }

    update_end = update_start + update_count - 1U;
    target_end = target_start + target_count - 1U;
    return ((update_start <= target_end) && (target_start <= update_end)) ? 1U : 0U;
}

/**
 * @brief 进入可嵌套的短临界区并返回原 PRIMASK
 */
static uint32_t HWT6033_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/**
 * @brief 按进入临界区前的状态恢复中断
 */
static void HWT6033_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/* -------------------------------------------------------------------------- */
